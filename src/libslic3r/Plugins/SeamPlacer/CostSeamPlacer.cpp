///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "CostSeamPlacer.hpp"
#include "SeamCandidatePreparation.hpp"
#include "SeamInitializationCancellation.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

#include "SeamPlacementConstraints.hpp"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/SeamPlacerViews.hpp"
#include "libslic3r/Plugins/SeamPlacer/SeamCostModel.hpp"
#include "libslic3r/Api/plugin/cpp/MeshQueryViews.hpp"

/*
Cost-based seam placement
=========================
The router runs this provider once per assigned PrintRegion batch. run_impl()
uses make_seam_placer_instance() to initialize a session from the PrintingPlan
before publishing it to the extrusion orderer. Preparation follows plan order
and stores candidates in scaled object-local XY by subtracting the instance
shift from scaled platter XY. Effective external-perimeter loops supply
loop-owned candidates, and every bound perimeter loop is a projection target;
a genuine internal perimeter uses the island fallback assembled from finalized
external-loop sets. A loop may have both roles. Painted constraints and
neighboring-region boundaries add endpoints and analytically located transition
points, rounded to scaled integer coordinates and reclassified there.

Each initialize() call collects and validates loops, then builds one visibility
model per object requiring visibility in this session. These local models are
destroyed before initialize() returns: the frozen plan's object handles identify
geometry shared by layers and translated instances. All models remain resident
while each external loop is filtered and scored independently. Cache construction
and perimeter interpolation each report their own 0..100% progress, aggregated
over that phase's objects or loops, with a reset when interpolation begins.

    CostSession::initialize()
    |-- collect and validate external loops and regional visibility weights
    |-- VisibilityModel::build(): cache samples and ray costs by object handle
    `-- VisibilityModel::calculate(): interpolate filtered loop candidates
Placement projects those candidates onto the requested loop and combines the
snapshot with the current travel-cost setting. The orderer queries approach
probes, then the actual approach position before rotating a loop to its seam.
Published maps are read-only; the host data read by concurrent placement calls
must not be mutated concurrently. Source changes do not refresh the snapshots.
*/

namespace slic3r_api { namespace SeamPlacement { namespace CostPlugin {

struct LoopBinding {
    RegionIslandKey island;
    c_point instance_shift;
    bool distance_only = false;
    bool has_external_geometry = false;
};

/*
The session has a build phase followed by a read-only placement phase.
m_storage is borrowed scratch storage used only by initialize(); its handle is
cleared after successful preparation, before ABI publication, without freeing
host storage. Region/island handles and loop keys remain borrowed: the plan and
referenced host objects must outlive the session. Candidate vectors and instance
shifts are owned values. Concurrent placement uses local scratch state and never
mutates these maps; this does not freeze host geometry between placement calls.
*/
class CostSession final : public SeamPlacerSession
{
public:
    explicit CostSession(storage_handle *storage,
                         const plugin_run_context *run_context = nullptr) :
        m_storage(storage), m_run_context(run_context) {}
    void initialize(const PrintingPlan &plan, const std::vector<PrintRegion> &regions) override;
    // Cost has no regional preference; the router keeps the first region on equal priorities.
    float get_priority(const ExtrusionEntity &, const LayerRegion &) const override { return 50.f; }
    c_point place_seam(const ExtrusionEntity &loop, const LayerRegion &region,
                       c_point start_position) const override;

private:
    // Aggregate one object's or perimeter's fraction within its named 0..100% phase.
    void report_visibility(const char *stage, size_t completed, size_t total,
                           double fraction) const;
    storage_handle *m_storage;
    const plugin_run_context *m_run_context;
    bool m_initialization_started = false;
    std::set<const print_region_handle *> m_regions;
    std::map<const extrusion_entity_handle *, std::vector<SeamCandidate>> m_loop_candidates;
    std::map<RegionIslandKey, std::vector<SeamCandidate>> m_fallback_candidates;
    std::map<const extrusion_entity_handle *, LoopBinding> m_bindings;
};

namespace {

const char *const k_plugin_id = "seam_placer.cost";
const char *const k_group_id = "seam_placer_plugin";
const char *const k_dependencies[] = { nullptr };
const char *const k_seam_angle_cost_key = "seam_angle_cost";
const char *const k_seam_travel_cost_key = "seam_travel_cost";
const char *const k_seam_visibility_key = "seam_visibility";
const raw_used_config_key k_used_config_keys[] = {
    { k_seam_angle_cost_key, RAW_CO_PERCENT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT },
    { k_seam_travel_cost_key, RAW_CO_PERCENT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT },
    { k_seam_visibility_key, RAW_CO_BOOL, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT }
};
const char *const k_defined_config_keys[] = {
    k_seam_angle_cost_key,
    k_seam_travel_cost_key,
    k_seam_visibility_key
};
constexpr int32_t k_defined_config_key_count = 3;

/* Mutable preparation data: point is scaled object-local XY; width, overhang
   and embedded are millimetres. Filtering reduces these records to SeamCandidate. */
/* Temporary work keeps each external loop's candidates separate. Borrowed island
   and object handles identify shared fallback inputs and visibility geometry. */
using LoopKey = const extrusion_entity_handle *;

struct ExternalLoopWork {
    LoopKey loop = nullptr;
    std::vector<CandidatePreparation::Candidate> candidates;
};

struct IslandWork {
    LayerRegionIsland island;
    const object_handle *object = nullptr;
    double visibility_weight = 0.0;
    bool has_perimeter_loop = false;
    bool requires_island_fallback = false;
    std::vector<ExternalLoopWork> external_loops;
};

class CostSeamPlacer final : public PluginBase
{
public:
    static CostSeamPlacer &instance(orchestrator_handle *orchestrator);
    explicit CostSeamPlacer(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}
private:
    const char *id_impl() const noexcept override { return k_plugin_id; }
    const char *name_impl() const noexcept override { return "Cost-based"; }
    const char *description_impl() const noexcept override { return "Balances seam visibility, corner shape, overhang quality and travel."; }
    const char *exclusive_group_impl() const noexcept override { return k_group_id; }
    slicing_step_t step_impl() const noexcept override { return SEAM_PLACER; }
    const char *const *dependencies_impl() const noexcept override { return k_dependencies; }
    // Return the common seam execution priority. Nothing is changed or transferred, and the call cannot fail.
    int32_t priority_impl() const noexcept override { return 600; }
    // Return this seam algorithm's preference. Nothing is changed or transferred, and the call cannot fail.
    int32_t exclusive_group_priority_impl() const noexcept override { return 0; }
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    int32_t defined_config_keys(const char **keys) const noexcept override;
    void inilialize_impl(storage_handle *) const override;
    void run_impl(const plugin_run_context *run_context) const override;
};

/* Register a percentage setting owned by this provider. */
void register_cost_option(orchestrator_handle *orchestrator,
                          const raw_used_config_key &config_key,
                          const char *label,
                          const char *full_label,
                          const char *tooltip,
                          const char *default_serialized_value)
{
    raw_config_option_def def = raw_config_option_def_init();
    def.opt_key = config_key.key;
    def.type = config_key.type;
    def.container_type = config_key.container_type;
    def.option_preset_type = config_key.option_preset_type;
    def.printer_technology = RAW_PT_FFF;
    def.label = label;
    def.full_label = full_label;
    def.category = RAW_OPTION_CATEGORY_PERIMETER;
    def.invalidates_step = STEP_ORDERING;
    def.tooltip = tooltip;
    if (def.type == RAW_CO_PERCENT) {
        def.sidetext = "%";
        def.has_min = 1;
        def.min_value = 0;
        def.has_max = 1;
        def.max_value = 1000;
    }
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = default_serialized_value;
    if (orchestrator_create_option_def(orchestrator, &def) != OPTION_DEF_ERROR_OK)
        throw std::runtime_error("Cannot register seam cost configuration.");
}

/* Return scaled travel distance; placement converts it to millimetres before
   applying the Gaussian travel penalty. */
double point_distance(c_point a, c_point b)
{
    return std::hypot(double(a.x) - b.x, double(a.y) - b.y);
}

} // namespace

/*
Prepare once, including failed attempts: the guard is set before input validation
and a failure does not roll back this session's partially built state. The normal
make_seam_placer_instance() path destroys it on failure and publishes only after
success. Constraint caches copy their polygons; they and the mutable island work
are local temporaries, not part of the published session.
*/
void CostSession::initialize(const PrintingPlan &plan, const std::vector<PrintRegion> &regions)
{
    if (m_initialization_started)
        throw std::logic_error(
            "Cost-based seam placement sessions may only be initialized once.");
    m_initialization_started = true;
    InitializationCancellation::check(m_run_context);
    if (!plan.valid() || m_storage == nullptr)
        throw std::invalid_argument("Cost-based seam placement needs a valid plan and temporary storage.");
    SeamPlacementConstraints constraints;
    constraints.initialize_for_regions(plan, m_storage, regions, m_run_context);
    std::map<RegionIslandKey, IslandWork> work;
    for (const PrintRegion &region : regions)
        if (region.valid()) m_regions.insert(region.handle());

    /*
    Traverse the plan's group/layer/tool/extrusion order. Invalid island metadata
    is tolerated only when the extrusion has no perimeter loop, and an island is
    prepared when any of its PrintRegions belongs to this provider session.
    */
    for (uint32_t group_index = 0; group_index < plan.group_count(); ++group_index) {
        InitializationCancellation::check(m_run_context);
        const ConstPrintingGroup group = plan.group(group_index);
        for (uint32_t layer_index = 0; layer_index < group.layer_group_count(); ++layer_index) {
            const ConstPrintingLayerGroup layer_group = group.layer_group(layer_index);
            for (uint32_t tool_index = 0; tool_index < layer_group.tool_group_count(); ++tool_index) {
                const ConstPrintingToolGroup tool = layer_group.tool_group(tool_index);
                for (uint32_t extrusion_index = 0; extrusion_index < tool.extrusion_count(); ++extrusion_index) {
                    InitializationCancellation::check_periodic(
                        m_run_context, extrusion_index);
                    const ConstPrintingExtrusion extrusion = tool.extrusion(extrusion_index);
                    const LayerRegionIsland island = extrusion.region_island();
                    const std::vector<CandidatePreparation::CollectedLoop> collected =
                        CandidatePreparation::collect_perimeter_loops(
                            extrusion.root(), extrusion.role(), m_run_context);
                    if (!island.valid() || island.region_count() == 0) {
                        if (!collected.empty())
                            throw std::runtime_error(
                                "Cost-based seam placement found a perimeter loop with an invalid source LayerRegionIsland.");
                        continue;
                    }
                    bool assigned = false;
                    for (uint32_t region_index = 0; region_index < island.region_count(); ++region_index) {
                        const PrintRegion print_region = island.region(region_index).print_region();
                        if (print_region.valid() && m_regions.count(print_region.handle()) != 0) {
                            assigned = true;
                            break;
                        }
                    }
                    if (!assigned) continue;
                    const c_point shift = plan.object_instance_shift(extrusion);
                    const RegionIslandKey key = island.handle();
                    const bool first = work.find(key) == work.end();
                    IslandWork &island_work = work[key];
                    if (first) {
                        island_work.island = island;
                        island_work.object = island.region(0).layer().object().handle();
                    }
                    for (const CandidatePreparation::CollectedLoop &loop : collected)
                        m_bindings.emplace(
                            loop.loop, LoopBinding{key, shift, loop.distance_only,
                                                   loop.has_external_geometry});
                    island_work.has_perimeter_loop = island_work.has_perimeter_loop || !collected.empty();
                    for (const CandidatePreparation::CollectedLoop &source : collected)
                        if (!source.distance_only && !source.has_external_geometry)
                            island_work.requires_island_fallback = true;
                    if (collected.empty()) continue;
                    // Assignment checks all regions, but region(0) supplies the
                    // fallback flow and external-perimeter nozzle parameters.
                    const LayerRegion region = island.region(0);
                    const c_flow external_flow = region.flow(RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
                    const double fallback_width = unscaled(external_flow.width);
                    for (const CandidatePreparation::CollectedLoop &source : collected) {
                        if (source.distance_only || !source.has_external_geometry) continue;
                        const ExtrusionEntity loop(source.loop);
                        const double width = source.external_width > 0 ?
                            source.external_width : fallback_width;
                        ExternalLoopWork external_loop;
                        external_loop.loop = source.loop;
                        external_loop.candidates =
                            CandidatePreparation::prepare_loop_candidates(
                                m_storage, constraints, island_work.island,
                                loop, region, shift, width,
                                unscaled(external_flow.nozzle_diameter),
                                m_run_context);
                        island_work.external_loops.push_back(std::move(external_loop));
                    }
                }
            }
        }
    }

    size_t total_models = 0;
    size_t total_visibility_loops = 0;
    std::set<const object_handle *> model_objects;
    // Validate known inputs before paying for any object's ray model.
    for (auto &[key, island_work] : work) {
        if (!island_work.has_perimeter_loop || island_work.external_loops.empty())
            continue;
        for (const ExternalLoopWork &loop : island_work.external_loops)
            if (loop.candidates.empty())
                throw std::runtime_error(
                    "Cost-based seam placement found an external perimeter without candidates.");
        for (const LayerRegion &region : island_work.island.regions())
            if (region.print_region().config().bool_or_default(k_seam_visibility_key, false)) {
                island_work.visibility_weight = 1.0;
                break;
            }
        if (island_work.visibility_weight == 0.0) continue;
        total_visibility_loops += island_work.external_loops.size();
        if (model_objects.insert(island_work.object).second) ++total_models;
    }
    const mesh_query_cancel cancel{
        // The C callback carries mutable void*, but only reads this run context.
        const_cast<plugin_run_context *>(m_run_context),
        [](void *opaque) -> int32_t {
            return is_cancelled(static_cast<const plugin_run_context *>(opaque)) ? 1 : 0;
        }};
    // Object handles stay valid while the plan and geometry are frozen here.
    // The cache never survives initialize(), so no invalidation is needed.
    std::map<const object_handle *, std::unique_ptr<Slic3r::SeamCostModel::VisibilityModel>> models;
    size_t completed_models = 0;
    for (auto &[key, island_work] : work) {
        if (island_work.visibility_weight == 0.0 || island_work.external_loops.empty() ||
            models.count(island_work.object) != 0)
            continue;
        const Object object = island_work.island.region(0).layer().object();
        std::unique_ptr<Slic3r::SeamCostModel::VisibilityModel> model(
            new Slic3r::SeamCostModel::VisibilityModel());
        try {
            model->build(object, m_run_context == nullptr ? nullptr : &cancel,
                [&](double fraction) {
                    report_visibility("Building seam visibility cache: object",
                        completed_models, total_models, fraction);
                });
        } catch (const MeshQueryError &error) {
            if (error.status == MESH_QUERY_CANCELLED) throw PluginCancelled();
            throw;
        }
        models.emplace(island_work.object, std::move(model));
        ++completed_models;
    }

    /*
    Finalize each external loop independently: enrich and filter its records,
    snapshot static costs, then deduplicate equal integer points. Equal points
    retain discovery position and the lowest static cost within that loop. An
    internal perimeter receives a simple concatenation of the finalized vectors.
    */
    size_t completed_loops = 0;
    for (auto &[key, island_work] : work) {
        InitializationCancellation::check(m_run_context);
        if (!island_work.has_perimeter_loop) continue;
        if (island_work.external_loops.empty()) {
            if (island_work.requires_island_fallback)
                throw std::runtime_error(
                    "Cost-based seam placement found perimeter loops but no external geometric candidate for LayerRegionIsland.");
            continue;
        }
        const Layer layer = island_work.island.region(0).layer();
        // Compute shared regional weights once; candidate enrichment and scoring
        // below still run independently for every external loop.
        double angle_weight = 0.0;
        const double visibility_weight = island_work.visibility_weight;
        for (const LayerRegion &lr : island_work.island.regions()) {
            const Config &region_config = lr.print_region().config();
            angle_weight += region_config.effective_float_or_percent_or_default(
                k_seam_angle_cost_key, 1.0, 0.0);
        }
        angle_weight = angle_weight / island_work.island.region_count();
        std::vector<SeamCandidate> fallback;
        for (ExternalLoopWork &external_loop : island_work.external_loops) {
            InitializationCancellation::check(m_run_context);
            CandidatePreparation::calculate_layer_attributes(
                island_work.island, external_loop.candidates, m_run_context);
            CandidatePreparation::filter_candidates(
                external_loop.candidates, m_run_context);
            if (external_loop.candidates.empty())
                throw std::runtime_error(
                    "Cost-based seam placement found an external perimeter without candidates.");
            // Interpolation queries use object-local XY and the legacy slice-Z
            // convention in millimetres. Query the shared model per loop to keep
            // visibility scores in the same order as that loop's candidate records.
            std::vector<float> visibility;
            if (visibility_weight != 0) {
                std::vector<c_vec3f> points;
                points.reserve(external_loop.candidates.size());
                for (const CandidatePreparation::Candidate &candidate :
                         external_loop.candidates)
                    points.push_back({float(unscaled(candidate.point.x)),
                                      float(unscaled(candidate.point.y)),
                                      float(unscaled(layer.slice_z()))});
                try {
                    visibility = models.at(island_work.object)->calculate(
                        points, m_run_context == nullptr ? nullptr : &cancel,
                        [&](double fraction) {
                            report_visibility("Calculating seam visibility: perimeter",
                                completed_loops, total_visibility_loops, fraction);
                        });
                } catch (const MeshQueryError &error) {
                    if (error.status == MESH_QUERY_CANCELLED)
                        throw PluginCancelled();
                    throw;
                }
                ++completed_loops;
            }
            std::vector<SeamCandidate> prepared;
            prepared.reserve(external_loop.candidates.size());
            for (size_t index = 0; index < external_loop.candidates.size(); ++index) {
                InitializationCancellation::check_periodic(
                    m_run_context, index);
                const CandidatePreparation::Candidate &candidate =
                    external_loop.candidates[index];
                const double overhang_penalty = candidate.overhang > 0 && candidate.width > 0 ?
                    std::clamp(2 * (candidate.overhang - candidate.width / 8) /
                                   candidate.width, 0.0, 3.0) : 0.0;
                const double visibility_cost = index < visibility.size() ?
                    visibility[index] : 0.0;
                const double static_cost = overhang_penalty +
                    visibility_weight * visibility_cost + angle_weight *
                    Slic3r::SeamCostModel::compute_angle_penalty(
                        float(candidate.angle));
                std::vector<SeamCandidate>::iterator duplicate =
                    std::find_if(prepared.begin(), prepared.end(),
                        [&](const SeamCandidate &entry) {
                            return entry.point.x == candidate.point.x &&
                                   entry.point.y == candidate.point.y;
                        });
                if (duplicate == prepared.end())
                    prepared.push_back({candidate.point, static_cost});
                else
                    duplicate->static_cost = std::min(
                        duplicate->static_cost, static_cost);
            }
            if (island_work.requires_island_fallback)
                fallback.insert(fallback.end(), prepared.begin(), prepared.end());
            m_loop_candidates.emplace(external_loop.loop, std::move(prepared));
        }
        if (island_work.requires_island_fallback) {
            if (fallback.empty())
                throw std::runtime_error(
                    "Cost-based seam placement found perimeter loops but no external geometric candidate for LayerRegionIsland.");
            m_fallback_candidates.emplace(key, std::move(fallback));
        }
    }
    m_storage = nullptr;
    m_run_context = nullptr;
}

void CostSession::report_visibility(const char *stage, size_t completed, size_t total,
                                    double fraction) const
{
    if (m_run_context == nullptr || total == 0) return;
    const double percent = (double(completed) + fraction) / double(total);
    const std::string message = std::string(stage) +
        " " + std::to_string(completed + 1) + "/" + std::to_string(total);
    slic3r_api::report_progress(m_run_context, percent, message.c_str());
}

c_point CostSession::place_seam(const ExtrusionEntity &loop, const LayerRegion &region,
                                c_point start_position) const
{
    /*
    The router resolves a compatible loop/region pair before this call. These
    checks validate region assignment and loop binding separately; they do not
    fully validate that relationship for direct callers. Missing candidates are
    an error, not an implicit fallback to another provider.
    */
    if (!loop.is_loop() || loop.empty())
        throw std::invalid_argument("Cost-based seam placement needs a non-empty closed loop.");
    if (!region.valid()) throw std::invalid_argument("Cost-based seam placement needs a valid LayerRegion.");
    const PrintRegion print_region = region.print_region();
    if (!print_region.valid() || m_regions.count(print_region.handle()) == 0)
        throw std::invalid_argument("Cost seam LayerRegion was not assigned to this provider session.");
    const std::map<const extrusion_entity_handle *, LoopBinding>::const_iterator binding =
        m_bindings.find(loop.handle());
    if (binding == m_bindings.end()) throw std::invalid_argument("Cost seam loop was not present in the initialized plan.");
    if (binding->second.distance_only) {
        const std::pair<c_point, double> projection =
            CandidatePreparation::project_onto_loop(loop, start_position);
        if (!std::isfinite(projection.second))
            throw std::runtime_error(
                "Cost-based seam placement could not project the approach position onto a distance-only loop.");
        return projection.first;
    }
    const std::vector<SeamCandidate> *candidates = nullptr;
    if (binding->second.has_external_geometry) {
        const std::map<const extrusion_entity_handle *,
                       std::vector<SeamCandidate>>::const_iterator own =
            m_loop_candidates.find(loop.handle());
        if (own == m_loop_candidates.end() || own->second.empty())
            throw std::runtime_error("Cost seam external loop has no candidates.");
        candidates = &own->second;
    } else {
        const std::map<RegionIslandKey,
                       std::vector<SeamCandidate>>::const_iterator fallback =
            m_fallback_candidates.find(binding->second.island);
        if (fallback == m_fallback_candidates.end() || fallback->second.empty())
            throw std::runtime_error("Cost seam source island has no candidates.");
        candidates = &fallback->second;
    }
    // Unlike static costs, this weight is read per request from borrowed host
    // configuration, which must not change concurrently with these reads.
    const double travel_weight = region.print_region().config().effective_float_or_percent_or_default(
        k_seam_travel_cost_key, 1.0, 0.0);
    c_point best{};
    double best_cost = std::numeric_limits<double>::infinity();
    /*
    Reapply the loop instance shift to each object-coordinate candidate, project
    it onto the requested loop in scaled platter XY, and measure distance to the
    scaled start_position (an approach probe or the actual approach). Convert that
    distance to mm before the Gaussian travel penalty, then add the weighted score,
    not a length. Strict comparison preserves the first winner on equal total cost.
    The projected point is not reclassified against paint or regional boundaries;
    those preferences describe the source candidate, not necessarily its projection.
    */
    for (const SeamCandidate &candidate : *candidates) {
        const c_point target = {candidate.point.x + binding->second.instance_shift.x,
                                candidate.point.y + binding->second.instance_shift.y};
        const std::pair<c_point, double> projection =
            CandidatePreparation::project_onto_loop(loop, target);
        const double distance_mm = unscaled(point_distance(projection.first, start_position));
        const double cost = candidate.static_cost + travel_weight *
            (1.0 - Slic3r::SeamCostModel::gauss(float(distance_mm), 0.f, 1.f, 0.005f));
        if (cost < best_cost) { best = projection.first; best_cost = cost; }
    }
    return best;
}

namespace {

CostSeamPlacer &CostSeamPlacer::instance(orchestrator_handle *orchestrator)
{
    static CostSeamPlacer plugin(orchestrator);
    return plugin;
}

int32_t CostSeamPlacer::used_config_keys(raw_used_config_key *keys) const noexcept
{
    if (keys != nullptr)
        for (size_t index = 0; index < std::size(k_used_config_keys); ++index) keys[index] = k_used_config_keys[index];
    return int32_t(std::size(k_used_config_keys));
}

int32_t CostSeamPlacer::defined_config_keys(const char **keys) const noexcept
{
    if (keys != nullptr)
        for (size_t index = 0; index < std::size(k_defined_config_keys); ++index)
            keys[index] = k_defined_config_keys[index];
    return k_defined_config_key_count;
}

void CostSeamPlacer::inilialize_impl(storage_handle *) const
{
    register_cost_option(
        m_orchestrator,
        k_used_config_keys[0],
        "Angle cost",
        "Seam angle cost",
        "Cost of placing the seam at a bad angle. The worst angle is flat.\n100% is the default penalty.",
        "60");
    register_cost_option(
        m_orchestrator,
        k_used_config_keys[1],
        "Travel cost",
        "Seam travel cost",
        "Cost of moving the extruder. The highest penalty is assigned when the seam is furthest from the approach position.",
        "100");
    register_cost_option(
        m_orchestrator,
        k_used_config_keys[2],
        "use visibility check",
        "Seam visibility check",
        "Check and penalize seams that are the most visible. Launch rays to check from how many directions a point is visible.\nThis is a compute-intensive option.",
        "1");
}

void CostSeamPlacer::run_impl(const plugin_run_context *run_context) const
{
    /*
    Validate the raw C context and copy non-owning region wrappers. The factory
    creates a session; make_seam_placer_instance() initializes it, then transfers
    its unique_ptr into the ABI holder. The router's SeamPlacer wrapper consumes
    that instance and eventually invokes its destroy callback. The context only
    carries the output; neither it nor the session owns the referenced host graph.
    Placement callbacks translate exceptions into C error results; the consuming
    C++ wrapper raises errors and checks the returned point against the loop path.
    */
    run_ctx_seam_placer *context = plugin_ctx_as_seam_placer(run_context);
    if (context == nullptr || context->plan == nullptr || (context->region_count != 0 && context->regions == nullptr))
        throw std::invalid_argument("Cost-based seam placement received an invalid run context.");
    std::vector<PrintRegion> regions;
    for (uint32_t index = 0; index < context->region_count; ++index) regions.emplace_back(context->regions[index]);
    InitializationCancellation::check(run_context);
    context->instance = make_seam_placer_instance(
        make_cost_seam_session(run_context->plugin_storage, run_context),
        PrintingPlan(context->plan), regions);
}

} // namespace

std::unique_ptr<SeamPlacerSession> make_cost_seam_session(storage_handle *storage)
{
    return std::unique_ptr<SeamPlacerSession>(new CostSession(storage));
}

std::unique_ptr<SeamPlacerSession> make_cost_seam_session(
    storage_handle *storage, const plugin_run_context *run_context)
{
    return std::unique_ptr<SeamPlacerSession>(
        new CostSession(storage, run_context));
}

void register_cost_seam_placer_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(orchestrator, CostSeamPlacer::instance(orchestrator).c_instance());
}

}}} // namespace slic3r_api::SeamPlacement::CostPlugin
