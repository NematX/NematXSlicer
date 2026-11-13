///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Perimeter-avoiding travel generation
===================================

This provider supplies XY paths for gaps between ordered extrusion leaves in a
PrintingPlan. ScopeTravelConnector, defined in TravelConnectionHelpers.hpp,
finds those gaps and inserts travels into the reserved phases. It also handles
epsilon snapping, Z interpolation and final travel flags; this file chooses
only the XY route.

setup_impl resets the connector and allocates its per-layer summaries.
setup_run_impl prepares one layer group's summary before run_impl connects its
scopes. The host finishes all setup_run_impl callbacks before starting run_impl
callbacks, so a run can read the next non-empty layer's summary to set its
outgoing flag. Each run owns a planner for that layer group. Both endpoints must enable
avoid_crossing_perimeters; a failed regional lookup retries at the interior
probe supplied by the connector. If avoid_crossing_not_first_layer is enabled,
the target's first layer is excluded, including layers at the same print Z.
If either endpoint has no region island or that island has no regions, the
planner keeps straight travel.
Endpoints without a valid object instance, including global skirt and brim,
keep the direct travel.

The planner copies the target's geometry, region settings and averaged spacing
through public API views, then
subtracts the target instance shift from both endpoints. PerimeterCrossing tests
the direct path against boundaries of prepared interior areas, not extrusion
centerlines. Without a crossing, the planner keeps the straight path.

TravelRouter uses internal boundaries for travel within one object instance and
external boundaries for travel between instances or objects. The external input
is copied only when requested. The router may keep a direct path when its
heuristic cannot find an acceptable detour or the extra length exceeds the
configured limit; it does not guarantee a shortest or crossing-free path.
The planner restores the instance shift and exact original endpoints before
returning XY points to the connector.

When both travel providers are active, this provider is the group default;
an explicit selector can still choose straight travel.

Each run owns its copied inputs and one router and crossing cache per target
layer/tool pair. Inputs outlive the routers borrowing them. Temporary geometry
operations use the run context's plugin storage. Planners do not escape the
worker callback, so their mutable caches are not shared between workers.

    setup_impl: reset and allocate connector summaries
    setup_run_impl: prepare the current layer group's summary
    run_impl: own the planner for this worker callback
    `-- ScopeTravelConnector::run: find gaps and insert the returned XY paths
        `-- LayerTravelPlanner::path: select straight travel or a detour
            |-- endpoint_enables_avoidance: check each endpoint's regional setting
            |-- input_for: obtain the target layer input
            |   `-- LayerInputCache::get: retain its copied geometry, settings and spacing
            |-- router_for: retain layer/tool state and defer external input
            |-- PerimeterCrossing::prepare/can_cross_perimeter: test the direct path
            `-- TravelRouter::travel_to: choose a route within the detour budget
*/

#include "AvoidCrossingPerimetersTravel.hpp"
#include "PerimeterTravelPlanner.hpp"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/RegionSettingsViews.hpp"
#include "AvoidCrossing/PerimeterCrossing.hpp"
#include "AvoidCrossing/LayerInputCache.hpp"
#include "AvoidCrossing/TravelRouter.hpp"
#include "TravelConnectionHelpers.hpp"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace AvoidCrossingPerimetersTravelPlugin {
// Register the permanent borrowed plugin and copy its alternative-selection setting description into the application.
// Print data is unchanged; allocation failures or rejected descriptions throw.
void register_avoid_crossing_perimeters_travel_plugin(orchestrator_handle *orchestrator);

namespace {

using namespace TravelConnection;
namespace AC = AvoidCrossing;
namespace Work = PluginCommon::work;

// Publish the router inputs read by both route generation and retraction planning.
int32_t perimeter_travel_config_keys(raw_used_config_key *keys) noexcept;

const char *const k_dependencies[] = {
    "layer_extrusion_edit.transition_scope.default",
    "layer_extrusion_edit.entry_state.default",
    nullptr
};

const char *const k_region_setting_keys[] = {
    "avoid_crossing_perimeters"
};

/* Identify one reusable router inside a layer-group worker. */
struct RouterKey
{
    const layer_handle *layer = nullptr;
    extruder_index_t extruder_id = EXTRUDER_ID_UNKNOWN;

    // Order borrowed public layer handles and zero-based tools for worker-local lookup.
    bool operator<(const RouterKey &rhs) const
    {
        return std::less<const layer_handle *>()(layer, rhs.layer) ||
            (layer == rhs.layer && extruder_id < rhs.extruder_id);
    }
};

// Own one layer/tool's router and crossing cache on the heap because TravelBoundary cannot move.
struct RoutingState
{
    AC::TravelRouter router;
    AC::PerimeterCrossing crossing;
};

/*
Own the expensive routing state used by one parallel layer-group run.

The planner never escapes run_impl(), so its map and the mutable caches inside
TravelRouter and PerimeterCrossing require no synchronization.
*/
class LayerTravelPlanner
{
public:
    // Retain borrowed views of the print and its settings, plus the plan and run context, during this worker callback.
    LayerTravelPlanner(const Print &print,
                       const PrintingPlan &plan,
                       const plugin_run_context *run_ctx);

    // Return print-space routing for an eligible gap, otherwise its direct segment; cancellation propagates.
    std::vector<c_point> path(const TravelEndpoint &source,
                              const TravelEndpoint &target,
                              extruder_index_t extruder_id);

    // Classify a print-space route against the perimeter contours of the target layer. With
    // exclude_top_surfaces, its tops count as exposed: a route over a top crosses even without
    // reaching a contour. Missing or different object provenance counts as a crossing.
    bool crosses(const TravelEndpoint &source,
                 const TravelEndpoint &target,
                 extruder_index_t extruder_id,
                 const std::vector<c_point> &path,
                 bool exclude_top_surfaces);

private:
    // Resolve regional activation only for valid object instances, probing inside on a boundary.
    bool endpoint_enables_avoidance(const TravelEndpoint &endpoint) const;

    /* Convert a final-plan point into the source object's local coordinates. */
    c_point local_point(c_point point, const TravelEndpoint &endpoint) const;

    // Copy target data once after finding its normal or auxiliary index through public views; missing identities throw.
    const AC::Input &input_for(const Layer &layer);

    // Initialize routing state for one target/tool, borrowing the cached input and run storage; failures propagate.
    RoutingState &router_for(const Layer &layer, const AC::Input &input, extruder_index_t extruder_id);

    /* Return a straight path whenever the advanced provider cannot be used. */
    std::vector<c_point> straight(const TravelEndpoint &source,
                                  const TravelEndpoint &target,
                                  extruder_index_t extruder_id) const;

    Print m_print;
    Config m_print_config;
    const PrintingPlan &m_plan;
    const plugin_run_context *m_run_ctx = nullptr;
    AC::LayerInputCache m_inputs;
    std::map<RouterKey, std::unique_ptr<RoutingState>> m_routers;
    // Retraction filtering chooses whether tops are exposed, so it uses its own crossing caches
    // per layer, tool and top choice.
    std::map<std::pair<RouterKey, bool>, AC::PerimeterCrossing> m_perimeter_crossings;
};

class AvoidCrossingPerimetersTravel : public PluginBase
{
public:
    // Return the permanent provider instance borrowing the first orchestrator supplied.
    static AvoidCrossingPerimetersTravel &instance(orchestrator_handle *orchestrator);
    // Bind the provider and its connector to the borrowed orchestrator.
    explicit AvoidCrossingPerimetersTravel(orchestrator_handle *orchestrator);

private:
    // Return the static provider ID used by the travel selector.
    const char *id_impl() const noexcept override;
    // Return the static display name for the provider.
    const char *name_impl() const noexcept override;
    // Return the static description of perimeter-avoiding travel.
    const char *description_impl() const noexcept override;
    // Return the shared travel-generation group ID.
    const char *exclusive_group_impl() const noexcept override;
    // Select the layer-extrusion-edit step that owns travel insertion.
    slicing_step_t step_impl() const noexcept override;
    // Return the static null-terminated list of required scope and entry-state providers.
    const char *const *dependencies_impl() const noexcept override;
    // Run after the entry-state, retraction and wipe providers and before lift insertion.
    int32_t priority_impl() const noexcept override;
    // Prefer this provider when the travel selector is absent; explicit provider IDs take precedence.
    int32_t exclusive_group_priority_impl() const noexcept override;
    // Report key types for host validation and GUI activation. Call with null to get the count,
    // then supply a caller-owned array of that size; copied entries borrow static key strings.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    // Return the static progress format for connected layer trees.
    const char *progress_message_format_impl() const noexcept override;
    // Reset connector state and allocate summaries when the run has a print and plan.
    void setup_impl(const plugin_run_context *run_ctx, uint32_t run_count) const override;
    // Publish this layer group summary when setup and the layer context are valid.
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    // Connect this layer group through a worker-local planner; invalid contexts do nothing and cancellation or errors propagate.
    void run_impl(const plugin_run_context *run_ctx) const override;

    mutable ScopeTravelConnector m_connector;
    mutable bool m_setup_valid = false;
};

LayerTravelPlanner::LayerTravelPlanner(const Print &print,
                                       const PrintingPlan &plan,
                                       const plugin_run_context *run_ctx) :
    m_print(print),
    m_print_config(print.config()),
    m_plan(plan),
    m_run_ctx(run_ctx)
{}

c_point LayerTravelPlanner::local_point(const c_point point, const TravelEndpoint &endpoint) const
{
    const Layer source_layer = endpoint.region_island.region(0).layer();
    const Object source_object = source_layer.object();
    if (endpoint.printing_catalog_id == SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE ||
        endpoint.printing_catalog_id >= m_plan.object_instance_count())
        throw std::runtime_error("A travel endpoint refers to an invalid object instance.");
    const uint64_t object_instance_idx = m_plan.object_instance(endpoint.printing_catalog_id).object_instance_idx();
    if (object_instance_idx >= source_object.instance_count())
        throw std::runtime_error("A travel endpoint refers to an invalid object instance.");
    const c_point shift = source_object.instance_shift(static_cast<size_t>(object_instance_idx));
    return c_point{point.x - shift.x, point.y - shift.y};
}

bool LayerTravelPlanner::endpoint_enables_avoidance(const TravelEndpoint &endpoint) const
{
    if (!endpoint.region_island.valid() || endpoint.region_island.region_count() == 0)
        return false;

    // Global skirt and brim regions have no object instance for local coordinates.
    if (endpoint.printing_catalog_id == SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE ||
        endpoint.printing_catalog_id >= m_plan.object_instance_count())
        return false;
    const Object object = endpoint.region_island.region(0).layer().object();
    if (m_plan.object_instance(endpoint.printing_catalog_id).object_instance_idx() >= object.instance_count())
        return false;

    const c_point endpoint_point{endpoint.position.x, endpoint.position.y};
    RegionSettingsPointResult result = RegionSettings::lookup_at_point(
        endpoint.region_island, {k_region_setting_keys[0]}, local_point(endpoint_point, endpoint));
    if (result.found())
        return result.value.get_bool();

    // A point on a modifier boundary may belong to two regions. The probe was
    // sampled just inside the geometric leaf, so it identifies the side that
    // this endpoint actually enters or leaves.
    if (endpoint.has_interior_point) {
        result = RegionSettings::lookup_at_point(
            endpoint.region_island,
            {k_region_setting_keys[0]},
            local_point(endpoint.interior_point, endpoint));
        if (result.found())
            return result.value.get_bool();
    }
    return false;
}

const AC::Input &LayerTravelPlanner::input_for(const Layer &layer)
{
    return m_inputs.get(m_print, layer, m_run_ctx);
}

RoutingState &LayerTravelPlanner::router_for(
    const Layer &layer, const AC::Input &input, const extruder_index_t extruder_id)
{
    const RouterKey key{layer.handle(), extruder_id};
    const auto found = m_routers.find(key);
    if (found != m_routers.end())
        return *found->second;
    std::unique_ptr<RoutingState> state = std::make_unique<RoutingState>();
    const AC::LayerIdentity identity = input.target->identity;
    // Defer the all-object copy until the router needs an external boundary.
    state->router.init_layer(m_run_ctx->plugin_storage, input,
        [this, identity]() {
            throw_if_cancelled(m_run_ctx);
            return AC::build_external_input(m_print, identity);
        }, [this]() { throw_if_cancelled(m_run_ctx); });
    return *m_routers.emplace(key, std::move(state)).first->second;
}

std::vector<c_point> LayerTravelPlanner::straight(const TravelEndpoint &source,
                                                   const TravelEndpoint &target,
                                                   const extruder_index_t extruder_id) const
{
    return straight_path(source, target, extruder_id);
}

std::vector<c_point> LayerTravelPlanner::path(const TravelEndpoint &source,
                                              const TravelEndpoint &target,
                                              const extruder_index_t extruder_id)
{
    throw_if_cancelled(m_run_ctx);
    // Missing regional provenance is normal for support and print-level
    // auxiliary paths. Those gaps deliberately retain the direct travel.
    if (!source.region_island.valid() || source.region_island.region_count() == 0 ||
        !target.region_island.valid() || target.region_island.region_count() == 0)
        return straight(source, target, extruder_id);
    if (!endpoint_enables_avoidance(source) || !endpoint_enables_avoidance(target))
        return straight(source, target, extruder_id);

    const Layer source_layer_view = source.region_island.region(0).layer();
    const Layer target_layer_view = target.region_island.region(0).layer();
    const Object source_object = source_layer_view.object();
    const Object target_object = target_layer_view.object();
    const uint64_t target_instance_idx = m_plan.object_instance(target.printing_catalog_id).object_instance_idx();

    if (m_print_config.bool_or_default("avoid_crossing_not_first_layer", true) &&
        AC::is_first_layer(target_object, target_layer_view))
        return straight(source, target, extruder_id);
    const AC::Input &input = input_for(target_layer_view);

    const c_point target_shift = target_object.instance_shift(static_cast<size_t>(target_instance_idx));
    const Work::Point local_source{
        source.position.x - target_shift.x,
        source.position.y - target_shift.y};
    const Work::Point local_target{
        target.position.x - target_shift.x,
        target.position.y - target_shift.y};

    RoutingState &state = router_for(target_layer_view, input, extruder_id);
    AC::TravelRouter &router = state.router;
    const double nozzle_diameter =
        m_print_config.vector_float_or_default("nozzle_diameter", extruder_id, 0.4);
    const coord_t nozzle_radius = std::max<coord_t>(1, scale_i(nozzle_diameter * 0.5));
    throw_if_cancelled(m_run_ctx);
    state.crossing.prepare(m_run_ctx->plugin_storage, *input.target, extruder_id, nozzle_radius,
                           [this]() { throw_if_cancelled(m_run_ctx); });
    throw_if_cancelled(m_run_ctx);

    Work::Polyline direct;
    direct.points = {local_source, local_target};
    throw_if_cancelled(m_run_ctx);
    if (!state.crossing.can_cross_perimeter(direct, true))
        return straight(source, target, extruder_id);

    const bool same_object_and_instance =
        source_object.handle() == target_object.handle() &&
        source.printing_catalog_id == target.printing_catalog_id;
    throw_if_cancelled(m_run_ctx);
    router.use_external_mp(!same_object_and_instance);

    const c_float_or_percent max_detour = m_print_config.float_or_percent_or_default(
        "avoid_crossing_perimeters_max_detour", c_float_or_percent{0.0, 0});
    const AC::TravelContext travel_context{
        local_source,
        Work::Point{target_shift.x, target_shift.y},
        extruder_id,
        max_detour.value,
        max_detour.percent != 0
    };
    throw_if_cancelled(m_run_ctx);
    const Work::Polyline routed = router.travel_to(travel_context, local_target);
    throw_if_cancelled(m_run_ctx);
    if (routed.size() < 2)
        return straight(source, target, extruder_id);

    std::vector<c_point> path;
    path.reserve(routed.size());
    for (const Work::Point &point : routed.points) {
        throw_if_cancelled(m_run_ctx);
        path.push_back(c_point{point.x() + target_shift.x, point.y() + target_shift.y});
    }
    path.front() = c_point{source.position.x, source.position.y};
    path.back() = c_point{target.position.x, target.position.y};
    return path;
}

bool LayerTravelPlanner::crosses(const TravelEndpoint &source,
                                 const TravelEndpoint &target,
                                 const extruder_index_t extruder_id,
                                 const std::vector<c_point> &path,
                                 const bool exclude_top_surfaces)
{
    throw_if_cancelled(m_run_ctx);
    if (!source.region_island.valid() || !target.region_island.valid() ||
        source.region_island.region_count() == 0 || target.region_island.region_count() == 0 ||
        source.printing_catalog_id == SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE ||
        source.printing_catalog_id != target.printing_catalog_id || path.size() < 2)
        return true;
    const Layer source_layer = source.region_island.region(0).layer();
    const Layer target_layer = target.region_island.region(0).layer();
    const Object object = target_layer.object();
    if (source_layer.object().handle() != object.handle())
        return true;
    if (target.printing_catalog_id >= m_plan.object_instance_count())
        throw std::runtime_error("A retraction refers to an invalid object instance.");
    const uint64_t instance = m_plan.object_instance(target.printing_catalog_id).object_instance_idx();
    if (instance >= object.instance_count())
        throw std::runtime_error("A retraction refers to an invalid object copy.");
    const c_point shift = object.instance_shift(static_cast<size_t>(instance));
    Work::Polyline local;
    for (const c_point point : path)
        local.points.push_back(Work::Point{point.x - shift.x, point.y - shift.y});

    const AC::Input &input = input_for(target_layer);
    const double nozzle_diameter = m_print_config.vector_float_or_default("nozzle_diameter", extruder_id, 0.4);
    AC::PerimeterCrossing &crossing =
        m_perimeter_crossings[{RouterKey{target_layer.handle(), extruder_id}, exclude_top_surfaces}];
    crossing.prepare(m_run_ctx->plugin_storage, *input.target, extruder_id,
                     std::max<coord_t>(1, scale_i(nozzle_diameter * 0.5)),
                     [this]() { throw_if_cancelled(m_run_ctx); }, exclude_top_surfaces);
    return crossing.touches_top_surface(m_run_ctx->plugin_storage, local) ||
        crossing.can_cross_perimeter(local, true);
}

AvoidCrossingPerimetersTravel &AvoidCrossingPerimetersTravel::instance(orchestrator_handle *orchestrator)
{
    static AvoidCrossingPerimetersTravel plugin(orchestrator);
    return plugin;
}

AvoidCrossingPerimetersTravel::AvoidCrossingPerimetersTravel(orchestrator_handle *orchestrator) :
    PluginBase(orchestrator),
    m_connector(orchestrator)
{}

const char *AvoidCrossingPerimetersTravel::id_impl() const noexcept
{
    return "layer_extrusion_edit.travel.avoid_crossing_perimeters";
}

const char *AvoidCrossingPerimetersTravel::name_impl() const noexcept
{
    return "Avoid crossing perimeters";
}

const char *AvoidCrossingPerimetersTravel::description_impl() const noexcept
{
    return "Reroutes eligible travels that would cross a perimeter.";
}

const char *AvoidCrossingPerimetersTravel::exclusive_group_impl() const noexcept
{
    return "layer_extrusion_edit.travel";
}

slicing_step_t AvoidCrossingPerimetersTravel::step_impl() const noexcept
{
    return STEP_LAYER_EXTRUSION_EDIT;
}

const char *const *AvoidCrossingPerimetersTravel::dependencies_impl() const noexcept
{
    return k_dependencies;
}

int32_t AvoidCrossingPerimetersTravel::priority_impl() const noexcept
{
    return -50;
}

int32_t AvoidCrossingPerimetersTravel::exclusive_group_priority_impl() const noexcept
{
    return -1;
}

int32_t AvoidCrossingPerimetersTravel::used_config_keys(raw_used_config_key *keys) const noexcept
{
    return perimeter_travel_config_keys(keys);
}

int32_t perimeter_travel_config_keys(raw_used_config_key *keys) noexcept
{
    static const raw_used_config_key used_keys[] = {
        {"avoid_crossing_perimeters", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"avoid_crossing_not_first_layer", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"avoid_crossing_perimeters_max_detour", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"nozzle_diameter", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"avoid_crossing_top", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"avoid_travel_island", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"avoid_travel_island_weight", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"infill_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"ironing", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"perimeter_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"perimeters", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"solid_infill_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
    };
    if (keys != nullptr)
        std::copy(std::begin(used_keys), std::end(used_keys), keys);
    return int32_t(sizeof(used_keys) / sizeof(used_keys[0]));
}

const char *AvoidCrossingPerimetersTravel::progress_message_format_impl() const noexcept
{
    return "Connecting perimeter-avoiding travel paths: %u / %u trees";
}

void AvoidCrossingPerimetersTravel::setup_impl(
    const plugin_run_context *run_ctx, const uint32_t run_count) const
{
    m_connector.reset();
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    m_setup_valid = ctx != nullptr && ctx->print != nullptr && ctx->plan != nullptr;
    if (m_setup_valid)
        m_connector.setup(PrintingPlan(ctx->plan), run_count);
}

void AvoidCrossingPerimetersTravel::setup_run_impl(const plugin_run_context *run_ctx) const
{
    if (!m_setup_valid)
        return;
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->layer_group == nullptr)
        return;
    PrintingLayerGroup layer_group(ctx->layer_group);
    m_connector.setup_run(layer_group,
                          ctx->group_idx, ctx->layer_group_idx, progress());
}

void AvoidCrossingPerimetersTravel::run_impl(const plugin_run_context *run_ctx) const
{
    if (!m_setup_valid)
        return;
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->print == nullptr || ctx->layer_group == nullptr)
        return;

    if (ctx->plan == nullptr)
        return;
    PrintingPlan plan(ctx->plan);
    LayerTravelPlanner planner(Print(ctx->print), plan, run_ctx);
    const TravelPathPlanner route = [&planner](const TravelEndpoint &source,
                                               const TravelEndpoint &target,
                                               const extruder_index_t extruder_id) {
        return planner.path(source, target, extruder_id);
    };
    PrintingLayerGroup layer_group(ctx->layer_group);
    m_connector.run(layer_group,
                    ctx->group_idx, ctx->layer_group_idx,
                    route, progress());
}

} // namespace

// Keep the provider's planner private while sharing its worker caches.
struct PerimeterTravelPlanner::Impl
{
    LayerTravelPlanner planner;
    // Keep the planner alive for all queries from one layer worker.
    Impl(const Print &print, const PrintingPlan &plan, const plugin_run_context *ctx) : planner(print, plan, ctx) {}
};

PerimeterTravelPlanner::PerimeterTravelPlanner(const Print &print, const PrintingPlan &plan,
    const plugin_run_context *ctx) : m_impl(std::make_unique<Impl>(print, plan, ctx)) {}
PerimeterTravelPlanner::~PerimeterTravelPlanner() = default;

int32_t PerimeterTravelPlanner::used_config_keys(raw_used_config_key *keys) noexcept
{
    return perimeter_travel_config_keys(keys);
}

std::vector<c_point> PerimeterTravelPlanner::path(const TravelEndpoint &source,
    const TravelEndpoint &target, extruder_index_t tool)
{
    return m_impl->planner.path(source, target, tool);
}

bool PerimeterTravelPlanner::crosses(const TravelEndpoint &source, const TravelEndpoint &target,
    extruder_index_t tool, const std::vector<c_point> &path, bool exclude_top_surfaces)
{
    return m_impl->planner.crosses(source, target, tool, path, exclude_top_surfaces);
}

void register_avoid_crossing_perimeters_travel_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(
        orchestrator, AvoidCrossingPerimetersTravel::instance(orchestrator).c_instance());
    const plugin_instance member = AvoidCrossingPerimetersTravel::instance(orchestrator).c_instance();
    raw_exclusive_group_def definition = raw_exclusive_group_def_init();
    definition.exclusive_group = member.vt->get_exclusive_group(member.ctx);
    definition.label = "Travel generation";
    definition.tooltip = "Selects how discontinuities between ordered extrusion leaves become travels.";
    AvoidCrossingPerimetersTravel::instance(orchestrator).register_exclusive_group(STEP_LAYER_EXTRUSION_EDIT, definition);
}

}}} // namespace slic3r_api::LayerExtrusionEdit::AvoidCrossingPerimetersTravelPlugin
