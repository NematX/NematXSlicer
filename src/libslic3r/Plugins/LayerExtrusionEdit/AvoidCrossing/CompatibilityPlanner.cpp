///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "CompatibilityPlanner.hpp"

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <utility>

#include "libslic3r/Api/plugin/cpp/RegionSettingsViews.hpp"

/*
CompatibilityPlanner supplies XY routes for travel providers through public API views.

Both endpoints must enable avoid_crossing_perimeters; a failed regional lookup
retries at the interior probe supplied by the connector. If
avoid_crossing_not_first_layer is enabled, the target's first layer is excluded,
including layers at the same print Z.
If either endpoint has no region island or that island has no regions, the
planner keeps straight travel.
Endpoints without a valid object instance, including global skirt and brim,
keep the direct travel.

The planner copies the target's geometry, region settings and averaged spacing
through public API views, then
subtracts the target instance shift from both endpoints. PerimeterCrossing tests
the direct path against boundaries of prepared interior areas, not extrusion
centerlines. Without a crossing, the planner keeps the straight path.
For off-object targets with a region and real instance, object areas below the
layer-group Z provide platter-space crossing boundaries. Current producers set no
inherited Z offset on these targets (support and per-object brim); regionless wipe-tower targets
keep the early straight return before detection.
Current and earlier groups supply real instances; later groups and synthetic entries
are excluded. Routing geometry remains the owner's local boundary.

TravelRouter uses internal boundaries for travel within one object instance and
external boundaries for travel between instances or objects. The external input
is copied only when requested. The router may keep a direct path when its
heuristic cannot find an acceptable detour or the extra length exceeds the
configured limit; it does not guarantee a shortest or crossing-free path.
The planner restores the instance shift and exact original endpoints before
returning XY points to the connector.

Each run owns its copied inputs and one router and crossing cache per target
layer/tool pair. Inputs outlive the routers borrowing them. Temporary geometry
operations use the run context's plugin storage. Planners do not escape the
worker callback, so their mutable caches are not shared between workers.

    path: select straight travel or a detour
    |-- avoidance_enabled: check regional provenance, instances and first-layer exclusion
    |   `-- endpoint_enables_avoidance: check each endpoint's regional setting
    |-- input_for: obtain the target layer input
    |   `-- LayerInputCache::get: retain copied geometry, settings and spacing
    |-- router_for: retain layer/tool state and defer external input
    |-- PerimeterCrossing::prepare/prepare_platter/can_cross_perimeter: test the direct path
    `-- TravelRouter::travel_to: choose a route within the detour budget
*/

namespace slic3r_api { namespace LayerExtrusionEdit { namespace AvoidCrossing {

using namespace TravelConnection;
namespace AC = AvoidCrossing;
namespace Work = PluginCommon::work;

namespace {
const char *const k_region_setting_keys[] = {
    "avoid_crossing_perimeters"
};
} // namespace

CompatibilityPlanner::CompatibilityPlanner(const Print &print,
                                       const PrintingPlan &plan,
                                       const plugin_run_context *run_ctx,
                                       uint32_t group_idx, coord_t print_z) :
    m_print(print),
    m_print_config(print.config()),
    m_plan(plan),
    m_run_ctx(run_ctx),
    m_group_idx(group_idx),
    m_print_z(print_z)
{}

c_point CompatibilityPlanner::local_point(const c_point point, const TravelEndpoint &endpoint) const
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

bool CompatibilityPlanner::endpoint_enables_avoidance(const TravelEndpoint &endpoint) const
{
    // Global skirt and brim regions have no object instance for local coordinates.
    if (!endpoint_has_object_instance(endpoint))
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

bool CompatibilityPlanner::endpoint_has_object_instance(const TravelEndpoint &endpoint) const
{
    if (!endpoint.region_island.valid() || endpoint.region_island.region_count() == 0 ||
        endpoint.printing_catalog_id == SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE ||
        endpoint.printing_catalog_id >= m_plan.object_instance_count()) return false;
    const Object object = endpoint.region_island.region(0).layer().object();
    return m_plan.object_instance(endpoint.printing_catalog_id).object_instance_idx() < object.instance_count();
}

const AC::Input &CompatibilityPlanner::input_for(const Layer &layer)
{
    return m_inputs.get(m_print, layer, m_run_ctx);
}

CompatibilityPlanner::RoutingState &CompatibilityPlanner::router_for(
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

std::vector<c_point> CompatibilityPlanner::straight(const TravelEndpoint &source,
                                                   const TravelEndpoint &target,
                                                   const extruder_index_t extruder_id) const
{
    return straight_path(source, target, extruder_id);
}

bool CompatibilityPlanner::avoidance_enabled(const TravelEndpoint &source,
                                             const TravelEndpoint &target) const
{
    // Missing regional provenance is normal for support and print-level
    // auxiliary paths. Those gaps deliberately retain the direct travel.
    if (!source.region_island.valid() || source.region_island.region_count() == 0 ||
        !target.region_island.valid() || target.region_island.region_count() == 0)
        return false;
    if (!endpoint_enables_avoidance(source) || !endpoint_enables_avoidance(target))
        return false;

    const Layer target_layer_view = target.region_island.region(0).layer();
    const Object target_object = target_layer_view.object();
    return !m_print_config.bool_or_default("avoid_crossing_not_first_layer", true) ||
        !AC::is_first_layer(target_object, target_layer_view);
}

std::vector<c_point> CompatibilityPlanner::path(const TravelEndpoint &source,
                                              const TravelEndpoint &target,
                                              const extruder_index_t extruder_id)
{
    throw_if_cancelled(m_run_ctx);
    if (!avoidance_enabled(source, target))
        return straight(source, target, extruder_id);

    const Layer source_layer_view = source.region_island.region(0).layer();
    const Layer target_layer_view = target.region_island.region(0).layer();
    const Object source_object = source_layer_view.object();
    const Object target_object = target_layer_view.object();
    const uint64_t target_instance_idx = m_plan.object_instance(target.printing_catalog_id).object_instance_idx();

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
    const bool off_object = is_off_object_layer(target_layer_view);
    if (off_object)
        state.crossing.prepare_platter(m_run_ctx->plugin_storage, m_plan, m_group_idx,
            m_print_z, nozzle_radius,
            [this]() { throw_if_cancelled(m_run_ctx); });
    else
        state.crossing.prepare(m_run_ctx->plugin_storage, *input.target, extruder_id, nozzle_radius,
            [this]() { throw_if_cancelled(m_run_ctx); });
    throw_if_cancelled(m_run_ctx);

    Work::Polyline direct;
    direct.points = off_object ? Work::Points{{source.position.x, source.position.y}, {target.position.x, target.position.y}} :
        Work::Points{local_source, local_target};
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

bool CompatibilityPlanner::crosses(const TravelEndpoint &source,
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
    PerimeterCrossing &crossing =
        m_perimeter_crossings[{RouterKey{target_layer.handle(), extruder_id}, exclude_top_surfaces}];
    crossing.prepare(m_run_ctx->plugin_storage, *input.target, extruder_id,
                     std::max<coord_t>(1, scale_i(nozzle_diameter * 0.5)),
                     [this]() { throw_if_cancelled(m_run_ctx); }, exclude_top_surfaces);
    return crossing.touches_top_surface(m_run_ctx->plugin_storage, local) ||
        crossing.can_cross_perimeter(local, true);
}

}}} // namespace slic3r_api::LayerExtrusionEdit::AvoidCrossing
