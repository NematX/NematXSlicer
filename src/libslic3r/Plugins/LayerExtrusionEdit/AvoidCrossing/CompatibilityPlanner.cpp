///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "CompatibilityPlanner.hpp"

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <stdexcept>
#include <utility>

#include "libslic3r/Api/plugin/cpp/RegionSettingsViews.hpp"

/*
CompatibilityPlanner answers regional avoidance and crossing questions through
public API views.

Both endpoints must enable avoid_crossing_perimeters; a failed regional lookup
retries at the interior probe supplied by the connector. If
avoid_crossing_not_first_layer is enabled, the target's first layer is excluded,
including layers at the same print Z.
If either endpoint has no region island or that island has no regions,
avoidance is disabled.
Endpoints without a valid object instance, including global skirt and brim,
never enable avoidance.

crosses() copies the target's geometry, region settings and averaged spacing
through public API views, then subtracts the target instance shift from the route.
PerimeterCrossing tests that route against boundaries of prepared interior areas,
not extrusion centerlines, optionally treating top surfaces as exposed. Missing or
different object provenance counts as a crossing.

Each planner owns its copied inputs and one crossing cache per target layer, tool
and top choice. Temporary geometry operations use the run context's plugin
storage. Planners do not escape the worker callback, so their mutable caches are
not shared between workers.

    avoidance_enabled: check regional provenance, instances and first-layer exclusion
    `-- endpoint_enables_avoidance: check each endpoint's regional setting
    crosses: classify a route for retraction planning
    |-- input_for: obtain the target layer input
    |   `-- LayerInputCache::get: retain copied geometry, settings and spacing
    `-- PerimeterCrossing::prepare/touches_top_surface/can_cross_perimeter: test the route
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

int32_t CompatibilityPlanner::used_config_keys(raw_used_config_key *keys) noexcept
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
