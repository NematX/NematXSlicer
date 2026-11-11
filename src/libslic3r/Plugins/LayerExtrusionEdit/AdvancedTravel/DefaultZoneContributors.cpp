///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#include "DefaultZoneContributors.hpp"

#include <memory>
#include <map>
#include <set>
#include <utility>

#include "../../Common/ClipperHelpers.hpp"
#include "../AvoidCrossing/LayerInputCache.hpp"
#include "../AvoidCrossing/TravelBoundary.hpp"

/*
Default contributors copy boundary geometry for travels with a target region island (LayerRegionIsland)
and a valid target instance. Each target layer builds one boundary using the
worker-local input cache, reused for all its instances within one contributor call.
Growth masks and excluded tops do
not depend on the selected tool. Copies are shifted into platter coordinates;
the boundaries and their borrowed grids are destroyed when the call ends.
An original island is a layer perimeter slice before inset; for support targets,
TravelBoundary also uses the object layer at or below the target. A membership
mask is an area used to decide island membership, not a routing ring.
The attractor contributor appends Attractor zones from these original islands,
before inset, top removal and tool clipping. Support masks include the lower
object layer and have no forbidden tops. The tops contributor appends Forbidden
zones. No contributor replaces out. Each mask is dated by consecutive target membership, whereas
static excluded tops apply throughout the layer, one zone per connected ExPolygon so
endpoint membership ignores only that top. Scope is per instance, not per object.
Printed tops use the same components, but start after the last target in the
original island containing the component. Target positions are unshifted before
membership testing, with the same border-inclusive rule as attractors. A top
whose island has no target is omitted.
Cancellation and geometry failures propagate; output may already contain zones.
Support obstacles instead copy slices directly at or below the group's Z, once
per plan-catalog instance. Separate components let an endpoint ignore only the
material it touches; all other components remain forbidden for support travels.

    DefaultAttractorContributor::contribute / DefaultForbiddenTopsContributor::contribute / PrintedTopsContributor::contribute
    `-- contribute_zones: select target layer/instance pairs and copy areas
        |-- LayerInputCache::get: borrow immutable worker-local layer input
        |-- TravelBoundary::init_layer / build_internal: build temporary geometry
        `-- shifted_area: copy local areas into platter coordinates
    SupportObstacleContributor::contribute: copy lower object slices for catalog instances
        `-- first_layer_below_print_z: select the last layer below Z plus epsilon
*/

namespace slic3r_api { namespace LayerExtrusionEdit { namespace AdvancedTravel {
namespace {
namespace AC = AvoidCrossing;

// Copy local contours and holes with the instance shift; the source geometry remains unchanged.
polygon_work::ExPolygons shifted_area(polygon_work::ExPolygons area, c_point shift);
// Append masks or tops for valid target pairs, optionally dating tops after their island's last arrival; cancellation and geometry errors propagate.
void contribute_zones(const ZoneContributorInput &input, std::vector<TravelZone> &out, ZoneRole role, bool printed = false);

void contribute_zones(const ZoneContributorInput &input, std::vector<TravelZone> &out, const ZoneRole role, const bool printed)
{
    std::set<std::pair<const layer_handle *, uint32_t>> visited;
    std::map<const layer_handle *, std::unique_ptr<AC::TravelBoundary>> boundaries;
    for (const PendingTravel &travel : input.travels) {
        throw_if_cancelled(&input.run_context);
        const TravelConnection::TravelEndpoint &target = travel.target;
        if (!target.region_island.valid() || target.region_island.region_count() == 0 ||
            target.printing_catalog_id == SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE ||
            target.printing_catalog_id >= input.plan.object_instance_count())
            continue;
        const Layer layer = target.region_island.region(0).layer();
        const Object object = layer.object();
        const uint64_t instance = input.plan.object_instance(target.printing_catalog_id).object_instance_idx();
        if (instance >= object.instance_count() ||
            !visited.emplace(layer.handle(), target.printing_catalog_id).second)
            continue;
        const c_point shift = object.instance_shift(static_cast<size_t>(instance));
        std::unique_ptr<AC::TravelBoundary> &boundary = boundaries[layer.handle()];
        if (!boundary) {
            const AC::Input &layer_input = input.layer_inputs.get(input.print, layer, &input.run_context);
            boundary = std::make_unique<AC::TravelBoundary>();
            boundary->init_layer(input.run_context.plugin_storage, *layer_input.target);
            boundary->build_internal(input.run_context.plugin_storage, layer_input, travel.extruder_id);
            throw_if_cancelled(&input.run_context);
        }
        const ZoneScope scope{EXTRUDER_ID_UNKNOWN, target.printing_catalog_id, false};
        if (role == ZoneRole::Forbidden) {
            for (const polygon_work::ExPolygon &top : boundary->internal_boundary().to_avoid) {
                throw_if_cancelled(&input.run_context);
                size_t begin = printed ? SIZE_MAX : 0;
                if (printed) {
                    // Each inset top belongs to the original island containing its contour.
                    for (const std::pair<polygon_work::ExPolygon, polygon_work::ExPolygon> &growth : boundary->internal_boundary().boundary_growth) {
                        if (!growth.first.contains(top.contour.points.front()))
                            continue;
                        for (size_t t = 0; t < input.travels.size(); ++t) {
                            throw_if_cancelled(&input.run_context);
                            const TravelConnection::TravelEndpoint &endpoint = input.travels[t].target;
                            if (endpoint.printing_catalog_id == target.printing_catalog_id &&
                                endpoint.region_island.valid() && endpoint.region_island.region_count() != 0 &&
                                endpoint.region_island.region(0).layer().handle() == layer.handle() &&
                                growth.first.contains({endpoint.position.x - shift.x, endpoint.position.y - shift.y}))
                                begin = t + 1;
                        }
                        break;
                    }
                }
                if (begin != SIZE_MAX)
                    out.push_back({role, shifted_area({top}, shift), begin, SIZE_MAX, scope});
            }
            continue;
        }
        for (const std::pair<polygon_work::ExPolygon, polygon_work::ExPolygon> &growth : boundary->internal_boundary().boundary_growth) {
            throw_if_cancelled(&input.run_context);
            size_t begin = SIZE_MAX;
            // A different target pair interrupts membership even when its coordinates overlap this mask.
            for (size_t t = 0; t <= input.travels.size(); ++t) {
                bool member = false;
                if (t < input.travels.size()) {
                    const TravelConnection::TravelEndpoint &endpoint = input.travels[t].target;
                    member = endpoint.printing_catalog_id == target.printing_catalog_id &&
                        endpoint.region_island.valid() && endpoint.region_island.region_count() != 0 &&
                        endpoint.region_island.region(0).layer().handle() == layer.handle() &&
                        growth.first.contains({endpoint.position.x - shift.x, endpoint.position.y - shift.y});
                }
                if (member && begin == SIZE_MAX) begin = t;
                if (!member && begin != SIZE_MAX) {
                    out.push_back({role, shifted_area({growth.first}, shift), begin, t, scope});
                    begin = SIZE_MAX;
                }
            }
        }
    }
}

polygon_work::ExPolygons shifted_area(polygon_work::ExPolygons area, const c_point shift)
{
    for (polygon_work::ExPolygon &island : area) {
        for (polygon_work::Point &point : island.contour.points) point += polygon_work::Point{shift.x, shift.y};
        for (polygon_work::Polygon &hole : island.holes)
            for (polygon_work::Point &point : hole.points) point += polygon_work::Point{shift.x, shift.y};
    }
    return area;
}
} // namespace

void DefaultAttractorContributor::contribute(const ZoneContributorInput &input, std::vector<TravelZone> &out) const
{
    contribute_zones(input, out, ZoneRole::Attractor);
}

void DefaultForbiddenTopsContributor::contribute(const ZoneContributorInput &input, std::vector<TravelZone> &out) const
{
    contribute_zones(input, out, ZoneRole::Forbidden);
}

void PrintedTopsContributor::contribute(const ZoneContributorInput &input, std::vector<TravelZone> &out) const
{
    contribute_zones(input, out, ZoneRole::Forbidden, true);
}

void SupportObstacleContributor::contribute(const ZoneContributorInput &input, std::vector<TravelZone> &out) const
{
    for (uint32_t oi = 0; oi < input.print.object_count(); ++oi) {
        throw_if_cancelled(&input.run_context);
        const Object object = input.print.object(oi);
        const std::optional<uint32_t> below = AC::first_layer_below_print_z(object, input.print_z + SCALED_EPSILON);
        if (!below)
            continue;
        const polygon_work::ExPolygons slices = PluginCommon::clipper::from_expolygon_collection_view(object.layer(*below).slices());
        for (uint32_t ii = 0; ii < input.plan.object_instance_count(); ++ii) {
            throw_if_cancelled(&input.run_context);
            const PrintingObjectInstance instance = input.plan.object_instance(ii);
            if (!instance.has_object() || instance.object().handle() != object.handle())
                continue;
            const c_point shift = object.instance_shift(instance.object_instance_idx());
            const ZoneScope scope{EXTRUDER_ID_UNKNOWN, SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE, true};
            for (const polygon_work::ExPolygon &slice : slices) {
                throw_if_cancelled(&input.run_context);
                out.push_back({ZoneRole::Forbidden, shifted_area({slice}, shift), 0, SIZE_MAX, scope});
            }
        }
    }
}

}}}
