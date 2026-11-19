///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#include "DefaultZoneContributors.hpp"

#include <memory>
#include <map>
#include <set>
#include <utility>

#include "../AvoidCrossing/LayerInputCache.hpp"
#include "../AvoidCrossing/TravelBoundary.hpp"

/*
Top contributors copy boundary geometry for travels with a target region island (LayerRegionIsland)
and a valid target instance. Each target layer builds one boundary using the
worker-local input cache, reused for all its instances within one contributor call.
Growth masks and excluded tops do
not depend on the selected tool. Copies are shifted into platter coordinates;
the boundaries and their borrowed grids are destroyed when the call ends.
An original island is a layer perimeter slice before inset; for support targets,
TravelBoundary also uses the last object layer strictly below the support target's
Z + SCALED_EPSILON. A membership
mask is an area used to decide island membership, not a routing ring.
The attractor contributor appends each original perimeter slice as one Attractor
zone and retains the source layer handle with the zone. Source or target membership
in that original slice dates consecutive runs on the same layer and instance.
AdvancedTravel performs thin-wall withdrawal later, after forbidden material is
removed, so one M component receives one W.
Support domains include the lower object layer and have no forbidden tops.
The tops contributor appends Forbidden zones. No contributor replaces out.
Static excluded tops apply throughout the layer, one zone per connected ExPolygon so
endpoint membership ignores only that top. Scope is per instance, not per object.
Printed tops use the same components, but start after the last target in the
original island containing the component. Target positions are unshifted before
membership testing, with the same border-inclusive rule as attractors. A top
whose island has no target is omitted.
Cancellation and geometry failures propagate; output may already contain zones.
The publication keeps each source slice separate so geometry withdrawal cannot
change the dates. Support target inputs also contain lower object slices; this
contributor keeps their existing target-layer scope because the input exposes no
separate borrowed lower-layer handle.

     DefaultAttractorContributor::contribute: copy, shift and date raw source slices
    DefaultForbiddenTopsContributor::contribute / PrintedTopsContributor::contribute
    `-- contribute_zones: select target layer/instance pairs and copy areas
        |-- LayerInputCache::get: borrow immutable worker-local layer input
        |-- TravelBoundary::init_layer / build_internal: build temporary geometry
        `-- shifted_area: copy local areas into platter coordinates
*/

namespace slic3r_api { namespace LayerExtrusionEdit { namespace AdvancedTravel {
namespace {
namespace AC = AvoidCrossing;

// Copy local contours and holes with the instance shift; the source geometry remains unchanged.
polygon_work::ExPolygons shifted_area(polygon_work::ExPolygons area, c_point shift);
// Append tops for valid target pairs, optionally dating them after their island's last arrival; cancellation and geometry errors propagate.
void contribute_zones(const ZoneContributorInput &input, std::vector<TravelZone> &out, bool printed = false);

void contribute_zones(const ZoneContributorInput &input, std::vector<TravelZone> &out, const bool printed)
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
        const ZoneScope scope{EXTRUDER_ID_UNKNOWN, target.printing_catalog_id};
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
                out.push_back({ZoneRole::Forbidden, shifted_area({top}, shift), begin, SIZE_MAX, scope});
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
    std::set<std::pair<const layer_handle *, uint32_t>> visited;
    std::map<const layer_handle *, polygon_work::ExPolygons> slices;
    for (const PendingTravel &travel : input.travels)
        for (const TravelConnection::TravelEndpoint *endpoint : {&travel.source, &travel.target}) {
            throw_if_cancelled(&input.run_context);
            if (!endpoint->region_island.valid() || endpoint->region_island.region_count() == 0 ||
                endpoint->printing_catalog_id == SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE ||
                endpoint->printing_catalog_id >= input.plan.object_instance_count()) continue;
            const Layer layer = endpoint->region_island.region(0).layer();
            const Object object = layer.object();
            const uint64_t instance = input.plan.object_instance(endpoint->printing_catalog_id).object_instance_idx();
            if (instance >= object.instance_count() || !visited.emplace(layer.handle(), endpoint->printing_catalog_id).second) continue;
            const c_point shift = object.instance_shift(static_cast<size_t>(instance));
            if (slices.find(layer.handle()) == slices.end()) {
                const AC::Input &layer_input = input.layer_inputs.get(input.print, layer, &input.run_context);
                polygon_work::ExPolygons source_slices = layer_input.target->perimeter_slices;
                if (layer_input.target->support)
                    source_slices.insert(source_slices.end(), layer_input.lower_perimeter_slices.begin(),
                                         layer_input.lower_perimeter_slices.end());
                slices.emplace(layer.handle(), std::move(source_slices));
            }
            const ZoneScope scope{EXTRUDER_ID_UNKNOWN, endpoint->printing_catalog_id};
            for (const polygon_work::ExPolygon &slice : slices[layer.handle()]) {
                size_t begin = SIZE_MAX;
                for (size_t t = 0; t <= input.travels.size(); ++t) {
                    throw_if_cancelled(&input.run_context);
                    bool member = false;
                    if (t < input.travels.size())
                        for (const TravelConnection::TravelEndpoint *candidate : {&input.travels[t].source, &input.travels[t].target})
                            if (candidate->printing_catalog_id == endpoint->printing_catalog_id &&
                                candidate->region_island.valid() && candidate->region_island.region_count() != 0 &&
                                candidate->region_island.region(0).layer().handle() == layer.handle() &&
                                slice.contains({candidate->position.x - shift.x, candidate->position.y - shift.y}, true)) member = true;
                    if (member && begin == SIZE_MAX) begin = t;
                    if (!member && begin != SIZE_MAX) {
                        out.push_back({ZoneRole::Attractor, shifted_area({slice}, shift), begin, t, scope, layer.handle()});
                        begin = SIZE_MAX;
                    }
                }
            }
        }
}

void DefaultForbiddenTopsContributor::contribute(const ZoneContributorInput &input, std::vector<TravelZone> &out) const
{
    contribute_zones(input, out);
}

void PrintedTopsContributor::contribute(const ZoneContributorInput &input, std::vector<TravelZone> &out) const
{
    contribute_zones(input, out, true);
}

}}}
