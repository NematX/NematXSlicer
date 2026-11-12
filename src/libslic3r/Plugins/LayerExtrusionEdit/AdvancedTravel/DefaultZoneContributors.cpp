///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#include "DefaultZoneContributors.hpp"

#include <memory>
#include <map>
#include <set>
#include <utility>

#include "../../Common/ClipperHelpers.hpp"
#include "../../ElephantFootCompensation.hpp"
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
The attractor contributor withdraws each perimeter slice with the shared thin-wall
helper and appends one Attractor zone per source slice and instance. Source or
target membership in that original slice dates consecutive runs on the same layer
and instance. Its per-call cache owns local domains, reused across instances.
Support domains include the lower object layer and have no forbidden tops.
The tops contributor appends Forbidden zones. No contributor replaces out.
Static excluded tops apply throughout the layer, one zone per connected ExPolygon so
endpoint membership ignores only that top. Scope is per instance, not per object.
Printed tops use the same components, but start after the last target in the
original island containing the component. Target positions are unshifted before
membership testing, with the same border-inclusive rule as attractors. A top
whose island has no target is omitted.
Cancellation and geometry failures propagate; output may already contain zones.
Support obstacles instead copy slices directly at or below the group's Z, once
per plan-catalog instance. Separate components let an endpoint ignore only the
material it touches; all other components remain forbidden for support travels.
The independent attractor_domain helper copies perimeter slices into SDK storage,
withdraws each island with the shared thin-wall helper, then unions owned results.
Its unioned object-local result serves geometry consumers; publication keeps each
source slice separate so withdrawal cannot change the dates.

    attractor_domain: withdraw slices and optional support lower slices, then union
    `-- withdrawn_slices: copy source slices and limit withdrawal near the minimum wall width
    DefaultAttractorContributor::contribute: cache withdrawn slices, shift and date source-or-target runs
    `-- withdrawn_slices: use elephant_foot_compensation to retain thin passages
    DefaultForbiddenTopsContributor::contribute / PrintedTopsContributor::contribute
    `-- contribute_zones: select target layer/instance pairs and copy areas
        |-- LayerInputCache::get: borrow immutable worker-local layer input
        |-- TravelBoundary::init_layer / build_internal: build temporary geometry
        `-- shifted_area: copy local areas into platter coordinates
    SupportObstacleContributor::contribute: copy lower object slices for catalog instances
        `-- first_layer_below_print_z: select the last layer below Z plus epsilon
*/

namespace slic3r_api { namespace LayerExtrusionEdit { namespace AdvancedTravel {
// Copy object-local slices through SDK thin-wall withdrawal and union owned results; a null target or empty slices return empty.
// Nonempty input requires valid storage and strictly positive minimum width; non-compensable islands reach the final union without withdrawal.
// That union normalizes the result and may drop degenerate contours.
polygon_work::ExPolygons attractor_domain(storage_handle *storage, const AvoidCrossing::Input &input);
namespace {
namespace AC = AvoidCrossing;

using WithdrawnSlices = std::vector<std::pair<polygon_work::ExPolygon, polygon_work::ExPolygons>>;
// Copy source slices and withdraw each through SDK thin-wall compensation, returning owned source/parts pairs; a null target returns empty.
// Include lower object slices for support; nonempty input requires valid storage and a positive minimum wall width; geometry errors propagate.
WithdrawnSlices withdrawn_slices(storage_handle *storage, const AC::Input &input);
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
            const AC::Input &layer_input = input.layer_inputs.get(input.print, layer, &input.run_context, nullptr);
            boundary = std::make_unique<AC::TravelBoundary>();
            boundary->init_layer(input.run_context.plugin_storage, *layer_input.target);
            boundary->build_internal(input.run_context.plugin_storage, layer_input, travel.extruder_id);
            throw_if_cancelled(&input.run_context);
        }
        const ZoneScope scope{EXTRUDER_ID_UNKNOWN, target.printing_catalog_id, false};
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

WithdrawnSlices withdrawn_slices(storage_handle *storage, const AC::Input &input)
{
    if (!input.target)
        return {};
    const AC::LayerInput &layer = *input.target;
    polygon_work::ExPolygons slices = layer.perimeter_slices;
    if (layer.support)
        slices.insert(slices.end(), input.lower_perimeter_slices.begin(), input.lower_perimeter_slices.end());
    if (slices.empty())
        return {};
    const StoredExPolygonCollection stored = PluginCommon::clipper::to_stored_expolygons(storage, slices);
    WithdrawnSlices withdrawn;
    size_t index = 0;
    for (const ExPolygon island : stored.readonly()) {
        const StoredExPolygonCollection inset = XYSizeCompensationPlugin::detail::elephant_foot_compensation(
            storage, island, unscaled(layer.external_perimeter_width + layer.perimeter_spacing),
            coord_t(1.5 * layer.perimeter_spacing));
        withdrawn.emplace_back(std::move(slices[index++]), PluginCommon::clipper::from_expolygon_collection_view(inset.readonly()));
    }
    return withdrawn;
}
} // namespace

polygon_work::ExPolygons attractor_domain(storage_handle *storage, const AvoidCrossing::Input &input)
{
    polygon_work::ExPolygons parts;
    for (const std::pair<polygon_work::ExPolygon, polygon_work::ExPolygons> &slice : withdrawn_slices(storage, input))
        parts.insert(parts.end(), slice.second.begin(), slice.second.end());
    if (parts.empty()) return {};
    return PluginCommon::clipper::union_ex(storage, parts);
}

void DefaultAttractorContributor::contribute(const ZoneContributorInput &input, std::vector<TravelZone> &out) const
{
    std::set<std::pair<const layer_handle *, uint32_t>> visited;
    std::map<const layer_handle *, WithdrawnSlices> domains;
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
            if (domains.find(layer.handle()) == domains.end()) {
                const AC::Input &layer_input = input.layer_inputs.get(input.print, layer, &input.run_context, nullptr);
                domains.emplace(layer.handle(), withdrawn_slices(input.run_context.plugin_storage, layer_input));
            }
            const ZoneScope scope{EXTRUDER_ID_UNKNOWN, endpoint->printing_catalog_id, false};
            for (const std::pair<polygon_work::ExPolygon, polygon_work::ExPolygons> &slice : domains[layer.handle()]) {
                size_t begin = SIZE_MAX;
                for (size_t t = 0; t <= input.travels.size(); ++t) {
                    throw_if_cancelled(&input.run_context);
                    bool member = false;
                    if (t < input.travels.size())
                        for (const TravelConnection::TravelEndpoint *candidate : {&input.travels[t].source, &input.travels[t].target})
                            if (candidate->printing_catalog_id == endpoint->printing_catalog_id &&
                                candidate->region_island.valid() && candidate->region_island.region_count() != 0 &&
                                candidate->region_island.region(0).layer().handle() == layer.handle() &&
                                slice.first.contains({candidate->position.x - shift.x, candidate->position.y - shift.y}, true)) member = true;
                    if (member && begin == SIZE_MAX) begin = t;
                    if (!member && begin != SIZE_MAX) {
                        if (!slice.second.empty()) out.push_back({ZoneRole::Attractor, shifted_area(slice.second, shift), begin, t, scope});
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
