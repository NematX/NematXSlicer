///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#include "TravelZones.hpp"

#include <algorithm>
#include <stdexcept>

#include "../../Common/ClipperHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

/*
TravelZones collects contributor-owned geometry and keeps its published dates
while travels are applied. A pair is two adjacent scope roots, indexed by
pair_index; its target owner is a PrintingExtrusion (PE). Contributors borrow
the print, plan, run context and worker-local layer-input cache only during
collection. The view keeps the run context for cancellation and storage-backed
polygon operations.

Dates and scope are separate: crossing an event changes the dated active set,
while tool and instance filtering can change the applicable set even
without an event. Sorted applicable indexes key stable cached unions of attractors
and ids; forbidden indexes remain available for the planner's later selection.
Only a completed attractor union is published, so cancellation cannot cache partial areas.

    TravelZoneView: collect zones, validate dates and sort start/end events
    query: advance events, filter scopes, reuse or build the union of attractors
        applies: match the tool and either endpoint instance
*/

namespace slic3r_api { namespace LayerExtrusionEdit { namespace AdvancedTravel {

namespace {

// Test scope against the travel tool and endpoint instances without inspecting geometry.
bool applies(const ZoneScope &scope, const PendingTravel &travel);

bool applies(const ZoneScope &scope, const PendingTravel &travel)
{
    if (scope.extruder != EXTRUDER_ID_UNKNOWN && scope.extruder != travel.extruder_id)
        return false;
    if (scope.printing_catalog_id != SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE &&
        scope.printing_catalog_id != travel.source.printing_catalog_id &&
        scope.printing_catalog_id != travel.target.printing_catalog_id)
        return false;
    return true;
}

} // namespace

TravelZoneView::TravelZoneView(
    const std::vector<std::reference_wrapper<const ZoneContributor>> &contributors,
    const ZoneContributorInput &input) : m_context(input.run_context)
{
    for (const ZoneContributor &contributor : contributors) {
        throw_if_cancelled(&m_context);
        contributor.contribute(input, m_zones);
        ++m_contributor_calls;
    }
    throw_if_cancelled(&m_context);
    for (TravelZone &zone : m_zones) {
        if (zone.begin > zone.end)
            throw std::invalid_argument("A travel zone has reversed dates.");
        zone.area.erase(std::remove_if(zone.area.begin(), zone.area.end(),
            [](const polygon_work::ExPolygon &area) { return area.contour.points.empty(); }),
            zone.area.end());
    }
    m_zones.erase(std::remove_if(m_zones.begin(), m_zones.end(),
        [](const TravelZone &zone) { return zone.area.empty() || zone.begin == zone.end; }),
        m_zones.end());
    for (size_t index = 0; index < m_zones.size(); ++index) {
        m_events.push_back({m_zones[index].begin, index, true});
        m_events.push_back({m_zones[index].end, index, false});
    }
    std::sort(m_events.begin(), m_events.end(), [](const Event &lhs, const Event &rhs) {
        return lhs.t < rhs.t || (lhs.t == rhs.t && lhs.index < rhs.index);
    });
}

const ActiveZones &TravelZoneView::query(const size_t t, const PendingTravel &travel)
{
    if (t < m_current_t)
        throw std::logic_error("Travel zone query dates must not decrease.");
    throw_if_cancelled(&m_context);
    while (m_event_cursor < m_events.size() && m_events[m_event_cursor].t <= t) {
        const Event &event = m_events[m_event_cursor++];
        if (event.begin)
            m_active.insert(event.index);
        else
            m_active.erase(event.index);
    }
    m_current_t = t;
    std::vector<size_t> indexes;
    for (size_t index : m_active) {
        if (applies(m_zones[index].scope, travel))
            indexes.push_back(index);
    }
    const std::map<std::vector<size_t>, ActiveZones>::const_iterator found = m_cache.find(indexes);
    if (found != m_cache.end())
        return found->second;

    ActiveZones active;
    active.id = m_cache.size();
    for (size_t index : indexes) {
        throw_if_cancelled(&m_context);
        if (m_zones[index].role == ZoneRole::Attractor) {
            active.attractor_indexes.push_back(index);
            active.attractors.insert(active.attractors.end(), m_zones[index].area.begin(), m_zones[index].area.end());
        } else {
            active.forbidden_indexes.push_back(index);
        }
    }
    throw_if_cancelled(&m_context);
    if (!active.attractors.empty()) {
        active.attractors = PluginCommon::clipper::union_ex(m_context.plugin_storage, active.attractors);
        ++m_union_builds;
    }
    throw_if_cancelled(&m_context);
    return m_cache.emplace(std::move(indexes), std::move(active)).first->second;
}

}}} // namespace slic3r_api::LayerExtrusionEdit::AdvancedTravel
