///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#include "TravelZones.hpp"

#include <algorithm>
#include <stdexcept>

#include "../../Common/ClipperHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

/*
TravelZones converts ordered pair owners into travel dates and collects owned
zone geometry before any travel is applied. A pair is two adjacent scope roots,
indexed by pair_index; its target owner is a PrintingExtrusion (PE).
Contributors borrow the print, plan, run context and worker-local layer-input
cache only during collection. The view
keeps the run context for cancellation and storage-backed polygon operations.

Dates and scope are separate: crossing an event changes the dated active set,
while tool, instance and support filtering can change the applicable set even
without an event. Sorted applicable indexes key stable cached unions and ids.
Only a completed union is published, so cancellation cannot cache partial areas.

    summarize: map pair runs to travel bounds and evaluate caller eligibility
    TravelZoneView: collect zones, validate dates and sort start/end events
    query: advance events, filter scopes, reuse or build the two role unions
        applies: match tool, either instance and either support endpoint
*/

namespace slic3r_api { namespace LayerExtrusionEdit { namespace AdvancedTravel {

// Convert consecutive pair runs to half-open travel dates, retaining empty intervals and borrowed owner views; predicate failures propagate.
ChronologySummary summarize(const TravelConnection::LayerTravelChronology &chronology,
    const std::function<bool(const PendingTravel &)> &avoidance_enabled);

namespace {

// Test scope against borrowed endpoint identities and PE roles without inspecting or changing geometry.
bool applies(const ZoneScope &scope, const PendingTravel &travel);

bool applies(const ZoneScope &scope, const PendingTravel &travel)
{
    if (scope.extruder != EXTRUDER_ID_UNKNOWN && scope.extruder != travel.extruder_id)
        return false;
    if (scope.printing_catalog_id != SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE &&
        scope.printing_catalog_id != travel.source.printing_catalog_id &&
        scope.printing_catalog_id != travel.target.printing_catalog_id)
        return false;
    if (!scope.support_travels_only)
        return true;
    return (travel.source_extrusion.valid() &&
            RAW_EXTRUSION_ROLE_IS_SUPPORT(travel.source_extrusion.role())) ||
           (travel.target_extrusion.valid() &&
            RAW_EXTRUSION_ROLE_IS_SUPPORT(travel.target_extrusion.role()));
}

} // namespace

ChronologySummary summarize(const TravelConnection::LayerTravelChronology &chronology,
    const std::function<bool(const PendingTravel &)> &avoidance_enabled)
{
    ChronologySummary result;
    result.travel_count = chronology.travels.size();
    // A pair can print without generating a travel; lower_bound retains its empty T interval.
    const auto date = [&chronology](size_t pair) -> size_t {
        return size_t(std::lower_bound(chronology.travels.begin(), chronology.travels.end(), pair,
            [](const PendingTravel &travel, size_t index) { return travel.pair_index < index; }) -
            chronology.travels.begin());
    };
    const std::vector<TravelConnection::LayerTravelChronology::PairExtrusion> &pairs =
        chronology.pair_extrusions;
    for (size_t begin = 0; begin < pairs.size();) {
        size_t end = begin + 1;
        while (end < pairs.size() && pairs[end].target.handle() == pairs[begin].target.handle())
            ++end;
        result.extrusions.push_back({pairs[begin].target, pairs[begin].extruder_id,
                                     {date(begin), date(end)}});
        begin = end;
    }
    for (size_t begin = 0; begin < pairs.size();) {
        size_t end = begin + 1;
        while (end < pairs.size() && pairs[end].extruder_id == pairs[begin].extruder_id)
            ++end;
        result.tools.push_back({pairs[begin].extruder_id, {date(begin), date(end)}});
        begin = end;
    }
    for (const PendingTravel &travel : chronology.travels) {
        const bool enabled = avoidance_enabled(travel);
        result.avoidance_active.push_back(enabled);
        result.any_avoidance_active = result.any_avoidance_active || enabled;
    }
    return result;
}

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
        polygon_work::ExPolygons &area = m_zones[index].role == ZoneRole::Attractor ?
            active.attractors : active.forbidden;
        area.insert(area.end(), m_zones[index].area.begin(), m_zones[index].area.end());
    }
    throw_if_cancelled(&m_context);
    if (!active.attractors.empty()) {
        active.attractors = PluginCommon::clipper::union_ex(m_context.plugin_storage, active.attractors);
        ++m_union_builds;
    }
    throw_if_cancelled(&m_context);
    if (!active.forbidden.empty()) {
        active.forbidden = PluginCommon::clipper::union_ex(m_context.plugin_storage, active.forbidden);
        ++m_union_builds;
    }
    throw_if_cancelled(&m_context);
    return m_cache.emplace(std::move(indexes), std::move(active)).first->second;
}

}}} // namespace slic3r_api::LayerExtrusionEdit::AdvancedTravel
