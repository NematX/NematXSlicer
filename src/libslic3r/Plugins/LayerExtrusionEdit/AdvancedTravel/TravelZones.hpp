///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

/*
TravelZones supplies dated platter-space areas to layer-local travel planners.
T is an index into LayerTravelChronology::travels, not a printable-scope index.
A PE is a PrintingExtrusion, the plan owner of one or more printable scopes.
summarize() maps every printed PE and tool into half-open T intervals, even
when the PE has no new gap. Its views borrow the unchanged PrintingPlan.
Eligibility means the caller's decision that avoidance is active for a travel;
the summary records this decision without selecting or invoking a planner.

Construct a ZoneContributorInput with a LayerInputCache owned by the
orchestrator for this worker, never shared between workers,
then construct TravelZoneView with the contributors in execution order. They
run immediately; query() lazily unions the applicable active areas by role.
T must not decrease between queries.
Attractors are routing domains; the planner may use straight endpoint attachments
or island jumps outside them. Default attractors are dated by source-or-target
membership in their original perimeter slice, not in the withdrawn domain.
The view owns zones and cached unions; returned references remain valid until
its destruction. Ids identify applicable zone-index sets within this view only.
Keep run_context and its storage alive for queries; use one view per worker.
    // This example disables avoidance for every travel.
    ChronologySummary summary = summarize(chronology,
        [](const PendingTravel &) { return false; });
    ZoneContributorInput input{print, plan, context, z, cache, chronology.travels, summary};
    TravelZoneView zones(contributors, input);
    const ActiveZones &active = zones.query(t, chronology.travels[t]);
*/
#ifndef slic3r_plugins_advanced_travel_zones_hpp_
#define slic3r_plugins_advanced_travel_zones_hpp_

#include <functional>
#include <limits>
#include <map>
#include <set>
#include <vector>

#include "../TravelConnectionHelpers.hpp"
#include "../../Common/PolygonWorkBuffers.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/c/slic3r_plugin.h"

namespace slic3r_api { namespace LayerExtrusionEdit {
namespace AvoidCrossing { class LayerInputCache; }
namespace AdvancedTravel {

namespace polygon_work = PluginCommon::polygon_work;
using TravelConnection::PendingTravel;

enum class ZoneRole { Attractor, Forbidden };

struct ZoneScope
{
    // EXTRUDER_ID_UNKNOWN matches any tool.
    extruder_index_t extruder = EXTRUDER_ID_UNKNOWN;
    // SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE matches any instance; otherwise either endpoint must match.
    uint32_t printing_catalog_id = SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE;
    // When true, at least one endpoint PE must have a support role.
    bool support_travels_only = false;
};

struct TravelZone
{
    ZoneRole role = ZoneRole::Attractor;
    polygon_work::ExPolygons area;
    // Dates are [begin, end) in T; SIZE_MAX is an open end through the layer's remaining travels.
    size_t begin = 0;
    size_t end = (std::numeric_limits<size_t>::max)();
    ZoneScope scope;
};

// For a consecutive pair run p..q, bounds are the first T with pair_index >= p and the first T with pair_index > q.
// The incoming travel is included and the outgoing travel excluded; a run without a new gap is retained with an empty interval.
struct TravelInterval { size_t begin = 0; size_t end = 0; };
struct ExtrusionInterval
{
    PrintingExtrusion extrusion{nullptr};
    extruder_index_t extruder = EXTRUDER_ID_UNKNOWN;
    TravelInterval interval;
};
struct ToolInterval
{
    extruder_index_t extruder = EXTRUDER_ID_UNKNOWN;
    TravelInterval interval;
};
struct ChronologySummary
{
    size_t travel_count = 0;
    std::vector<ExtrusionInterval> extrusions;
    std::vector<ToolInterval> tools;
    // Caller-supplied eligibility for each T in the detailed travel list.
    std::vector<bool> avoidance_active;
    // True exactly when at least one entry of avoidance_active is true.
    bool any_avoidance_active = false;
};

// Map consecutive pair-owner and tool runs to half-open T intervals and evaluate eligibility once per gap.
ChronologySummary summarize(const TravelConnection::LayerTravelChronology &chronology,
    const std::function<bool(const PendingTravel &)> &avoidance_enabled);

struct ZoneContributorInput
{
    const Print &print;
    const PrintingPlan &plan;
    const plugin_run_context &run_context;
    coord_t print_z;
    AvoidCrossing::LayerInputCache &layer_inputs;
    const std::vector<PendingTravel> &travels;
    const ChronologySummary &summary;
};

class ZoneContributor
{
public:
    // Destroy the internal C++ contributor through its base type.
    virtual ~ZoneContributor() = default;
    // Append owned platter-space zones; the input and its views are borrowed during this call.
    virtual void contribute(const ZoneContributorInput &input, std::vector<TravelZone> &out) const = 0;
};

struct ActiveZones
{
    size_t id = 0;
    polygon_work::ExPolygons attractors;
    polygon_work::ExPolygons forbidden;
    // Individual applicable forbidden zones, before union or expansion; indexes refer to TravelZoneView::zone_area().
    std::vector<size_t> forbidden_indexes;
};

class TravelZoneView
{
public:
    // Collect owned areas once, reject reversed dates and drop empty areas or date intervals before sorting events; errors and cancellation propagate.
    TravelZoneView(const std::vector<std::reference_wrapper<const ZoneContributor>> &contributors,
                  const ZoneContributorInput &input);
    // Return cached role unions for the dated, scope-matching set using the borrowed context's valid storage; T must not decrease (logic_error).
    // Cancellation and geometry failures propagate without caching partial areas.
    const ActiveZones &query(size_t t, const PendingTravel &travel);
    // Borrow one collected zone's unchanged area for an index returned by query(); valid until view destruction.
    const polygon_work::ExPolygons &zone_area(size_t index) const { return m_zones.at(index).area; }
    // Return the number of contributor calls completed at construction.
    size_t contributor_calls() const { return m_contributor_calls; }
    // Return the number of completed nonempty role-union constructions, including work later discarded by cancellation; empty sets add nothing.
    size_t union_builds() const { return m_union_builds; }

private:
    struct Event { size_t t; size_t index; bool begin; };
    const plugin_run_context &m_context;
    std::vector<TravelZone> m_zones;
    std::vector<Event> m_events;
    std::set<size_t> m_active;
    size_t m_event_cursor = 0;
    size_t m_current_t = 0;
    std::map<std::vector<size_t>, ActiveZones> m_cache;
    size_t m_contributor_calls = 0;
    size_t m_union_builds = 0;
};

}}} // namespace slic3r_api::LayerExtrusionEdit::AdvancedTravel
#endif
