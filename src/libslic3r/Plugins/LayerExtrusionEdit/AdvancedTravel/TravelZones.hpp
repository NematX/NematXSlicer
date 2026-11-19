///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

/*
TravelZones supplies dated platter-space areas to layer-local travel planners.
T is an index into LayerTravelChronology::travels, not a printable-scope index.
A PE is a PrintingExtrusion, the plan owner of one or more printable scopes.
The views borrow the unchanged PrintingPlan. Eligibility is checked by the
AdvancedTravel provider before this view is constructed, so the view retains
no extra chronology state.

Construct a ZoneContributorInput with a LayerInputCache owned by the
orchestrator for this worker, never shared between workers,
then construct TravelZoneView with the contributors in execution order. They
run immediately; query() lazily unions the applicable attractors.
T must not decrease between queries.
Attractors publish original perimeter slices, dated by source-or-target membership
in those slices. The planner tries keeping or omitting each group of original
obstacle pieces containing an endpoint. Overlap or boundary contact joins pieces
transitively, including the object slices used as obstacles for outside-part travel.
For each combination the remaining pieces form Z3. The planner subtracts Z3 from the union of active attractor slices
to form M_a, and withdraws
each M_a component into W_a for ring routing. A fixed bed_shape contour supplies M_p and W_p
for platter routes. M_a and M_p control membership and admission; W_a and W_p are
only their routing domains. An endpoint in a kept group can reach W by a straight
exit checked against the inset of other kept obstacles; the rest of the route is
checked against the inset of Z3. Only these material checks shrink the obstacles
by SCALED_EPSILON; domains, detours, the reference and the score use the original geometry.
The planner compares admissible routes, counting length inside endpoint groups
three times and keeping the first route on ties.
The view owns zones and cached unions; returned references remain valid until
its destruction. Ids identify applicable zone-index sets within this view only.
Keep run_context and its storage alive for queries; use one view per worker.
    ZoneContributorInput input{print, plan, context, z, cache, chronology.travels};
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
    // Layer associated with the source geometry; ordinary attractor slices borrow their original layer from the
    // print, while support-derived slices use the support target layer because LayerInput exposes no lower-layer handle.
    // The print and plan keep the handle alive during routing; support endpoints never receive attractor candidates,
    // so the support exception has no routing effect.
    const layer_handle *source_layer = nullptr;
};

struct ZoneContributorInput
{
    const Print &print;
    const PrintingPlan &plan;
    const plugin_run_context &run_context;
    coord_t print_z;
    AvoidCrossing::LayerInputCache &layer_inputs;
    const std::vector<PendingTravel> &travels;
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
    // Individual applicable attractor zones before union; indexes refer to TravelZoneView::zone_area().
    std::vector<size_t> attractor_indexes;
    // Individual applicable forbidden zones before expansion; indexes refer to TravelZoneView::zone_area().
    std::vector<size_t> forbidden_indexes;
};

class TravelZoneView
{
public:
    // Collect owned areas once, reject reversed dates and drop empty areas or date intervals before sorting events; errors and cancellation propagate.
    TravelZoneView(const std::vector<std::reference_wrapper<const ZoneContributor>> &contributors,
                  const ZoneContributorInput &input);
    // Return the cached attractor union for the dated, scope-matching set using the borrowed context's valid storage; T must not decrease (logic_error).
    // Cancellation and geometry failures propagate without caching partial areas.
    const ActiveZones &query(size_t t, const PendingTravel &travel);
    // Borrow one collected zone's unchanged geometry for an index returned by query(); valid until view destruction.
    const polygon_work::ExPolygons &zone_area(size_t index) const { return m_zones.at(index).area; }
    // Borrow one collected zone's scope and source-layer provenance for an index returned by query().
    const TravelZone &zone(size_t index) const { return m_zones.at(index); }
    // Return the number of contributor calls completed at construction.
    size_t contributor_calls() const { return m_contributor_calls; }
    // Return the number of completed nonempty attractor-union constructions; empty sets add nothing.
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
