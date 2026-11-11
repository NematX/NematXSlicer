///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

/*
Default contributors supply instance-scoped membership masks and excluded tops
to TravelZoneView. Pass the selected contributors in the view's contributor list.
Each forbidden top component is a separate zone, so ignoring an endpoint's top
does not ignore disjoint tops. Attractors copy original islands, not routing rings, and are active only during
consecutive travels targeting that island. Static forbidden tops last for the layer.
PrintedTopsContributor instead starts each top after the last arrival in its
original island, on the same layer and instance; islands without arrivals emit
no top. This is the first travel after that island's last arrival, not an
extrusion timestamp. Advanced travel uses dated tops instead of static tops
when printed tops are active. Same-attractor compatibility routing still has
its own top avoidance.
All three use the worker's borrowed LayerInputCache and return owned platter geometry.
An original island is a layer perimeter slice before inset; TravelBoundary also
uses the object layer at or below a support target. A membership mask decides
island membership, not a route. Support masks include that lower object layer;
support targets have no forbidden tops. DefaultAttractorContributor appends
Attractor zones to out; both top contributors append Forbidden zones.
No contributor replaces existing output.
    DefaultAttractorContributor attractors;
    DefaultForbiddenTopsContributor tops;
    TravelZoneView view({std::cref(attractors), std::cref(tops)}, input);
*/
#ifndef slic3r_plugins_default_zone_contributors_hpp_
#define slic3r_plugins_default_zone_contributors_hpp_

#include "TravelZones.hpp"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace AdvancedTravel {

class DefaultAttractorContributor : public ZoneContributor
{
public:
    // Copy original islands from temporary boundaries into per-instance membership runs; cancellation and geometry failures propagate.
    void contribute(const ZoneContributorInput &input, std::vector<TravelZone> &out) const override;
};

class DefaultForbiddenTopsContributor : public ZoneContributor
{
public:
    // Copy excluded tops from temporary boundaries into whole-layer instance zones; cancellation and geometry failures propagate.
    void contribute(const ZoneContributorInput &input, std::vector<TravelZone> &out) const override;
};

class PrintedTopsContributor : public ZoneContributor
{
public:
    // Append per-instance tops from the first travel after their original island's last arrival; failures propagate.
    void contribute(const ZoneContributorInput &input, std::vector<TravelZone> &out) const override;
};

}}}
#endif
