///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

/*
Default contributors supply instance-scoped membership masks and excluded tops
to TravelZoneView. Pass either contributor in the view's contributor list.
Attractors copy original islands, not routing rings, and are active only during
consecutive travels targeting that island. Forbidden tops last for the layer.
Both use the worker's borrowed LayerInputCache and return owned platter geometry.
An original island is a layer perimeter slice before inset; TravelBoundary also
uses the object layer at or below a support target. A membership mask decides
island membership, not a route. Support masks include that lower object layer;
support targets have no forbidden tops. DefaultAttractorContributor appends
Attractor zones to out; DefaultForbiddenTopsContributor appends Forbidden zones.
Neither replaces existing output.
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

}}}
#endif
