///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

/*
Default contributors supply instance-scoped routing domains and excluded tops
to TravelZoneView. Pass the selected contributors in the view's contributor list.
Each forbidden top component is a separate zone, so ignoring an endpoint's top
does not ignore disjoint tops. Attractors publish each original perimeter slice
as one zone, retaining its source layer and instance provenance. They are active during
consecutive travels whose source or target belongs to that original slice on the
same layer and instance. Static forbidden tops last for the layer.
PrintedTopsContributor instead starts each top after the last arrival in its
original island, on the same layer and instance; islands without arrivals emit
no top. This is the first travel after that island's last arrival, not an
extrusion timestamp. Advanced travel uses dated tops instead of static tops
when printed tops are active. Only published forbidden zones restrict the advanced planner.
The attractor and top contributors use the worker's borrowed LayerInputCache and
return owned platter geometry; SupportObstacleContributor reads slices directly.
An original island is a layer perimeter slice before inset. Support domains include
the last object layer strictly below the support target's Z + SCALED_EPSILON;
support targets have no forbidden tops. DefaultAttractorContributor appends
Attractor zones to out; both top contributors append Forbidden zones.
No contributor replaces existing output. The planner later withdraws each M
component with the shared thin-wall helper, after forbidden material is removed.
SupportObstacleContributor copies object slices at or below the group's Z for
only the instances in the plan catalog. Each connected component is a separate
whole-layer forbidden zone for travels touching support, with no tool or instance
restriction. It reads slices directly, without the boundary cache.
    DefaultAttractorContributor attractors;
    DefaultForbiddenTopsContributor tops;
    TravelZoneView view({std::cref(attractors), std::cref(tops)}, input);
*/
#ifndef slic3r_plugins_default_zone_contributors_hpp_
#define slic3r_plugins_default_zone_contributors_hpp_

#include "TravelZones.hpp"
#include "../AvoidCrossing/AvoidCrossingInput.hpp"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace AdvancedTravel {

class DefaultAttractorContributor : public ZoneContributor
{
public:
    // Append one raw zone per source slice and instance, dated by source-or-target membership; failures propagate.
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

class SupportObstacleContributor : public ZoneContributor
{
public:
    // Append shifted object components at or below print Z for support travels; cancellation and geometry failures propagate.
    void contribute(const ZoneContributorInput &input, std::vector<TravelZone> &out) const override;
};

}}}
#endif
