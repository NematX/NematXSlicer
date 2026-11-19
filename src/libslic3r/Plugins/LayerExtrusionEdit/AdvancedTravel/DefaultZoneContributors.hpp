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
when printed tops are active.
The attractor and top contributors use the worker's borrowed LayerInputCache and
return owned platter geometry.
An original island is a layer perimeter slice before inset. Support domains include
the last object layer strictly below the support target's Z + SCALED_EPSILON;
support targets have no forbidden tops. DefaultAttractorContributor appends
Attractor zones to out; both top contributors append Forbidden zones.
NoTravelContributor appends the layer's NoTravel sections, already sliced by
NoTravelRouting.hpp, as Forbidden zones for every tool and instance during the
whole layer, one zone per connected section.
No contributor replaces existing output. The planner later withdraws each M
component with the shared thin-wall helper, after forbidden material is removed.
    DefaultAttractorContributor attractors;
    DefaultForbiddenTopsContributor tops;
    NoTravelContributor no_travel(sections);
    TravelZoneView view({std::cref(attractors), std::cref(tops), std::cref(no_travel)}, input);
*/
#ifndef slic3r_plugins_default_zone_contributors_hpp_
#define slic3r_plugins_default_zone_contributors_hpp_

#include "TravelZones.hpp"

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

class NoTravelContributor : public ZoneContributor
{
public:
    // Borrow platter-space sections sliced at the layer print_z; they must outlive every contribute() call.
    explicit NoTravelContributor(const polygon_work::ExPolygons &sections) : m_sections(sections) {}
    // Append one whole-layer Forbidden zone per section, for any tool and instance.
    void contribute(const ZoneContributorInput &input, std::vector<TravelZone> &out) const override;

private:
    const polygon_work::ExPolygons &m_sections;
};

}}}
#endif
