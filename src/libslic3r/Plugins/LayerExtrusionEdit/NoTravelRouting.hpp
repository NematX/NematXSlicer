///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_Plugins_LayerExtrusionEdit_NoTravelRouting_hpp_
#define slic3r_Plugins_LayerExtrusionEdit_NoTravelRouting_hpp_

/*
NoTravel volumes mark areas that travels should avoid. These helpers let travel
providers share one best-effort policy: slice the volumes once per layer worker,
then ask path() for each gap. path() keeps the direct gap unless it enters a
section, then tries one Clipper detour along the section borders. A section
containing an endpoint is exempt for that gap, and a detour that still enters a
section is rejected in favour of the direct gap. Both cases are counted so that
the provider can publish one warning per execution with warning_message().
Layer entries and gaps whose endpoints are not at the layer print_z keep their
direct path. No setting is read and no clearance is added around the volumes.

Typical use inside a layer worker, with the plan, layer group and run context of
that callback:

    const polygon_work::ExPolygons obstacles = NoTravel::sections(plan, layer.print_z(), run_ctx);
    NoTravel::Counts counts;
    connector.apply(chronology, [&](const TravelEndpoint &source, const TravelEndpoint &target,
                                    extruder_index_t tool) {
        return NoTravel::path(source, target, tool, layer_entry, layer.print_z(), obstacles, run_ctx, counts);
    }, progress);

The sections are owned values; Clipper operands and API collections live only
inside each call and use the worker's plugin storage.
*/

#include <cstdint>
#include <string>
#include <vector>

#include "libslic3r/Plugins/Common/PolygonWorkBuffers.hpp"
#include "TravelConnectionHelpers.hpp"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace NoTravel {

namespace polygon_work = PluginCommon::polygon_work;

// Best-effort outcomes accumulated by one worker, then added to execution-wide totals.
struct Counts
{
    // Gaps that ignored at least one section containing one of their endpoints.
    uint64_t exemptions = 0;
    // Gaps that kept their direct path because the detour still entered a section.
    uint64_t fallbacks = 0;
};

// Slice the NoTravel volumes of every real plan instance at print_z and union them in platter
// coordinates, holes preserved. Return an empty set when the plan has no NoTravel volume.
// Cancellation and allocation failures propagate.
polygon_work::ExPolygons sections(const ConstPrintingPlan &plan, coord_t print_z,
                                  const plugin_run_context *run_ctx);

// Return the direct gap for a layer entry, an endpoint off print_z, empty sections or a gap that
// avoids every non-exempt section; otherwise one validated detour, or the direct gap when the
// detour is rejected. Update counts. Cancellation and allocation failures propagate.
std::vector<c_point> path(const TravelConnection::TravelEndpoint &source,
                          const TravelConnection::TravelEndpoint &target, extruder_index_t tool,
                          bool layer_entry, coord_t print_z, const polygon_work::ExPolygons &sections,
                          const plugin_run_context *run_ctx, Counts &counts);

// Return the execution summary for nonzero totals, or an empty string when every gap was routed.
std::string warning_message(uint64_t exemptions, uint64_t fallbacks);

}}} // namespace slic3r_api::LayerExtrusionEdit::NoTravel

#endif // slic3r_Plugins_LayerExtrusionEdit_NoTravelRouting_hpp_
