///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_Plugins_LayerExtrusionEdit_TravelConnectionHelpers_hpp_
#define slic3r_Plugins_LayerExtrusionEdit_TravelConnectionHelpers_hpp_

/*
Ordered travel connection helpers
=================================

Travel providers share the delicate tree mutation needed to connect compact
PrintingPlan scopes. This module owns endpoint tracking, epsilon snapping,
reserved-phase validation, Travel attributes, final materialization flags and
Z interpolation. A provider only chooses the two-dimensional path used for a
real gap.

ScopeTravelConnector mirrors the three callbacks of a layer-extrusion-edit
plugin. setup() allocates a small summary slot per PrintingLayerGroup,
setup_run() publishes each layer's first-boundary decision independently, and
run() connects the layer's scopes.
The host barrier makes summaries immutable before any run reads another layer.
annotate() may then add one G-code comment per travel, for verbose output; it
places the comment first in the target scope's before phase, right after the
inserted travel, so it leaves the travel phase a single geometric leaf.

run() has two phases. prepare() records local pairs through
PrintingEntityPropertyTraversal and snaps nearby seams immediately, so later
sources can see the changed closed seam. apply() asks the provider for paths,
writes travels and updates flags.

A LayerTravelChronology owns its bookkeeping and borrowed plan views, not the
plan geometry. Apply it with the same connector before any structural plan
mutation or layer destruction.
pair_extrusions retains the target owner and tool of every traversed pair,
including pairs that are continuous or already have a materialized travel.
Calling reset() or setup() on that connector also invalidates the chronology,
because apply() reads the connector's next-layer summaries.
No property pointer or traversal context is retained. Typical use after setup_run
and the host barrier is:
    LayerTravelChronology chronology = connector.prepare(layer, group, index);
    connector.apply(chronology, planner, progress);

All tree mutations remain local to the current layer. In particular, a worker
never writes the previous layer's final scope: that scope's own worker updates
its outgoing flag from the next non-empty layer summary.
*/

#include <functional>
#include <string>
#include <vector>

#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Plugins/PrintingPlan/PrintingExtrusionScopeProperty.h"
#include "libslic3r/Plugins/PrintingPlan/PrintingLayerEntryStateProperties.h"

namespace slic3r_api {

class PluginProgress;

namespace LayerExtrusionEdit { namespace TravelConnection {

/* One absolute point in the coordinate system of the final PrintingPlan. */
struct PlannedPosition
{
    coord_t x = 0;
    coord_t y = 0;
    coord_t z = 0;
    bool known = false;
};

/*
Regional provenance for one side of a gap. interior_point lies up to
SCALED_EPSILON inside the geometric leaf and follows arcs; providers use it to
resolve settings when the endpoint itself lies on a region boundary.
*/
struct TravelEndpoint
{
    PlannedPosition position;
    LayerRegionIsland region_island;
    uint32_t printing_catalog_id = SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE;
    c_point interior_point{};
    bool has_interior_point = false;
};

/*
Return an ordered path including source and target. Invalid or shorter results
are rejected instead of being silently inserted into the extrusion tree.
*/
using TravelPathPlanner = std::function<std::vector<c_point>(
    const TravelEndpoint &source,
    const TravelEndpoint &target,
    extruder_index_t extruder_id)>;

/* One real gap after earlier seam snaps; source_extrusion is invalid when the travel starts from the layer entry. */
struct PendingTravel
{
    TravelEndpoint source;
    TravelEndpoint target;
    extruder_index_t extruder_id = 0;
    PrintingExtrusion source_extrusion{nullptr};
    PrintingExtrusion target_extrusion{nullptr};
    size_t pair_index = 0;
};

/* Own ordered gaps and application state, including print Z and borrowed scope roots; structural mutation, layer destruction or connector reset/setup invalidates it. */
struct LayerTravelChronology
{
    std::vector<PendingTravel> travels;
    // Target owner and tool for every traversed pair, including continuous and already materialized transitions.
    struct PairExtrusion
    {
        PrintingExtrusion target{nullptr};
        extruder_index_t extruder_id = EXTRUDER_ID_UNKNOWN;
    };
    std::vector<PairExtrusion> pair_extrusions;

private:
    friend class ScopeTravelConnector;
    coord_t print_z = 0;
    size_t flat_idx = 0;
    std::vector<MutableExtrusionEntity> roots;
    std::vector<bool> materialized;
    std::vector<coord_t> target_z;
    MutableExtrusionEntity final_root;
    bool final_has_outgoing = false;
};

/*
Shared lifecycle and tree editor used by one travel provider instance.

The object stores only runtime property keys, the stable flat layer mapping and
one scalar summary per layer. It never stores borrowed extrusion or property
pointers between callbacks. prepare() returns a layer-local chronology with
seams already snapped; apply() consumes its pair order to plan paths and update
flags. run() combines these phases for providers that need no intermediate work.
*/
class ScopeTravelConnector
{
public:
    /* Register the private scope and layer-entry properties. */
    explicit ScopeTravelConnector(orchestrator_handle *orchestrator);

    /* Discard all summaries from a previous PrintingPlan execution. */
    void reset();

    /* Allocate one summary slot per final PrintingLayerGroup. */
    void setup(const ConstPrintingPlan &plan, uint32_t run_count);

    /* Analyze one layer in its worker-owned slot before the host barrier. */
    void setup_run(PrintingLayerGroup &layer,
                   uint32_t group_idx,
                   uint32_t layer_idx,
                   PluginProgress &progress);

    /* Copy every pair's owner and tool plus ordered gap endpoints while snapping seams, without planning paths, writing travels or updating flags.
       Views borrow the stable plan; invalid indexes, unprepared summaries or invalid pairs throw. */
    LayerTravelChronology prepare(PrintingLayerGroup &layer,
                                  uint32_t group_idx, uint32_t layer_idx) const;

    /* Plan gaps in pair order, reacquiring phases and flags from borrowed roots, and update layer-owned flags and progress.
       On normal return the planner has been called exactly once per PendingTravel, in travels order.
       Invalid paths, lost markers or inter-layer disagreement throw; use the same connector before reset() or setup(). */
    void apply(LayerTravelChronology &chronology,
               const TravelPathPlanner &planner, PluginProgress &progress) const;

    /* Insert each non-empty comment as a comment event first in its target scope's before phase. comments is
       indexed like chronology.travels; call it after apply() on the same chronology. A size mismatch or a
       target without a before phase throws. */
    void annotate(const LayerTravelChronology &chronology, const std::vector<std::string> &comments) const;

    /* Prepare and apply consecutive scopes with a non-empty planner; either phase may throw after local edits. */
    void run(PrintingLayerGroup &layer,
             uint32_t group_idx,
             uint32_t layer_idx,
             const TravelPathPlanner &planner,
             PluginProgress &progress) const;

private:
    /* Information required by the previous layer's final scope. */
    struct LayerBoundarySummary
    {
        bool prepared = false;
        bool has_scope = false;
        bool first_has_incoming_transition = false;
        bool first_travel_materialized_after_run = false;
    };

    /* Convert final group/layer indexes into m_layers. */
    size_t flat_layer_index(uint32_t group_idx, uint32_t layer_idx) const;

    /* Find the next layer summary containing at least one scope. */
    const LayerBoundarySummary *next_nonempty_layer(size_t flat_idx) const;

    PluginPropertyKey<PrintingExtrusionScopeProperty> m_scope_property;
    PluginPropertyKey<PrintingLayerEntryPositionProperty> m_entry_position_property;
    bool m_setup_valid = false;
    std::vector<std::vector<size_t>> m_flat_indices;
    std::vector<LayerBoundarySummary> m_layers;
};

/* Build the direct two-point path used by the simple provider and fallbacks. */
std::vector<c_point> straight_path(const TravelEndpoint &source,
                                   const TravelEndpoint &target,
                                   extruder_index_t extruder_id);

}}} // namespace slic3r_api::LayerExtrusionEdit::TravelConnection

#endif // slic3r_Plugins_LayerExtrusionEdit_TravelConnectionHelpers_hpp_
