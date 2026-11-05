///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_support_common_supportbridgestrokes_hpp_
#define plugins_support_common_supportbridgestrokes_hpp_

#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"

/*
Bridge strokes of printed extrusions
====================================

The demand-removal plugins of dont_support_bridges (support.demand.bridge_removal for
the perimeters and gap fills, support.demand.bridge_infill_removal for the bridge
infill) decide which printed lines are bridges with the same rule, shared here.

A straight segment of a printed line is a bridge when its middle is not over the
lower layer and both of its ends, extended by the flow width, are over it. The
area such a line covers (its stroke) is what the plugins subtract from the support
demand. The usual sequence is:

    bridge_line_flow()
    `-- width, spacing and nozzle of the lines to examine, from the regions of an island
    grown_lower_support()
    `-- the lower layer slices grown by half a nozzle: a line over that margin is supported
    for each extrusion tree of the island:
        append_entity_strokes()          every straight segment of the tree, as it is printed
        or append_polyline_strokes()     the segments of one leaf, collinear ones merged first
        `-- append_segment_strokes()
            |-- split the segment at its intersections with the lower support
            |-- skip every piece whose middle is over the lower support
            |-- extend the other pieces at both ends by the flow width
            |-- skip a piece with an extended end that is not over the lower support
            `-- add the piece grown by half the line width to the strokes

Conventions shared by every function:
- Lengths are scaled coordinates (coord_t) or scaled doubles; the C views use the same units.
- Geometry arguments (island, layers, extrusion trees, lower support) are borrowed read-only
  views: they must stay valid for the call and are not modified.
- The strokes are an owned ClipperOperand of the caller, in the storage of its ClipperContext;
  strokes accumulate in it without being merged (union them afterwards).
- Results returned by value are owned by the caller (in its clipper storage).
- Curved segments (radius different from 0) are ignored, never bridges.
- The helpers have no plugin run context, so they never check cancellation: the callers check it (throw_if_cancelled) between
  islands and layers, and one call is bounded by the lines of one extrusion tree.
- A flow read from the host can fail (the host then returns an empty flow with a width of 0): bridge_line_flow() leaves such a region
  out, and an island without any usable flow keeps the defaults of BridgeLineFlow (0.4 mm extension, 0.2 mm half width, 0.2 mm probe).
- The exceptions that can leave a function are std::bad_alloc, and the std::invalid_argument / std::runtime_error that the clipper and
  geometry views throw when the host rejects an argument; a function that throws leaves the strokes it was given in a valid state
  (pieces added so far stay, none is partially added).
*/

namespace slic3r_api::Support::Common {

// The dimensions that decide what a printed line covers and where it counts as supported.
struct BridgeLineFlow
{
    // Length added at both ends of an unsupported piece before testing that the ends are supported (scaled).
    coord_t endpoint_extension = scale_i(0.4);
    // Half the width of the stroke of a line (scaled).
    double half_width = scale_d(0.2);
    // Margin added around the lower layer slices: a line over that margin counts as supported (scaled).
    double support_probe_offset = scale_d(0.2);
};

// The dimensions of the lines of a role over the regions of an island: the narrowest flow width for the extension, the widest line for the
// stroke and the largest nozzle for the probe; the values of a region without flow are left out, and an island without any keeps the
// defaults. `bridging` selects the bridging flow of the role instead of its normal flow. The island is borrowed.
BridgeLineFlow bridge_line_flow(const LayerIsland &island, raw_extrusion_role role, bool bridging);

// The flow of a line printed with the given width: the width gives the extension and the stroke, the probe offset is the one of `base`.
// The base is read only.
BridgeLineFlow bridge_line_flow_of_width(const BridgeLineFlow &base, coord_t line_width);

// The slices of the lower layer grown by the probe offset of the flow, as a collection owned by the clipper storage. The layer is borrowed.
StoredExPolygonCollection grown_lower_support(const ClipperContext &clipper, const Layer &lower_layer, const BridgeLineFlow &flow);

// Add to `strokes` the strokes of the bridge pieces of one straight segment. `lower_support` is the grown lower support (borrowed); `storage`
// holds the temporary geometry of the call. A curved or degenerate segment adds nothing.
void append_segment_strokes(storage_handle *storage,
                            const ExPolygonCollection &lower_support,
                            const BridgeLineFlow &flow,
                            const ClipperContext &clipper,
                            const c_extrusion_segment &segment,
                            ClipperOperand &strokes);

// Add to `strokes` the strokes of the bridge pieces of every segment of an extrusion tree, each segment as it is printed.
void append_entity_strokes(storage_handle *storage,
                           const ExPolygonCollection &lower_support,
                           const BridgeLineFlow &flow,
                           const ClipperContext &clipper,
                           const ExtrusionEntity &entity,
                           ClipperOperand &strokes);

// Add to `strokes` the strokes of the bridge pieces of the polyline of one extrusion leaf. Consecutive straight segments on the same line
// (same direction, within SCALED_EPSILON of it) are merged into one before the rule applies, so a line split into collinear segments is
// tested as a whole, like the old support code tested whole polylines. A leaf without polyline adds nothing.
void append_polyline_strokes(storage_handle *storage,
                             const ExPolygonCollection &lower_support,
                             const BridgeLineFlow &flow,
                             const ClipperContext &clipper,
                             const ExtrusionEntity &leaf,
                             ClipperOperand &strokes);

} // namespace slic3r_api::Support::Common

#endif // plugins_support_common_supportbridgestrokes_hpp_
