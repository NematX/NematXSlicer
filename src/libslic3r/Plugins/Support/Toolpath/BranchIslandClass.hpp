///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Printing method of a fused branch island: the limits that choose it, and the function that applies them.

   The cross-section of a tree branch is a wide disc near the trunk and a few extrusion widths across near a tip, and
   tips get thinner as the layers go up. A sheath, which walls the outline with a loop half a width inside it, needs
   room: in a part narrower than 1.5 w its two sides overlap each other. Each fused island of a support layer (the
   branch surfaces of one region island, closed by one epsilon) therefore gets exactly one method, chosen from its
   shape, where w is the support extrusion width:

       thin part      the island minus its opening by r = 0.75 w, i.e. everything narrower than 1.5 w
       thin fraction  area(thin part) / area(island)
       compactness    Q = 4 pi A / P^2 with P the length of all closed boundaries (1 for a disc, 0.785 for a square,
                      0.698 for a rectangle twice as long as wide, lower the more elongated or holed the island is)

       thin fraction < 50 %              Sheath      the sheath, as for any wide island
       thin fraction >= 50 %, Q < 2 pi/9 Centreline  the medial axis of the island, one variable-width line
       thin fraction >= 50 %, Q >= 2 pi/9 Blob       the island is a small compact dot (a medial axis of a small
                                                      disc is a point); one stationary deposit prints
                                                      its net area times layer height, at the contour centroid
                                                      when the island contains it, else at the closest boundary
                                                      point (contour or holes)

   The opening uses the default miter join of the Clipper offsets, which keeps the convex corners: the opening of a
   rectangle is the rectangle, so a 2 x 1 mm island has no thin part.

   The centreline is the output of the medial axis helper, configured by centreline_widths(). Where the island is S wide
   (a spacing), the helper prints the extrusion width l = S + h (1 - pi / 4) sr, with h the layer height and sr the spacing
   ratio. A part whose width l is below the nozzle diameter b is widened to b when the widening is small (under
   centreline_max_widening) and within the nominal support XY budget; narrower parts are trimmed; an island left with no line prints a deposit (BranchSheathPaths.cpp). The widest island part
   the line covers is 1.5 w.

   These limits are tunable named constants below; no other copy of their values exists.

       const BranchIslandMeasure measure = classify_island(storage, island, flow.width);
       if (measure.kind == BranchIslandClass::Centreline) {
           const CentrelineWidths widths = centreline_widths(flow, xy_distance);
           ... medial_axis_extrusion(role, flow).medial_widths(widths.min_medial_width, widths.max_medial_width)
                   .extrusion_widths(widths.min_extrusion_width, 0) ...
       }  */
#ifndef plugins_support_toolpath_branch_island_class_hpp_
#define plugins_support_toolpath_branch_island_class_hpp_

#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"

namespace slic3r_api::Support::Toolpath {

// Radius r of the opening that finds the thin part of an island, as a multiple of the support extrusion width w. It decides
// what counts as thin: every part narrower than 2 r = 1.5 w, where the two sides of a sheath loop would overlap each other.
// It stays above 0.5, the half width at which the sheath is offset from the outline: a sheath-mode island then always has a
// sheath (see BranchSheathPaths.cpp).
constexpr double thin_radius_over_width = 0.75;

// Share of the island area that the thin part must reach for the island to be thin as a whole. It decides that an island
// gets one method: below it the sheath prints and its small thin leftovers stay unfilled, from it on the medial axis or the
// blob prints for the whole island.
constexpr double thin_fraction_limit = 0.5;

// Compactness Q = 4 pi A / P^2 under which a thin island is elongated. It decides between the centreline and the blob: the
// value is the Q of a rectangle twice as long as wide (2 pi / 9), so a thin island longer than 2:1 gets a centreline and a
// more compact one a blob.
constexpr double compactness_limit = 2. * PI / 9.;

// Largest relative widening of a centreline part. A part whose extrusion width l is below nozzle diameter b may be widened
// to b when Delta = b - l is under this share of l (Delta / l < 0.1) and Delta / 2 does not exceed gap_xy, the nominal
// support XY budget selected for widening. This is not a geometric clearance guarantee. Growth skips points inside anchors = diff_ex(bounds,
// m_expolygon). With no separate extension bounds, bounds is the island and m_expolygon is its frontier after
// simplify_polygon_frontier() applies remove_collinear_angle(1 degree). A point in a resulting contour sliver, such as an
// endpoint extended onto the contour, is skipped by growth and keeps its then-current width (at least the trim threshold before
// volume scaling) instead of reaching b. ensure_not_overextrude() scales widths only when the line volume exceeds a bound based
// on island area times layer height plus contour and hole gap terms. A narrower part is trimmed, and an island narrower
// everywhere has no line and prints a stationary deposit instead (BranchSheathPaths.cpp). The nozzle sets b; the extrusion width w sets the case.
constexpr double centreline_max_widening = 0.1;

// Widest island part that the centreline covers, as a multiple of w: the diameter 2 r of the opening. It bounds the width of
// the island (a spacing), so the extrusion width of the line may reach 1.5 w + h (1 - pi / 4) sr. Wider parts of a thin island
// (under half of its area) get no line, or a line capped to this width.
constexpr double centreline_max_width_over_width = 2. * thin_radius_over_width;

// The method that prints a fused island.
enum class BranchIslandClass { Sheath, Centreline, Blob };

// The measures of an island and the method they select.
struct BranchIslandMeasure {
    // Share of the island area that the opening by r removes.
    double thin_fraction = 0.;
    // Q = 4 pi A / P^2, the area A of the island over the squared length P of all its closed boundaries.
    double compactness = 0.;
    BranchIslandClass kind = BranchIslandClass::Sheath;
};

// Measure a fused island, whose extrusion width is width (scaled units), and choose its method. storage owns the temporary
// offsets. A thin island that is not elongated is a Blob even when its sheath would not vanish.
BranchIslandMeasure classify_island(storage_handle *storage, const ExPolygon &island, coord_t width);

// The width settings of the medial axis helper for the centreline of an island, in scaled units. The helper works in spacings:
// a part of the island S wide is printed with the extrusion width S + h (1 - pi / 4) sr.
struct CentrelineWidths {
    // Narrowest island width kept: narrower parts are trimmed away.
    coord_t min_medial_width = 0;
    // Widest island width covered by the line.
    coord_t max_medial_width = 0;
    // Setting that makes the helper widen every kept part narrower than the nozzle to the extrusion width b exactly (the growth
    // adds the rounded ends once for the spacing ratio 1 and the conversion to an extrusion width adds them for the real ratio).
    coord_t min_extrusion_width = 0;
};

// The widths of the centreline of an island printed with the flow, whose nozzle_diameter sets the extrusion width b of the
// widened parts. xy_distance is the support XY distance (scaled units). A flow whose widest width 1.5 w is under the narrowest
// gives a maximum under the minimum, which the helper refuses: the island has no line and prints a deposit.
CentrelineWidths centreline_widths(const c_flow &flow, coord_t xy_distance);

} // namespace slic3r_api::Support::Toolpath

#endif // plugins_support_toolpath_branch_island_class_hpp_
