///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "BranchIslandClass.hpp"

#include <algorithm>
#include <cmath>

#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"

/* Classification of a fused branch island (the rules and limits are described in BranchIslandClass.hpp).

   The thin part is found with an opening: the island is shrunk by r and grown back by r, which keeps every part wide
   enough to hold a disc of radius r and drops the rest. The thin fraction is the area lost by the opening over the area
   of the island. The compactness uses the length of every boundary, the closing edge of each ring included: the API
   length of a polygon is the length of its open polyline, so the edge from the last point to the first is added.

   The centreline widths use two settings of the medial axis helper (slic3r_geometry.cpp configure_medial_axis()). The helper
   trims the parts of the island narrower than min_medial_width, a spacing. Its min_extrusion_width activates the growth of
   MedialAxis::grow_to_nozzle_diameter(), which runs after the trim and conditional volume scaling: it raises each unanchored
   point narrower than Flow::new_from_spacing(X).width = X + h (1 - pi / 4) (spacing ratio 1) to that value, and the conversion
   to an extrusion width adds h (1 - pi / 4) sr. Choosing X = b - h (1 - pi / 4) (1 + sr) therefore makes grown parts exactly
   b wide. Growth leaves points inside the anchors (the bounds minus the simplified frontier) at the widths they have after
   trimming and any volume adjustment. */

namespace slic3r_api::Support::Toolpath {
namespace {
// The length of a closed ring: its open length plus the edge from the last point back to the first.
double closed_length(const Polygon &ring);

double closed_length(const Polygon &ring)
{
    const c_point first = ring.front(), last = ring.back();
    return double(ring.length()) + std::hypot(double(last.x) - double(first.x), double(last.y) - double(first.y));
}
} // namespace

BranchIslandMeasure classify_island(storage_handle *storage, const ExPolygon &island, coord_t width)
{
    BranchIslandMeasure measure;
    const double area = island.area();
    const double radius = thin_radius_over_width * double(width);
    const ClipperContext clipper(storage);
    // The default miter join keeps the convex corners, so a rectangle wider than 2 r is its own opening.
    const StoredExPolygonCollection opened = clipper_offset2(clipper(island), -radius, radius).to_expolygon_collection();
    double opened_area = 0.;
    for (const ExPolygon part : opened)
        opened_area += part.area();
    measure.thin_fraction = (area - opened_area) / area;

    double perimeter = closed_length(island.contour());
    for (const Polygon hole : island.holes())
        perimeter += closed_length(hole);
    measure.compactness = 4. * PI * area / (perimeter * perimeter);

    if (measure.thin_fraction < thin_fraction_limit)
        measure.kind = BranchIslandClass::Sheath;
    else if (measure.compactness < compactness_limit)
        measure.kind = BranchIslandClass::Centreline;
    else
        measure.kind = BranchIslandClass::Blob;
    return measure;
}

// Derive scaled trim, growth, and maximum settings from the flow and nominal support XY distance; return widths by value without
// changing or owning the inputs. The medial-axis helper yields no lines when the maximum is below the trim width.
CentrelineWidths centreline_widths(const c_flow &flow, coord_t xy_distance)
{
    const double nozzle = double(flow.nozzle_diameter);
    // The extra width of the rounded ends of a line, h (1 - pi / 4), for a spacing ratio of 1.
    const double rounded_ends = double(flow.height) * (1. - 0.25 * PI);
    // The narrowest extrusion width kept: the widening to the nozzle stays under the relative limit, and half of it on each
    // side stays within the nominal support XY budget.
    const double narrowest = std::max(nozzle / (1. + centreline_max_widening), nozzle - 2. * double(xy_distance));
    CentrelineWidths widths;
    widths.min_medial_width = coord_t(std::lround(narrowest - rounded_ends * double(flow.spacing_ratio)));
    widths.max_medial_width = coord_t(std::lround(centreline_max_width_over_width * double(flow.width)));
    widths.min_extrusion_width = coord_t(std::lround(nozzle - rounded_ends * (1. + double(flow.spacing_ratio))));
    // The helper skips growth when these settings are equal. Round the trim limit up one unit so widths below the limit stay trimmed.
    if (widths.min_extrusion_width == widths.min_medial_width)
        ++widths.min_medial_width;
    return widths;
}

} // namespace slic3r_api::Support::Toolpath
