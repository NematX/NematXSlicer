///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "ClipperResults.hpp"

#include <cmath>
#include <cstdlib>

/* Native-like Clipper results and ordered ring copying (see ClipperResults.hpp). The host flattens each area
   as contour then holes; append operations insert each borrowed ring separately to retain native insertion's
   near-closing-vertex handling. polygon_rings() supplies ordered views without copying their vertices. */

namespace slic3r_api::Support::Common {
// Borrows valid area views and copies rings through the host into caller storage; no sorting or geometry changes occur, and allocation failures propagate.
StoredPolygonCollection to_polygons(storage_handle *storage, const ExPolygonCollection &areas)
{
    StoredPolygonCollection rings(storage);
    ::expolygons_to_polygons(areas.handle(), rings.mutable_handle());
    return rings;
}

// Inserts a borrowed area's rings individually into caller storage, retaining native ring insertion's vertex handling; the input is unchanged and allocation failures propagate.
StoredPolygonCollection expolygon_rings(storage_handle *storage, const ExPolygon &area)
{
    StoredPolygonCollection rings(storage);
    append_polygons(rings, area);
    return rings;
}

// Lists borrowed contour and hole views without copying vertices; the area must outlive the returned vector and allocation failures propagate.
std::vector<Polygon> polygon_rings(const ExPolygon &area)
{
    std::vector<Polygon> rings{area.contour()};
    for (const Polygon hole : area.holes())
        rings.push_back(hole);
    return rings;
}

// Inserts each borrowed area's ordered rings after existing destination rings; native insertion may remove a near-closing vertex, views must be valid and allocation failures propagate.
void append_polygons(StoredPolygonCollection &destination, const ExPolygonCollection &areas)
{
    for (const ExPolygon area : areas)
        append_polygons(destination, area);
}

// Inserts borrowed ordered ring views into destination storage without an intermediate polygon collection; native insertion may remove a near-closing vertex, existing rings remain and allocation failures propagate.
void append_polygons(StoredPolygonCollection &destination, const ExPolygon &area)
{
    for (const Polygon ring : polygon_rings(area))
        destination.push_back(ring);
}

bool is_small_area(const std::vector<c_point> &ring)
{
    // Signed area and centroid exactly as ClipperLib::Area() and ClipperLib::Centroid() compute them,
    // so the same rings pass the threshold as in the native filter.
    const size_t size = ring.size();
    double area = 0;
    if (size >= 3)
        for (size_t i = 0, j = size - 1; i < size; ++i) {
            area += (double(ring[j].x) + ring[i].x) * (double(ring[j].y) - ring[i].y);
            j = i;
        }
    area = -area * 0.5;
    double x_temp = 0, y_temp = 0;
    const int last = int(size) - 1;
    size_t i = 0;
    for (; int(i) < last; ++i) {
        const size_t j = i + 1;
        const double cross = double(ring[i].x) * ring[j].y - double(ring[j].x) * ring[i].y;
        x_temp += double(ring[i].x + ring[j].x) * cross;
        y_temp += double(ring[i].y + ring[j].y) * cross;
    }
    const double cross = double(ring[i].x) * ring[0].y - double(ring[0].x) * ring[i].y;
    x_temp += double(ring[i].x + ring[0].x) * cross;
    y_temp += double(ring[i].y + ring[0].y) * cross;
    const coord_t centroid_x = coord_t(x_temp / (6 * area));
    const coord_t centroid_y = coord_t(y_temp / (6 * area));
    area = std::abs(area);
    // The native code ORs the squared distances instead of taking their maximum: a cheap upper
    // bound, kept as is so the threshold is the same.
    coord_t max_dist_sqr = 0;
    for (const c_point &point : ring) {
        const coord_t dx = std::abs(point.x - centroid_x) & 0x3FFFFFFF;
        const coord_t dy = std::abs(point.y - centroid_y) & 0x3FFFFFFF;
        max_dist_sqr = max_dist_sqr | (dx * dx + dy * dy);
    }
    return area < (2 * SCALED_EPSILON) * std::sqrt(double(max_dist_sqr));
}

StoredExPolygonCollection native_expolygons(const ClipperOperand &result)
{
    StoredPolygonCollection rings = result.to_polygon_collection();
    for (uint32_t idx = rings.size(); idx-- > 0;)
        if (is_small_area(rings.at(idx).points()))
            rings.erase(idx);
    return clipper_union(ClipperOperand(result.storage(), rings.readonly())).to_expolygon_collection();
}

} // namespace slic3r_api::Support::Common
