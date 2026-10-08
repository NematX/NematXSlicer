///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "ClipperResults.hpp"

#include <cmath>
#include <cstdlib>

/* Implementation of the native-like Clipper results (see ClipperResults.hpp). */

namespace slic3r_api::Support::Common {
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