///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Native-like results of Clipper boolean operations.

   The native ExPolygons booleans of ClipperUtils (diff_ex, intersection_ex,
   union_ex) do not keep the result of the operation as it is: they first drop
   the rings that are too small or too narrow (remove_small_areas()), then unite
   the remaining rings in a second pass, which also decides the order of the
   areas and the start point of each ring. The API operations return the first
   result directly. native_expolygons() applies the same two steps to an API
   result, so code ported from the native functions gets the same areas:

       // native: ExPolygons islands = diff_ex(contours, mask);
       StoredExPolygonCollection islands = native_expolygons(clipper_diff(clipper(contours), clipper(mask)));

   The polygon booleans (diff, intersection, union_) have no such filter and
   need no helper. to_polygons() flattens areas through the host without changing ring order: each contour is
   followed by its holes, and areas keep their input order. expolygon_rings() copies one area by individual insertion.
   append_polygons() uses that insertion form while retaining any existing destination rings.
   It uses individual ring insertion, whose native Polygon move can remove a near-closing vertex; the host collection
   flattening uses copies instead. These two forms retain their respective vertex-copy behavior. */
#ifndef plugins_support_common_clipper_results_hpp_
#define plugins_support_common_clipper_results_hpp_

#include <vector>

#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"

namespace slic3r_api::Support::Common {

// Host-copies areas into storage-owned contour-then-holes rings, preserving a near-closing last vertex.
StoredPolygonCollection to_polygons(storage_handle *storage, const ExPolygonCollection &areas);
// Individually inserts one area's contour then holes into owned rings; native moves may drop a near-closing last vertex.
StoredPolygonCollection expolygon_rings(storage_handle *storage, const ExPolygon &area);
// Returns borrowed contour-then-holes views without copying or dropping vertices; the area must outlive them.
std::vector<Polygon> polygon_rings(const ExPolygon &area);
// Individually inserts ordered area rings after existing destination rings; native moves may drop a near-closing last vertex.
void append_polygons(StoredPolygonCollection &destination, const ExPolygonCollection &areas);
// Individually inserts one contour then holes after existing destination rings; native moves may drop a near-closing last vertex.
void append_polygons(StoredPolygonCollection &destination, const ExPolygon &area);

// Whether a Clipper result ring is too small or too narrow to keep, like the native test_path()
// used by remove_small_areas() (ClipperUtils.cpp): its area is below 2 * SCALED_EPSILON times its
// approximate largest radius around the centroid.
bool is_small_area(const std::vector<c_point> &ring);

// Materialize a boolean result like the native *_ex() functions: drop the small rings, unite the
// others. The collection lives in the result's storage.
StoredExPolygonCollection native_expolygons(const ClipperOperand &result);

} // namespace slic3r_api::Support::Common

#endif // plugins_support_common_clipper_results_hpp_
