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
   need no helper. */
#ifndef plugins_support_common_clipper_results_hpp_
#define plugins_support_common_clipper_results_hpp_

#include <vector>

#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"

namespace slic3r_api::Support::Common {

// Whether a Clipper result ring is too small or too narrow to keep, like the native test_path()
// used by remove_small_areas() (ClipperUtils.cpp): its area is below 2 * SCALED_EPSILON times its
// approximate largest radius around the centroid.
bool is_small_area(const std::vector<c_point> &ring);

// Materialize a boolean result like the native *_ex() functions: drop the small rings, unite the
// others. The collection lives in the result's storage.
StoredExPolygonCollection native_expolygons(const ClipperOperand &result);

} // namespace slic3r_api::Support::Common

#endif // plugins_support_common_clipper_results_hpp_