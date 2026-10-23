///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Writing support paths, shared by the support toolpath algorithms: the base pattern fill with its recipe,
   the path entities that carry the flow, stationary deposits, and the validation of polylines.
   A deposit carries one point and the island's net area times height as volume; the point is the contour centroid when
   the island contains it, else the closest boundary point (contour or holes). It bypasses polyline validation, which
   drops paths with fewer than two points.

   A base fill hands an area to an INFILL_PATTERN plugin through the pattern service of the payload. The recipe
   gives the pattern the flow and spacing of its lines, their density, angle and Z phase. With perimeter set, each
   area is first wrapped in a perimeter loop and the pattern fills its inset, as the native FillWithPerimeter does
   (see PerimeterFill.hpp).

       // context is the borrowed plugin_run_context * received by the calling plugin's run callback.
       const raw_infill_pattern_params recipe = base_fill_recipe(*payload, "rectilinear", role, flow, spacing, density, angle, 0.);
       fill_base_areas(context, *payload, storage, out, areas, layer, recipe, true); */
#ifndef plugins_support_common_paths_hpp_
#define plugins_support_common_paths_hpp_

#include <string>
#include <vector>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_toolpath.h"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/ExtrusionProperties.hpp"

namespace slic3r_api::Support::Common {

// Degrees to radians in float, as the native Geometry::deg2rad<float>().
float radians(float degrees);

// Recipe of one base pattern fill. pattern is the INFILL_PATTERN plugin id, resolved for this step run; role,
// flow, spacing (mm), density in (0, 1], angle (radians) and phase z (mm) are the values the legacy base paths use.
raw_infill_pattern_params base_fill_recipe(const run_ctx_support_toolpath &payload, const std::string &pattern,
                                           raw_extrusion_role role, const c_flow &flow, double spacing, float density,
                                           float angle, double z);

// Fill the areas with the recipe and append the paths to out. With perimeter, each area gets a perimeter loop and its
// inset is filled with the recipe pattern; otherwise the pattern fills each area alone. Stopping exits through silent
// PluginCancelled (the cause has its message, the step names it, or the runner rethrows the print cancellation);
// ordinary refusals throw std::runtime_error. context is the calling plugin's borrowed run context, used only after
// a host callback failure to exit through PluginCancelled when stopping; nullptr is accepted and disables this check.
void fill_base_areas(const plugin_run_context *context, const run_ctx_support_toolpath &payload, storage_handle *storage, MutableExtrusionEntity out,
                     const ExPolygonCollection &areas, const layer_handle *layer, const raw_infill_pattern_params &recipe,
                     bool perimeter);

// Append each polyline to parent as a path carrying the attributes, or as a fixed loop when it is closed. reversible
// tells whether the paths may be reversed; the loops never may.
void append_paths(storage_handle *storage, MutableExtrusionEntity parent, const std::vector<std::vector<c_point>> &lines,
                  const EPropertyAttributes &attributes, bool reversible);

// Append one stationary support deposit with net island area times flow height as volume. It sits at the contour centroid when the
// island contains it, else at the point of the island boundary closest to the centroid (a crescent has its centroid outside).
void append_deposit(storage_handle *storage, MutableExtrusionEntity parent, const ExPolygon &island, const c_flow &flow);

// Return a new collection with the polylines simplified by the host ensure_valid(Polylines), and leave the input untouched.
// Polylines with fewer than two points are dropped; each remaining polyline is simplified with Douglas-Peucker at the
// resolution (scaled units), and a polyline that the simplification reduces to two coinciding points is dropped.
std::vector<std::vector<c_point>> valid_polylines(storage_handle *storage, const std::vector<std::vector<c_point>> &lines,
                                                  coord_t resolution);

// Move every child of src to the end of dst, leaving src with empty children.
void move_children(MutableExtrusionEntity dst, MutableExtrusionEntity src);

} // namespace slic3r_api::Support::Common

#endif // plugins_support_common_paths_hpp_
