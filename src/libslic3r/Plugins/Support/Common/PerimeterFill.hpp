///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Perimeter-wrapped fill, the plugin-API version of the native
   FillWithPerimeter (Fill/FillBase.cpp). The legacy support uses it for the
   first-layer flange (rectilinear inside, ratio 0.2) and for the sheath around
   the sparse base (supportbase or honeycomb inside).

   How it works, for one surface:
   1. Shrink the surface by half a line spacing (offset2 with a miter join,
      then an intersection that removes the islands a large miter can merge).
      Each resulting area gets one perimeter: its contour as a closed loop,
      then each hole as a closed loop.
   2. Shrink that area again by (ratio_fill_inside + 0.5) spacings and grow it
      back by half a spacing: what remains is filled by the inner pattern,
      which overlaps the perimeter by half a line.
   3. Build the same extrusion tree as the native class:

        root (sortable, reversible)           only when there are several areas
          area (fixed order)                  the root itself when only one area
            perimeters (sortable, reversible)
              loop (fixed) -> closed path     one per contour or hole
            inner fill (sortable, reversible) one per inner area
              path or loop -> closed path     one per pattern polyline

      Every path carries the given role and flow (mm3 per mm, width, height)
      and is reversible unless monotonic is set.

   The inner pattern is a callback, so the helper does not depend on how the
   pattern is called. pattern_service_fill() builds the usual one, which calls
   an INFILL_PATTERN plugin through the STEP_SUPPORT_TOOLPATH payload (the
   pattern service); its recipe is the caller's raw_infill_pattern_params, and
   only the polylines of its output are kept, as the native class keeps only
   the polylines of the inner Fill.

       const PerimeterFillSettings settings{spacing, 0.2f, role, flow};
       fill_with_perimeter(storage, surface, settings,
                           pattern_service_fill(*payload, rectilinear_id, recipe, layer, island, region, primary),
                           output);

   The native class also applies the "fill exactly" flow correction and the
   recipe flow_mult; the legacy support uses neither (dont_adjust, flow_mult 1),
   so they are not reproduced. */
#ifndef plugins_support_common_perimeter_fill_hpp_
#define plugins_support_common_perimeter_fill_hpp_

#include <functional>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_toolpath.h"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"

namespace slic3r_api::Support::Common {

struct PerimeterFillSettings
{
    // Line spacing of the fill (mm), the native Fill::get_spacing().
    double spacing = 0.;
    // Extra inset of the inner area, in line spacings (native ratio_fill_inside).
    float ratio_fill_inside = 0.f;
    // Role and flow written on every path.
    raw_extrusion_role role = RAW_EXTRUSION_ROLE_NONE;
    c_flow flow = {};
    // Overlap between the perimeter and the surface border, 0 to 1 (native overlap_ratio).
    float overlap_ratio = 0.f;
    // Monotonic paths keep their direction (native FillParams::monotonic).
    bool monotonic = false;
};

/*
Fill one inner area. surface is a storage-owned Surface holding the area,
with the type of the filled surface; output is an empty entity the callback
fills with paths. Throw to report an error.
*/
using InnerFill = std::function<void(const surface_handle *surface, MutableExtrusionEntity output)>;

/*
Fill surface (its expolygon and type) with a perimeter and the inner pattern,
and append one root entity to out when anything was generated. Temporary
geometry lives in storage and is released before returning.
*/
void fill_with_perimeter(storage_handle *storage, const Surface &surface, const PerimeterFillSettings &settings,
                         const InnerFill &inner_fill, MutableExtrusionEntity out);

/*
Inner fill calling the INFILL_PATTERN plugin pattern_id through the pattern
service of a STEP_SUPPORT_TOOLPATH payload, with recipe params. layer, island
and region_island describe where the surface is printed and are passed to the
pattern as context. primary_region supplies the print settings the pattern
reads: smooth and ensuring return nothing when it is null. The payload and
params must outlive the returned callback. A call the service refuses (unknown
id, pattern needing STEP_INFILL initialization) throws std::runtime_error.
*/
InnerFill pattern_service_fill(const run_ctx_support_toolpath &payload, infill_pattern_runtime_id pattern_id,
                               const raw_infill_pattern_params &params, const layer_handle *layer,
                               const layer_island_handle *island, const layer_region_island_handle *region_island,
                               const layer_region_handle *primary_region);

} // namespace slic3r_api::Support::Common

#endif // plugins_support_common_perimeter_fill_hpp_
