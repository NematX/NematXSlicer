///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Native Fill and Flow calculations use settings shared with the grid family.
   The data tree supplies auxiliary layers, surfaces and extrusion destinations.
   A support slab is the volume between a layer's top Z and printable bottom;
   nearby slabs can shorten an extrusion's height without moving its top Z.
   The internal helpers are exposed so tests can drive them with synthetic
   slabs instead of a full sliced print. */
#ifndef plugins_support_toolpath_paths_hpp_
#define plugins_support_toolpath_paths_hpp_
#include "SupportToolpathLegacy.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"

namespace slic3r_api::Support::Toolpath {
// Generate one surface role using native patterns and publish paths through the data tree.
void generate_paths(const plugin_run_context *context, SurfaceRole role);

namespace Internal {
// One support slab below the current one: its top Z and its footprint.
struct Overlap {
    coord_t z;
    Slic3r::Polygons polygons;
};
// Choose the filler pattern of one surface part; a part using the base filler keeps the base
// pattern, and FillSupportBase rejects full-infill and bridging parameters, so a bridging contact
// falls back to rectilinear.
Slic3r::InfillPattern surface_pattern(bool as_base, bool contact, bool bridge, bool base_filler, Slic3r::InfillPattern contact_pattern, Slic3r::InfillPattern base_pattern);
// Split each path where it overlaps the listed slabs and scale its flow by the remaining height.
void modulate(Slic3r::ExtrusionEntityCollection &out, coord_t z, const std::vector<Overlap> &overlapping, coord_t resolution);
}
}
#endif
