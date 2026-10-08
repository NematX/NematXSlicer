///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Support toolpaths of one role, written with the plugin API only. The data tree
   supplies the auxiliary layers, their surfaces and the extrusion destinations.
   A support slab is the volume between a layer's top Z and its printable bottom;
   nearby slabs can shorten an extrusion's height without moving its top Z. The
   internal helpers are exposed so tests can drive them with synthetic slabs
   instead of a full sliced print. */
#ifndef plugins_support_toolpath_paths_hpp_
#define plugins_support_toolpath_paths_hpp_
#include <string>
#include <vector>

#include "SupportToolpathLegacy.hpp"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"

namespace slic3r_api::Support::Toolpath {
// Generate one surface role with the pattern plugins and publish paths through the data tree.
void generate_paths(const plugin_run_context *context, SurfaceRole role);

namespace Internal {
// One support slab below the current one: its top Z and its footprint.
struct Overlap {
    coord_t z;
    StoredPolygonCollection polygons;
};
// Choose the pattern plugin id of one surface part; a part printed as base uses the base pattern where
// the base filler is allowed, and the rectilinear pattern otherwise, and supportbase rejects bridging
// parameters, so a bridging contact that is not printed as base falls back to rectilinear.
std::string surface_pattern(bool as_base, bool contact, bool bridge, bool base_filler, const std::string &contact_pattern, const std::string &base_pattern);
// Split each path of out where it overlaps the listed slabs and scale its flow by the remaining height.
void modulate(storage_handle *storage, MutableExtrusionEntity out, coord_t z, const std::vector<Overlap> &overlapping, coord_t resolution);
}
}
#endif
