///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Support toolpaths of one role, written with the plugin API only. The data tree
   supplies the auxiliary layers, their surfaces and the extrusion destinations.
   A support slab is the volume between a layer's top Z and its printable bottom;
   nearby slabs can shorten an extrusion's height without moving its top Z. The
   internal helpers are exposed so tests can drive them with synthetic slabs
   instead of a full sliced print.

   An optional InterfaceFill replaces only the fill of dense passes above the
   bed. Contact loops, base passes, bed flanges, flow modulation and publication
   retain their usual handling. This is an internal host C++ extension, not a
   C ABI promise for external DLLs. The callback is synchronous; all context
   views and the output are borrowed for that invocation only.
   For example: generate_paths(context, SurfaceRole::TopInterface, custom_fill). */
#ifndef plugins_support_toolpath_paths_hpp_
#define plugins_support_toolpath_paths_hpp_
#include <functional>
#include <string>
#include <vector>

#include "SupportToolpathLegacy.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_toolpath.h"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"

namespace slic3r_api {
class Object;
}

namespace slic3r_api::Support::Toolpath {
// Borrow the effective geometry and recipe of one dense pass before modulation and publication.
struct InterfaceFillContext {
    const plugin_run_context *context;
    const run_ctx_support_toolpath &payload;
    storage_handle *storage;
    const Object &object;
    const ExPolygonCollection &areas;
    const layer_handle *layer;
    const std::string &pattern;
    // Exact millimeter spacing of the effective flow before scaled-coordinate rounding.
    double flow_spacing;
    const raw_infill_pattern_params &recipe;
};
// Append owned children to the borrowed output while preserving their properties.
// Do not retain context views or output handles; exceptions propagate to the provider.
using InterfaceFill = std::function<void(const InterfaceFillContext &pass, MutableExtrusionEntity output)>;
// Generate one surface role with the pattern plugins and publish paths through the data tree.
// An empty callback retains ordinary fill; a supplied callback fills only non-base passes above the bed.
void generate_paths(const plugin_run_context *context, SurfaceRole role, const InterfaceFill &interface_fill = {});

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
// The index of the object layer that a contact at the expected Z is matched with: among the layers whose key (bottom_z when bottom, else print_z)
// is within tolerance of expected, the one whose key is nearest to it; on a tie the first in layer order (the lower one). layer_count() when the
// window holds no layer. Layers are sorted by key.
uint32_t matched_layer(const Object &object, coord_t expected, coord_t tolerance, bool bottom);
}
}
#endif
