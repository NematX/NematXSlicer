///|/ Released under the GNU Affero General Public License, version 3 or later.
#ifndef nematx_interface_gap_fill_hpp_
#define nematx_interface_gap_fill_hpp_

#include <vector>
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"

/* Geometry used by the support gap-fill provider and its SDK tests. Snapshot
   paths with collect_paths(), compute their coverage(), then generate only the
   remaining areas with generate(). All returned geometry belongs to storage.
   Callers choose the flow of the interface pass before generate(), then adjust
   the new paths to lower support layers and publish them. */
namespace nematx::interface_gaps {
using namespace slic3r_api;
// Copy of one printed path: its points and its effective, possibly inherited, attributes.
struct Path {
    std::vector<c_point> points;
    EPropertyAttributes attributes;
};
// Gap-fill thresholds in millimeters (min_area in mm2), percentages already resolved
// against the flow width; resolution is in scaled coordinates.
struct Limits {
    double min_width = 0., max_width = 0., min_area = 0.;
    double min_length = 0., extension = 0.;
    coord_t resolution = 0;
};
// Copy printable leaves with inherited attributes; invalid attributes throw.
void collect_paths(const ExtrusionEntity &tree, std::vector<Path> &out,
                   const EPropertyAttributes *inherited = nullptr);
// Offset snapshots by useful spacing, or physical width for an unrelated recipe.
StoredPolygonCollection coverage(storage_handle *storage, const std::vector<Path> &paths,
                                 const c_flow &flow, bool physical = false);
// Fill narrow residuals through the SDK, preserving nominal flow until modulation.
StoredExtrusionEntity generate(storage_handle *storage, const ExPolygonCollection &residual,
                               const ExPolygonCollection &allowed, c_flow flow, const Limits &limits);
}
#endif
