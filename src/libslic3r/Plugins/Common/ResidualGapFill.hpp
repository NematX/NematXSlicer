///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_residual_gap_fill_hpp_
#define slic3r_residual_gap_fill_hpp_

#include <functional>
#include <vector>
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"

/* Residual gap filling for SDK toolpath providers. Snapshot existing paths with
   collect_paths(), subtract coverage() from the permitted domain, then generate()
   nominal additions. Resolve percentage settings with resolve_limits() first.
   Callers apply flow multipliers or support modulation only to the additions.
   ordered_boundary() preserves old properties; append_additions() publishes after
   them. Storage is borrowed from the host and must outlive returned collections.

       std::vector<Path> paths;
       collect_paths(tree.readonly(), paths);
       auto occupied = coverage(storage, paths, 1.f, Footprint::Spacing);
*/
namespace slic3r_api { namespace GapFill {

// Rounded ends of neighbouring paths overlap by height * (1 - pi/4), so each
// path covers its spacing rather than its width.
constexpr double k_rounded_section = 1. - PI / 4.;

// Independent linear centerline and its effective, possibly inherited attributes.
struct Path {
    std::vector<c_point> points;
    EPropertyAttributes attributes;
};

// Thresholds in millimeters, area in square millimeters, resolution in scaled units.
struct Limits {
    double min_width = 0., max_width = 0., min_area = 0., min_length = 0., extension = 0.;
    coord_t resolution = 0;
};
using EffectiveValue = std::function<double(const char *key, double base)>;

// Resolve the five thresholds against width, using squared width for area.
Limits resolve_limits(const EffectiveValue &value, double width, coord_t resolution);

// Minimum greater than maximum means that no printable width is permitted.
struct WidthRange { double minimum = 0., maximum = 0.; };
// Bound medial widths by the nozzle, flow, settings and bridge diameter.
WidthRange width_range(const c_flow &flow, const Limits &limits);
// Append linear path snapshots with inherited attributes; invalid dimensions throw.
void collect_paths(const ExtrusionEntity &tree, std::vector<Path> &out,
                   const EPropertyAttributes *inherited = nullptr);

// Callers select Physical for unrelated obstacles and bridged flows.
enum class Footprint { Spacing, Physical };
// Offset each path by useful spacing or physical width, preserving path order.
StoredPolygonCollection coverage(storage_handle *, const std::vector<Path> &,
                                 float spacing_ratio, Footprint);
// Open and filter residual areas, then build nominal paths within allowed geometry.
StoredExtrusionEntity generate(storage_handle *, const ExPolygonCollection &residual,
                               const ExPolygonCollection &allowed, const c_flow &flow,
                               const Limits &, raw_extrusion_role role,
                               const plugin_run_context *run_ctx = nullptr);
// Move old content and properties behind an ordered boundary and return its new queue.
MutableExtrusionEntity ordered_boundary(MutableExtrusionEntity tree);
// Move additions into the queue; insertion failure throws.
void append_additions(MutableExtrusionEntity queue, StoredExtrusionEntity &additions);

}} // namespace slic3r_api::GapFill
#endif
