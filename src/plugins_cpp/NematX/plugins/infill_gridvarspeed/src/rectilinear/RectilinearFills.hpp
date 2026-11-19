///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ Copyright (c) Prusa Research 2016 - 2023 Vojtech Bubnik, Lukas Hejl, Lukas Matena
///|/ Copyright (c) Slic3r 2016 Alessandro Ranellucci
///|/ Released under the GNU Affero General Public License, version 3 or later.
#pragma once

// Only the connected single-direction sweep required by gridvarspeed. No host-private
// includes, nested pattern dispatch, monotonic solver or support connectors.
#include "ClipperHelpers.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"

namespace slic3r_api { namespace GridVarSpeedCopy { namespace RectilinearPatternsPlugin {
// The copied helpers live in the shared plugin namespaces, as in the host's RectilinearConnect.hpp.
namespace work = GridVarSpeedCopy::polygon_work;
namespace clipper = GridVarSpeedCopy::clipper;
namespace fills {
using namespace work;
using namespace clipper;
struct SegmentedIntersectionLine;
struct ExPolygonWithOffset;

class FillRectilinear {
public:
    void init_spacing(coordf_t spacing, const raw_infill_pattern_params &params);
    coordf_t get_spacing() const { return spacing_priv; }
    size_t layer_id = size_t(-1);
    double z = 0.;
    float angle = 0.f;
    bool can_angle_cross = false;
    BoundingBox bounding_box;
    coordf_t overlap = 0.;
    coord_t link_max_length = 0;
    coord_t loop_clipping = 0;
    storage_handle *plugin_storage = nullptr;

    bool fill_surface_by_lines(const slic3r_api::ExPolygon &, const raw_infill_pattern_params &,
                               float angleBase, float pattern_shift, Polylines &) const;
private:
    coordf_t spacing_priv = 0.;
    coord_t _line_spacing_for_density(const raw_infill_pattern_params &p) const {
        return scale_i((p.max_sparse_infill_spacing > 0 ? p.max_sparse_infill_spacing : get_spacing()) / p.density);
    }
    std::pair<float, Point> _infill_direction(const slic3r_api::ExPolygon &polygon,
                                             const raw_infill_pattern_params &p) const {
        const c_bounding_box box = polygon.contour().bounding_box();
        Point center((box.min.x + box.max.x) / 2, (box.min.y + box.max.y) / 2);
        if (!empty(bounding_box)) center = bounding_box.center();
        // FillGridVarSpeed::_layer_angle is always zero. No CAN_ANGLE_CROSS.
        return {float((p.bridge_angle >= 0 ? p.bridge_angle : angle) + 3.14159265358979323846 / 2.), center};
    }
    std::vector<SegmentedIntersectionLine> _vert_lines_for_polygon(
        const ExPolygonWithOffset &, const BoundingBox &, const raw_infill_pattern_params &, coord_t) const;
};

void configure_filler(FillRectilinear &, const raw_infill_pattern_params &, storage_handle *);
}}}}
