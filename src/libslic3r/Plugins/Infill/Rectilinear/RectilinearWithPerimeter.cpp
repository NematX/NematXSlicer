///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "RectilinearWithPerimeter.hpp"

#include <algorithm>
#include <vector>

#include "libslic3r/Plugins/Infill/PatternPluginHelpers.hpp"

/*
Rectilinear-with-perimeter implementation
==========================================

The legacy algorithm builds one root, one non-sortable node per outer offset
island, and separate perimeter and infill groups below that node. The same
hierarchy is built here with storage-owned API entities so later pipeline steps
can apply the usual ordering and volume correction plugins.
*/

namespace slic3r_api { namespace Infill { namespace RectilinearPatternsPlugin {
namespace rect_work = PluginCommon::polygon_work;
namespace recti {
namespace {

// Legacy FillWithPerimeter defaults: no perimeter overlap adjustment and a 50% inner offset.
constexpr double k_overlap_ratio = 0.;
constexpr double k_ratio_fill_inside = 0.;

// Append one offset island while preserving its fixed ordering boundary.
void append_expolygon(StoredExtrusionEntity &root,
                      storage_handle *storage,
                      const ExPolygon &expolygon,
                      const fills::FillRectilinear &infill,
                      const raw_infill_pattern_params &params,
                      bool split_outer_islands);
// Append the perimeter loop group for one offset island.
void append_perimeter_group(StoredExtrusionEntity &destination,
                            storage_handle *storage,
                            const ExPolygon &expolygon,
                            const raw_infill_pattern_params &params);
// Wrap one closed contour in the non-sortable loop node used by the legacy tree.
void append_perimeter_loop(StoredExtrusionEntity &destination,
                           storage_handle *storage,
                           const std::vector<c_point> &points,
                           const raw_infill_pattern_params &params);
// Append the rectilinear infill generated behind one perimeter.
void append_inner_group(StoredExtrusionEntity &destination,
                        storage_handle *storage,
                        const ExPolygon &expolygon,
                        const fills::FillRectilinear &infill,
                        const raw_infill_pattern_params &params);
// Return a closed contour with the orientation expected by the legacy extrusion path.
std::vector<c_point> oriented_closed_points(const Polygon &polygon, bool clockwise);

} // namespace

// Reproduce FillWithPerimeter's offset hierarchy with storage-owned API entities.
StoredExtrusionEntity RectiWithPerimeter::fill_with_perimeter(
    storage_handle *storage,
    const slic3r_api::ExPolygon &surface,
    const fills::FillRectilinear &infill,
    const raw_infill_pattern_params &params)
{
    StoredExtrusionEntity root(storage);
    root.set_flags(RAW_EXTRUSION_FLAG_SORTABLE | RAW_EXTRUSION_FLAG_REVERSIBLE);

    const double spacing = infill.get_spacing();
    const coord_t offset_for_overlap = scale_d(spacing / 2.) * ((1. - k_overlap_ratio) / 2.);
    ClipperOperand subject(storage, surface);
    ClipperOperand perimeter = clipper_offset2(
        subject,
        scale_d(-spacing) / 2. - double(offset_for_overlap),
        double(offset_for_overlap),
        CLIPPER_JOIN_MITER,
        scale_d(spacing) * 10.);
    ClipperOperand clipped_perimeter = clipper_intersection(
        perimeter,
        clipper_offset(subject, scale_d(-spacing / 2.)));
    StoredExPolygonCollection path_perimeter = clipped_perimeter.to_expolygon_collection();
    path_perimeter.ensure_valid();

    const bool split_outer_islands = path_perimeter.size() > 1;
    for (const ExPolygon expolygon : path_perimeter)
        append_expolygon(root, storage, expolygon, infill, params, split_outer_islands);
    if (path_perimeter.size() == 1)
        root.set_flags(0);

    return root;
}

namespace {

void append_expolygon(StoredExtrusionEntity &root,
                      storage_handle *storage,
                      const ExPolygon &expolygon,
                      const fills::FillRectilinear &infill,
                      const raw_infill_pattern_params &params,
                      bool split_outer_islands)
{
    StoredExtrusionEntity expolygon_group(storage);
    StoredExtrusionEntity &destination = split_outer_islands ? expolygon_group : root;

    append_perimeter_group(destination, storage, expolygon, params);
    append_inner_group(destination, storage, expolygon, infill, params);

    if (split_outer_islands && !expolygon_group.empty()) {
        // Child insertion restores default flags, so apply the fixed-island flags last.
        destination.disable_sort().disable_reverse();
        root.append_child_move(expolygon_group.mutable_view());
    }
}

void append_perimeter_group(StoredExtrusionEntity &destination,
                            storage_handle *storage,
                            const ExPolygon &expolygon,
                            const raw_infill_pattern_params &params)
{
    StoredExtrusionEntity perimeter(storage);
    perimeter.set_flags(RAW_EXTRUSION_FLAG_SORTABLE | RAW_EXTRUSION_FLAG_REVERSIBLE);

    const std::vector<c_point> contour = oriented_closed_points(expolygon.contour(), false);
    if (contour.size() >= 2)
        append_perimeter_loop(perimeter, storage, contour, params);

    for (const Polygon hole : expolygon.holes()) {
        const std::vector<c_point> hole_points = oriented_closed_points(hole, true);
        if (hole_points.size() >= 2)
            append_perimeter_loop(perimeter, storage, hole_points, params);
    }

    if (!perimeter.empty())
        destination.append_child_move(perimeter.mutable_view());
}

void append_perimeter_loop(StoredExtrusionEntity &destination,
                           storage_handle *storage,
                           const std::vector<c_point> &points,
                           const raw_infill_pattern_params &params)
{
    StoredExtrusionEntity path = PatternHelpers::make_path(storage, points, params);
    StoredExtrusionEntity loop(storage);
    loop.append_child_move(path.mutable_view());
    loop.set_flags(0);
    destination.append_child_move(loop.mutable_view());
}

void append_inner_group(StoredExtrusionEntity &destination,
                        storage_handle *storage,
                        const ExPolygon &expolygon,
                        const fills::FillRectilinear &infill,
                        const raw_infill_pattern_params &params)
{
    ClipperOperand subject(storage, expolygon);
    StoredExPolygonCollection inner = clipper_offset2(
        subject,
        scale_d(-infill.get_spacing() * (k_ratio_fill_inside + 0.5)),
        scale_d(infill.get_spacing() / 2.),
        CLIPPER_JOIN_MITER).to_expolygon_collection();
    inner.ensure_valid();

    StoredExtrusionEntity infill_group(storage);
    infill_group.set_flags(RAW_EXTRUSION_FLAG_SORTABLE | RAW_EXTRUSION_FLAG_REVERSIBLE);
    for (const ExPolygon inner_expolygon : inner) {
        const rect_work::Polylines lines = infill.fill_surface(inner_expolygon, params);
        for (const rect_work::Polyline &line : lines) {
            if (line.points.size() < 2)
                continue;
            StoredExtrusionEntity path = PatternHelpers::make_path(storage, line.points, params);
            infill_group.append_child_move(path.mutable_view());
        }
    }

    if (!infill_group.empty())
        destination.append_child_move(infill_group.mutable_view());
}

std::vector<c_point> oriented_closed_points(const Polygon &polygon, bool clockwise)
{
    std::vector<c_point> points = polygon.points();
    if (points.size() >= 3 && polygon.is_clockwise() != clockwise)
        std::reverse(points.begin(), points.end());
    if (!points.empty())
        points.push_back(points.front());
    return points;
}

} // namespace
} // namespace recti
}}} // namespace slic3r_api::Infill::RectilinearPatternsPlugin
