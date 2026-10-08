///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "PerimeterFill.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <vector>

#include "ClipperResults.hpp"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"

/* Implementation of the perimeter-wrapped fill (see PerimeterFill.hpp).

   Each step mirrors one statement of the native
   FillWithPerimeter::fill_surface_extrusion() (Fill/FillBase.cpp), with the
   same offsets, joins and miter limits, so the Clipper results are the same
   points. Native collections become API entities with the same flags:
   ExtrusionEntityCollection::set_can_sort_reverse(a, b) becomes the SORTABLE
   and REVERSIBLE flags, and a closed polyline, which the native
   extrusion_entities_append_paths() wraps in an ExtrusionLoop, becomes a fixed
   node holding the closed path. Flags are set after the children are added:
   adding the first child to an empty entity makes it sortable. */

namespace slic3r_api::Support::Common {
namespace {

// Append one path to parent like extrusion_entities_append_paths(): fewer than 2 points are
// skipped, and a closed polyline is wrapped in a fixed loop node.
void append_path(storage_handle *storage, MutableExtrusionEntity parent, const std::vector<c_point> &points,
                 const PerimeterFillSettings &settings);
// Points of a polygon starting at its first point and closed by repeating it (Polygon::split_at_index(0)),
// oriented counter-clockwise for a contour and clockwise for a hole.
std::vector<c_point> closed_loop_points(const Polygon &polygon, bool counter_clockwise);
// Collect, in printing order, the polylines of every leaf of an extrusion tree.
void collect_leaf_polylines(const ExtrusionEntity &entity, std::vector<std::vector<c_point>> &out);

// Fill one perimeter area: its loops, then the inner pattern on its inset.
void fill_area(storage_handle *storage, const ExPolygon &area, raw_surface_type surface_type,
               const PerimeterFillSettings &settings, const InnerFill &inner_fill, MutableExtrusionEntity area_entity);

void append_path(storage_handle *storage, MutableExtrusionEntity parent, const std::vector<c_point> &points,
                 const PerimeterFillSettings &settings)
{
    if (points.size() < 2)
        return;
    StoredExtrusionEntity path(storage, points);
    path.set_flags(settings.monotonic ? 0u : uint32_t(RAW_EXTRUSION_FLAG_REVERSIBLE));
    path.get_or_add(EPropertyAttributes::key)
        .extrusion_role(int32_t(settings.role))
        .mm3_per_mm(settings.flow.mm3_per_mm)
        .width(float(unscaled(settings.flow.width)))
        .height(float(unscaled(settings.flow.height)));
    const bool closed = points.front().x == points.back().x && points.front().y == points.back().y;
    if (!closed) {
        parent.append_child_move(path.mutable_view());
        return;
    }
    // ExtrusionLoop: neither sortable nor reversible, holding the path.
    StoredExtrusionEntity loop(storage);
    loop.append_child_move(path.mutable_view());
    loop.set_flags(0u);
    parent.append_child_move(loop.mutable_view());
}

std::vector<c_point> closed_loop_points(const Polygon &polygon, bool counter_clockwise)
{
    std::vector<c_point> points = polygon.points();
    if (points.empty())
        return points;
    if (polygon.is_counter_clockwise() != counter_clockwise)
        std::reverse(points.begin(), points.end());
    points.push_back(points.front());
    return points;
}

void collect_leaf_polylines(const ExtrusionEntity &entity, std::vector<std::vector<c_point>> &out)
{
    if (entity.has_polyline())
        out.push_back(entity.points());
    for (uint32_t idx = 0; idx < entity.child_count(); ++idx)
        collect_leaf_polylines(entity.child(idx), out);
}

void fill_area(storage_handle *storage, const ExPolygon &area, raw_surface_type surface_type,
               const PerimeterFillSettings &settings, const InnerFill &inner_fill, MutableExtrusionEntity area_entity)
{
    // Perimeter loops, contour first, sortable and reversible as a group.
    StoredExtrusionEntity perimeters(storage);
    append_path(storage, perimeters.mutable_view(), closed_loop_points(area.contour(), true), settings);
    for (const Polygon hole : area.holes())
        append_path(storage, perimeters.mutable_view(), closed_loop_points(hole, false), settings);
    perimeters.set_flags(RAW_EXTRUSION_FLAG_SORTABLE | RAW_EXTRUSION_FLAG_REVERSIBLE);
    area_entity.append_child_move(perimeters.mutable_view());

    // Inner area: inset by (ratio + 0.5) spacing, grown back by half a spacing so the
    // pattern overlaps the perimeter by half a line. The float ratio is widened like the native one.
    const ClipperContext clipper(storage);
    const StoredExPolygonCollection inner = clipper_offset2(clipper(area),
        scale_d(-settings.spacing * (settings.ratio_fill_inside + 0.5)), scale_d(settings.spacing / 2),
        CLIPPER_JOIN_MITER, 3.0).to_expolygon_collection();
    for (uint32_t idx = 0; idx < inner.size(); ++idx) {
        // The pattern reads a Surface: copy the inner area into a temporary one of the same type.
        StoredSurfaceCollection inner_surface(storage);
        inner_surface.append(inner.at(idx), surface_type);
        StoredExtrusionEntity pattern_output(storage);
        inner_fill(surface_collection_at(inner_surface.handle(), 0), pattern_output.mutable_view());
        std::vector<std::vector<c_point>> polylines;
        collect_leaf_polylines(pattern_output.readonly(), polylines);
        if (polylines.empty())
            continue;
        StoredExtrusionEntity fill(storage);
        for (const std::vector<c_point> &polyline : polylines)
            append_path(storage, fill.mutable_view(), polyline, settings);
        fill.set_flags(RAW_EXTRUSION_FLAG_SORTABLE | RAW_EXTRUSION_FLAG_REVERSIBLE);
        area_entity.append_child_move(fill.mutable_view());
    }
}

} // namespace

void fill_with_perimeter(storage_handle *storage, const Surface &surface, const PerimeterFillSettings &settings,
                         const InnerFill &inner_fill, MutableExtrusionEntity out)
{
    const ClipperContext clipper(storage);
    const ExPolygon expolygon = surface.expolygon();
    // Perimeter centerline: shrink by half a spacing with a miter join; the offset2 pair keeps
    // the overlap with the border, then the intersection removes islands a large miter merged.
    const double half_spacing = scale_d(settings.spacing / 2);
    const coord_t offset_for_overlap = coord_t(half_spacing * ((1 - settings.overlap_ratio) / 2));
    // Both operands are materialized as ExPolygons first, like the native ExPolygons arguments of
    // intersection_ex(). Like the native intersection_ex(), the rings too small or too narrow are
    // dropped and the others are united in a second pass: that pass decides the order of the areas
    // and the start point of each ring, which becomes the start of its perimeter loop.
    const StoredExPolygonCollection shrunk = clipper_offset2(clipper(expolygon),
        scale_d(-settings.spacing) / 2 - offset_for_overlap, double(offset_for_overlap),
        CLIPPER_JOIN_MITER, scale_d(settings.spacing) * 10).to_expolygon_collection();
    const StoredExPolygonCollection limit =
        clipper_offset(clipper(expolygon), scale_d(-settings.spacing / 2), CLIPPER_JOIN_MITER, 3.0).to_expolygon_collection();
    const StoredExPolygonCollection areas =
        native_expolygons(clipper_intersection(clipper(shrunk.readonly()), clipper(limit.readonly())));
    if (areas.empty())
        return;

    // One area fills the root itself; several areas each get a fixed node under a sortable root.
    StoredExtrusionEntity root(storage);
    for (uint32_t idx = 0; idx < areas.size(); ++idx) {
        if (areas.size() == 1) {
            fill_area(storage, areas.at(idx), surface.type(), settings, inner_fill, root.mutable_view());
            continue;
        }
        StoredExtrusionEntity area_entity(storage);
        fill_area(storage, areas.at(idx), surface.type(), settings, inner_fill, area_entity.mutable_view());
        area_entity.set_flags(0u);
        root.append_child_move(area_entity.mutable_view());
    }
    root.set_flags(areas.size() == 1 ? 0u : uint32_t(RAW_EXTRUSION_FLAG_SORTABLE | RAW_EXTRUSION_FLAG_REVERSIBLE));
    if (!root.empty())
        out.append_child_move(root.mutable_view());
}

InnerFill pattern_service_fill(const run_ctx_support_toolpath &payload, infill_pattern_runtime_id pattern_id,
                               const raw_infill_pattern_params &params, const layer_handle *layer,
                               const layer_island_handle *island, const layer_region_island_handle *region_island,
                               const layer_region_handle *primary_region)
{
    return [&payload, pattern_id, &params, layer, island, region_island, primary_region](const surface_handle *surface,
                                                                                         MutableExtrusionEntity output) {
        if (payload.generate_pattern == nullptr ||
            payload.generate_pattern(&payload, pattern_id, layer, island, region_island, primary_region, surface, nullptr,
                                     &params, output.mutable_handle()) == 0)
            throw std::runtime_error("The support pattern service could not fill a perimeter-wrapped area.");
    };
}

} // namespace slic3r_api::Support::Common
