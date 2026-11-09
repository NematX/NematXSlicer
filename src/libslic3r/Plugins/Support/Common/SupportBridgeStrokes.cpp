///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "SupportBridgeStrokes.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/cpp/Views.hpp"

/*
Bridge strokes of printed extrusions: implementation (the tutorial and the conventions are in SupportBridgeStrokes.hpp).

append_segment_strokes() works on one straight segment: it finds the places where the segment crosses the outline of
the grown lower support, which cut it into pieces that are over the lower support or over the void (the middle of a
piece tells which). Each piece over the void is extended at both ends by the line width, and is a bridge only when both
extended ends are over the lower support; its stroke is the piece grown by half the line width with square ends.
append_polyline_strokes() first merges the consecutive collinear segments of a leaf, so that a line split into several
segments is tested whole.

append_entity_strokes() resolves widths through the borrowed extrusion tree and
records the widest stroke contributed by an anchored straight segment. Both demand
removers use remove_leftovers_along_strokes(): subtract an opening from remaining
demand, then remove a connected thin piece only when it touches the strokes and
stays wholly inside their one-line-width neighbourhood. Other thin demand stays.

    append_entity_strokes()
        `-- append_tree_strokes(): inherit attributes and select each leaf's flow
            `-- append_segment_strokes(): split at support and require two anchors
    append_polyline_strokes(): merge only straight, ordered collinear segments
        `-- append_segment_strokes()
    remove_leftovers_along_strokes(): opening -> contact/reach checks -> subtraction
*/

namespace slic3r_api::Support::Common {

namespace {

// Resolve borrowed attributes along each branch and return the widest stroke actually contributed by its segments.
double append_tree_strokes(storage_handle *storage,
                           const ExPolygonCollection &lower_support,
                           const BridgeLineFlow &flow,
                           const ClipperContext &clipper,
                           const ExtrusionEntity &entity,
                           const EPropertyAttributes *attributes,
                           ClipperOperand &strokes);

// Whether the point lies in one of the polygons (contour and holes taken into account by ExPolygon::contains).
bool point_supported_by_expolygons(const ExPolygonCollection &support_polygons, c_point point)
{
    for (ExPolygon slice : support_polygons)
        if (slice.contains(point))
            return true;
    return false;
}

// The position of a point of the segment along it, 0 at segment_a and 1 at segment_b, measured on the longer axis.
double segment_parameter(c_point point, c_point segment_a, c_point segment_b)
{
    const coord_t dx = segment_b.x - segment_a.x;
    const coord_t dy = segment_b.y - segment_a.y;
    if (std::abs(dx) >= std::abs(dy))
        return dx == 0 ? 0. : double(point.x - segment_a.x) / double(dx);
    return dy == 0 ? 0. : double(point.y - segment_a.y) / double(dy);
}

// The point of the segment at a parameter (0 at segment_a, 1 at segment_b), rounded to the coordinate grid.
c_point segment_point_at(c_point segment_a, c_point segment_b, double parameter)
{
    c_point out = {};
    out.x = coord_t(std::llround(double(segment_a.x) + double(segment_b.x - segment_a.x) * parameter));
    out.y = coord_t(std::llround(double(segment_a.y) + double(segment_b.y - segment_a.y) * parameter));
    return out;
}

// Append to `parameters` the parameters, within [0, 1], where the segment crosses the polygon outline.
void add_polygon_line_intersections(const Polygon &polygon,
                                    c_point segment_a,
                                    c_point segment_b,
                                    std::vector<double> &parameters)
{
    const multipoint_const_view polygon_view = polygon.view();
    const uint32_t intersection_count = points_intersections(&polygon_view, segment_a, segment_b, nullptr);
    if (intersection_count == 0)
        return;

    std::vector<c_point> intersections(intersection_count);
    multipoint_view out_intersections = {};
    out_intersections.array = intersections.data();
    out_intersections.size = intersection_count;
    points_intersections(&polygon_view, segment_a, segment_b, &out_intersections);

    for (c_point point : intersections) {
        const double parameter = segment_parameter(point, segment_a, segment_b);
        if (parameter >= -1e-9 && parameter <= 1. + 1e-9)
            parameters.push_back(std::clamp(parameter, 0., 1.));
    }
}

// The sorted, deduplicated parameters of the segment: its two ends and its crossings with every contour and hole of the lower support.
std::vector<double> lower_support_intersection_parameters(const ExPolygonCollection &lower_support,
                                                          c_point segment_a,
                                                          c_point segment_b)
{
    std::vector<double> parameters;
    parameters.reserve(8);
    parameters.push_back(0.);
    parameters.push_back(1.);

    for (ExPolygon support_area : lower_support) {
        add_polygon_line_intersections(support_area.contour(), segment_a, segment_b, parameters);
        for (Polygon hole : support_area.holes())
            add_polygon_line_intersections(hole, segment_a, segment_b, parameters);
    }

    std::sort(parameters.begin(), parameters.end());
    parameters.erase(std::unique(parameters.begin(), parameters.end(),
                                 [](double lhs, double rhs) { return std::abs(lhs - rhs) < 1e-9; }),
                     parameters.end());
    return parameters;
}

} // namespace

BridgeLineFlow bridge_line_flow(const LayerIsland &island, raw_extrusion_role role, bool bridging)
{
    BridgeLineFlow out;
    bool found = false;
    for (uint32_t region_idx = 0; region_idx < island.region_count(); ++region_idx) {
        const LayerRegion region = island.region(region_idx);
        const c_flow flow = bridging ? region.bridging_flow(role) : region.flow(role);
        if (flow.width <= 0)
            continue;

        out.endpoint_extension = found ? std::min(out.endpoint_extension, flow.width) : flow.width;
        const coord_t effective_spacing = flow.spacing > 0 ? flow.spacing : flow.width;
        const double half_width = 0.5 * double(std::max(flow.width, effective_spacing)) + scale_d(0.001);
        out.half_width = found ? std::max(out.half_width, half_width) : half_width;
        const double support_probe_offset = 0.5 * double(flow.nozzle_diameter > 0 ? flow.nozzle_diameter : flow.width);
        out.support_probe_offset = found ? std::max(out.support_probe_offset, support_probe_offset) : support_probe_offset;
        found = true;
    }
    return out;
}

BridgeLineFlow bridge_line_flow_of_width(const BridgeLineFlow &base, coord_t line_width)
{
    BridgeLineFlow out = base;
    out.endpoint_extension = line_width;
    out.half_width = 0.5 * double(line_width) + scale_d(0.001);
    return out;
}

StoredExPolygonCollection grown_lower_support(const ClipperContext &clipper, const Layer &lower_layer, const BridgeLineFlow &flow)
{
    ClipperOperand lower_support_operand = clipper_offset(clipper(lower_layer.slices()),
                                                          flow.support_probe_offset,
                                                          CLIPPER_JOIN_SQUARE,
                                                          0.);
    return lower_support_operand.to_expolygon_collection();
}

bool append_segment_strokes(storage_handle *storage,
                            const ExPolygonCollection &lower_support,
                            const BridgeLineFlow &flow,
                            const ClipperContext &clipper,
                            const c_extrusion_segment &segment,
                            ClipperOperand &strokes)
{
    if (segment.radius != 0.f || c_point_distance_to(segment.point_a, segment.point_b) <= SCALED_EPSILON)
        return false;

    // The old support code first subtracts the lower layer from perimeter
    // polylines, then checks whether each remaining straight unsupported piece
    // is anchored at both ends. Rebuild that split locally: every interval
    // between lower-support intersections is sampled, and only unsupported
    // intervals whose extended endpoints land on support are considered real
    // bridges.
    const std::vector<double> parameters =
        lower_support_intersection_parameters(lower_support, segment.point_a, segment.point_b);
    bool added = false;
    for (size_t idx = 1; idx < parameters.size(); ++idx) {
        const double begin = parameters[idx - 1];
        const double end = parameters[idx];
        if (end - begin <= 1e-9)
            continue;

        const c_point begin_point = segment_point_at(segment.point_a, segment.point_b, begin);
        const c_point end_point = segment_point_at(segment.point_a, segment.point_b, end);
        if (c_point_distance_to(begin_point, end_point) <= SCALED_EPSILON)
            continue;

        const c_point middle = segment_point_at(segment.point_a, segment.point_b, 0.5 * (begin + end));
        const bool middle_supported = point_supported_by_expolygons(lower_support, middle);
        if (middle_supported)
            continue;

        StoredPolyline line(storage);
        line.push_back(begin_point);
        line.push_back(end_point);
        line.extend_start(flow.endpoint_extension);
        line.extend_end(flow.endpoint_extension);
        const bool front_supported = point_supported_by_expolygons(lower_support, line.front());
        const bool back_supported = point_supported_by_expolygons(lower_support, line.back());
        if (!front_supported || !back_supported)
            continue;

        ClipperOperand bridge_area = clipper_offset(clipper(line), flow.half_width, CLIPPER_JOIN_SQUARE, 0.,
                                                    CLIPPER_END_OPEN_SQUARE);
        if (!bridge_area.empty()) {
            strokes.concat_replace(bridge_area);
            added = true;
        }
    }
    return added;
}

double append_entity_strokes(storage_handle *storage,
                           const ExPolygonCollection &lower_support,
                           const BridgeLineFlow &flow,
                           const ClipperContext &clipper,
                           const ExtrusionEntity &entity,
                           ClipperOperand &strokes)
{
    return append_tree_strokes(storage, lower_support, flow, clipper, entity, nullptr, strokes);
}

namespace {

double append_tree_strokes(storage_handle *storage,
                           const ExPolygonCollection &lower_support,
                           const BridgeLineFlow &flow,
                           const ClipperContext &clipper,
                           const ExtrusionEntity &entity,
                           const EPropertyAttributes *attributes,
                           ClipperOperand &strokes)
{
    if (const EPropertyAttributes *local = entity.get(EPropertyAttributes::key))
        attributes = local;
    BridgeLineFlow line_flow = flow;
    if (attributes != nullptr && attributes->c_extrusion_property_attributes::width > 0.f)
        line_flow = bridge_line_flow_of_width(flow, coord_t(std::llround(scale_d(attributes->c_extrusion_property_attributes::width))));

    double widest = 0.;
    for (uint32_t segment_idx = 0; segment_idx < entity.segment_count(); ++segment_idx)
        if (append_segment_strokes(storage, lower_support, line_flow, clipper, entity.segment(segment_idx), strokes))
            widest = std::max(widest, 2. * line_flow.half_width);

    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        widest = std::max(widest, append_tree_strokes(storage, lower_support, flow, clipper, entity.child(child_idx), attributes, strokes));
    return widest;
}

} // namespace

void append_polyline_strokes(storage_handle *storage,
                             const ExPolygonCollection &lower_support,
                             const BridgeLineFlow &flow,
                             const ClipperContext &clipper,
                             const ExtrusionEntity &leaf,
                             ClipperOperand &strokes)
{
    // `current` is the straight segment being built: it grows while the next segment continues it on the same line. `chain` holds its
    // vertices (start, absorbed joints, end): a segment is absorbed only when every vertex stays within SCALED_EPSILON of the new chord and
    // the vertices keep their order along it, so the deviation is bounded for the whole chain and a back-and-forth never merges.
    bool has_current = false;
    c_extrusion_segment current = {};
    std::vector<c_point> chain;
    for (uint32_t segment_idx = 0; segment_idx < leaf.segment_count(); ++segment_idx) {
        const c_extrusion_segment next = leaf.segment(segment_idx);
        if (has_current && current.radius == 0.f && next.radius == 0.f && current.point_b.x == next.point_a.x &&
            current.point_b.y == next.point_a.y) {
            const c_point start = current.point_a;
            const double chord_x = double(next.point_b.x) - double(start.x);
            const double chord_y = double(next.point_b.y) - double(start.y);
            const double chord_length = std::hypot(chord_x, chord_y);
            bool absorbed = chord_length > 0.;
            double previous_along = 0.;
            for (size_t vertex_idx = 1; absorbed && vertex_idx < chain.size(); ++vertex_idx) {
                const double vertex_x = double(chain[vertex_idx].x) - double(start.x);
                const double vertex_y = double(chain[vertex_idx].y) - double(start.y);
                const double deviation = std::abs(vertex_x * chord_y - vertex_y * chord_x) / chord_length;
                const double along = (vertex_x * chord_x + vertex_y * chord_y) / chord_length;
                absorbed = deviation <= double(SCALED_EPSILON) && along > previous_along && along < chord_length;
                previous_along = along;
            }
            if (absorbed) {
                current.point_b = next.point_b;
                chain.push_back(next.point_b);
                continue;
            }
        }
        if (has_current)
            append_segment_strokes(storage, lower_support, flow, clipper, current, strokes);
        current = next;
        chain = { next.point_a, next.point_b };
        has_current = true;
    }
    if (has_current)
        append_segment_strokes(storage, lower_support, flow, clipper, current, strokes);
}

void remove_leftovers_along_strokes(const ClipperContext &clipper,
                                    const ClipperOperand &strokes,
                                    double line_width,
                                    ClipperOperand &remaining)
{
    // The residue of demand - strokes touches the strokes: a few SCALED_EPSILON absorb the rounding.
    const double tolerance = 4. * double(SCALED_EPSILON);
    // A classic opening: mitered joins keep the convex corners of the remaining demand.
    const ClipperOperand opened = clipper_offset2(remaining, -0.5 * line_width, 0.5 * line_width, CLIPPER_JOIN_MITER, 3.);
    const ClipperOperand thin_parts = clipper_diff(remaining, opened);
    if (thin_parts.empty())
        return;

    const ClipperOperand near_strokes = clipper_offset(strokes, line_width + tolerance, CLIPPER_JOIN_SQUARE, 0.);
    ClipperOperand removable = clipper.empty();
    const StoredExPolygonCollection pieces = thin_parts.to_expolygon_collection();
    for (ExPolygon piece : pieces) {
        const ClipperOperand shape = clipper(piece);
        const ClipperOperand reach = clipper_offset(shape, tolerance, CLIPPER_JOIN_SQUARE, 0.);
        if (clipper_intersection(reach, strokes).empty())
            continue;
        if (!clipper_diff(shape, near_strokes).empty())
            continue;
        removable.concat_replace(shape);
    }
    if (!removable.empty())
        remaining = clipper_diff(remaining, removable);
}

} // namespace slic3r_api::Support::Common
