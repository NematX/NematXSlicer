///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ Copyright (c) Prusa Research 2020 - 2022
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#include "TravelBoundary.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "libslic3r/Plugins/Common/ClipperHelpers.hpp"

/*
Travel boundaries for the avoid-crossing router
==============================================
TravelRouter (TravelRouter.hpp) replaces perimeter-crossing travels with detours
along rings inside printed areas. This file builds internal rings from the copied
layer and external rings from the external input, both defined in AvoidCrossingInput.hpp;
it does no routing. Construction must reproduce
the core AvoidCrossingPerimeters exactly: tests compare rings, lengths and grid
cells. Float lengths, operation order and truncated coordinates preserve that equality.

The travel provider in AvoidCrossingPerimetersTravel.cpp keeps one router per layer
and tool. Each router owns one TravelBoundary and builds either boundary lazily
when a route first needs it:
- Internal: travels within one object instance use object coordinates. Rings lie
  up to 1.5 perimeter spacings inside each island so detours follow inner perimeters.
- External: travels between objects or instances use print coordinates. Only holes
  of all objects at that height are placed per instance. For support targets,
  the external input also includes holes of each object's last layer at or below
  the target height. Reversing their clockwise rings before expansion grows them
  into the material by half an external perimeter
  spacing; reversing them back gives rings around which travels are routed.

Internal construction prepares the rings in four stages:
1. Adaptive inset: move rings inward without closing narrow necks, preparing the
   islands for top removal and tool clipping. Where edges are long enough,
   resampling adds vertices offset/2 from corners and splits the intervening span
   at intervals of at most 0.5 mm so the inset can vary locally. The resampled island
   is kept unless integer truncation pushes part outside the original; intersection
   is not used because it drops collinear samples. At each vertex, contour_distance
   measures the nearest edge facing it across the island, capped by a search radius.
   The inset is zero below a width threshold, half the excess above it in between,
   and the full offset in wide areas. Thresholds offset/2, offset and 2*offset+epsilon
   are tried in order. The first single piece with unchanged hole count is accepted.
   After the last try, one piece with changed holes is accepted, otherwise the
   largest of several pieces, otherwise the pre-inset candidate (original or
   resampled) is kept. Islands whose bounding box, grown by epsilon, has area below
   (0.1 mm)^2 skip inset attempts and keep that candidate. Original island/inset pairs
   are saved before any top removal or tool clipping. Routing uses these pairs for
   containment and to find an inset edge when no ring lies within the edge grid's
   search reach.

2. Support supplement: add insets of the object layer at or below the target to
   supply rings for support layers, which have no perimeter slices. This matches
   the core rule inherited from PrusaSlicer in 2020.
   The purpose of that rule is undocumented; it does not route support travels
   around objects.
3. Top exclusion: on non-support layers, shrink selected top surfaces by half a
   spacing and remove them into to_avoid, preparing rings that do not cross those
   tops. Routing checks to_avoid before shortcuts when an endpoint cannot be placed
   on a boundary. If the region has perimeters and no ironing, retain a band from
   a quarter spacing outside to half a spacing inside the top's edge so a detour
   can follow its perimeter. With ironing the band is useless because ironing
   passes over the top's edge.
4. Tool clipping: on layers using several tools, restrict the remaining rings to
   the area printed by the current tool, preparing detours that do not ooze its
   material onto another tool's area.

Indexing runs after construction, so grids point at the final ring arrays; those
arrays, including the contours and holes of to_avoid, must not change afterwards.
Internal rings are listed contour then holes, with each ring mapped to its island's
contour index.
Per-ring boxes identify enclosing holes. A global box, padded by two perimeter
spacings plus epsilon, limits routing; a 1 mm edge grid finds crossings and nearest
points. Cumulative float lengths let routing choose the shorter way around a ring.
Selected-ring grids are created when an island jump first uses the ring: an island's
contour or the hole enclosing the other island, keyed by its index in boundaries.
Top exclusions supplied before indexing are preserved and indexed separately.
init_layer also insets perimeter slices by half an external perimeter width without
a prior union, as the core does. Routing only tests whether this result is empty
to leave travels direct when there is no interior; support bypasses that test.

    TravelBoundary
    |-- init_layer: reset caches and prepare the interior-presence test
    |   `-- offset_ex: inset concatenated perimeter slices without prior union
    |-- build_internal: construct one tool's object-space rings
    |   |-- get_boundary: form insets, use lower slices for support, exclude tops, clip by tool
    |   |   `-- inner_offset: preserve narrow connections while moving rings inward
    |   |       |-- resample_expolygon: add local width sampling points
    |   |       |   `-- resample_polygon: insert points without changing ring order
    |   |       |-- precompute_expolygon_distances: measure separation along each ring
    |   |       |-- contour_distance: find facing edges across the island
    |   |       `-- variable_offset_inner_ex: apply the width-dependent inset
    |   `-- init_boundary (islands): flatten contours and holes, retain island indices
    `-- build_external: construct print-space rings around holes
        |-- get_boundary_external: place instances and grow holes into material
        `-- init_boundary (rings): index rings without island membership
            `-- init_boundary_distances: cache lengths (also called by the islands overload)
                `-- precompute_polygon_distances: include the closing edge

Boundary owns rings, original/inset pairs and excluded tops until reset. Boundary
and therefore TravelBoundary cannot be copied or moved, and their owners keep stable
addresses. Clipper calls allocate temporary geometry results in the storage passed
by the caller. Construction retains no handle to print data and does not retain
the caller's storage pointer. Each worker,
the provider thread processing one layer group in AvoidCrossingPerimetersTravel.cpp,
owns its TravelBoundary. Workers may share allocation storage; the plugin host
synchronizes its allocation bookkeeping.
*/
namespace slic3r_api { namespace LayerExtrusionEdit { namespace AvoidCrossing {
namespace {
using namespace PluginCommon::polygon_work;
using namespace PluginCommon::work;
namespace pw = PluginCommon::polygon_work;
using Polygon = pw::Polygon;
using ExPolygon = pw::ExPolygon;
namespace clipper = PluginCommon::clipper;
namespace EdgeGrid = PluginCommon::EdgeGrid;

// Insert truncated points near corners and along long edges so an inset can change locally, preserving ring order.
void resample_polygon(Polygon &polygon, double dist_from_vertex, double max_allowed_distance);
// Resample contour and holes with the same near-vertex distance and maximum edge length.
void resample_expolygon(ExPolygon &polygon, double dist_from_vertex, double max_allowed_distance);
// Store cumulative float lengths, including the closing edge, for a nonempty ring.
void precompute_polygon_distances(const Polygon &polygon, std::vector<float> &distances);
// Store contour then hole length arrays for nonempty rings.
void precompute_expolygon_distances(const ExPolygon &polygon, std::vector<std::vector<float>> &distances);
// Return the signed double cross product, preserving the core multiplication order.
double cross2(const Vec2d &a, const Vec2d &b);
// Return each vertex's distance to the nearest facing edge across the island, capped at search_radius.
// Distances are returned only for rings with more than two vertices; two pass the assertion but return an empty result.
// An edge of the same ring counts only when far enough along it; polygon must have at least two vertices.
std::vector<float> contour_distance(const EdgeGrid::Grid &grid, const std::vector<float> &distances,
    size_t contour_idx, const Polygon &polygon, double compensation, double search_radius);
// Return original/inset pairs with width-dependent insets to preserve narrow connections; update branch counts.
// Prefer one piece with unchanged holes, then changed holes, then the largest piece, or retain the pre-inset island.
std::vector<std::pair<ExPolygon, ExPolygon>> inner_offset(storage_handle *storage, const ExPolygons &polygons, double offset, InnerOffsetCounts &counts);
// Build object-space detour islands for a zero-based tool, preserving narrow connections and avoiding selected tops.
// Append original/inset pairs and top exclusions for endpoint placement; accumulate inset branch counts.
ExPolygons get_boundary(storage_handle *storage, const Input &input, int extruder,
    std::vector<std::pair<ExPolygon, ExPolygon>> &growth, ExPolygons &to_avoid, InnerOffsetCounts &counts);
// Return print-space rings around all placed holes, grown into material by half an external perimeter spacing.
// Support input also includes holes of each object's last layer at or below the target height.
// Reverse clockwise holes for expansion and restore their orientation afterwards.
Polygons get_boundary_external(storage_handle *storage, const ExternalInput &input);
// Fill cumulative edge lengths for every indexed ring.
void init_boundary_distances(Boundary &boundary);
// Replace external rings and index the resulting rings for routing, preserving and indexing supplied top exclusions.
// Use scaled padding plus epsilon for the clipping box; no island membership or per-ring boxes are produced.
void init_boundary(storage_handle *storage, Boundary &boundary, Polygons &&polygons, coord_t extra_available_space);
// Flatten internal islands contour then holes, retaining membership and boxes to find enclosing holes.
// Index the stored rings with scaled padding plus epsilon, preserving and indexing supplied top exclusions.
void init_boundary(storage_handle *storage, Boundary &boundary, ExPolygons &&islands, coord_t extra_available_space);

ExPolygons get_boundary(storage_handle *storage, const Input &input, int extruder,
    std::vector<std::pair<ExPolygon, ExPolygon>> &growth, ExPolygons &to_avoid, InnerOffsetCounts &counts)
{
    const LayerInput &layer = *input.target;
    const coord_t spacing = layer.perimeter_spacing;
    ExPolygons perimeter = clipper::union_ex(storage, layer.perimeter_slices);
    ExPolygons boundary;
    std::vector<std::pair<ExPolygon, ExPolygon>> pairs = inner_offset(storage, perimeter, 1.5 * spacing, counts);
    for (const std::pair<ExPolygon, ExPolygon> &pair : pairs) {
#ifndef NDEBUG
        // An inset must not extend outside its island; retaining the island unchanged is also valid.
        PluginCommon::work::Polyline contour;
        contour.points = pair.second.contour.points;
        contour.points.push_back(contour.points.front());
        assert(clipper::contains(storage, pair.first, contour) ||
            (pair.first.contour.points == pair.second.contour.points && pair.first.holes.size() == pair.second.holes.size() &&
             std::equal(pair.first.holes.begin(), pair.first.holes.end(), pair.second.holes.begin(),
                 [](const Polygon &a, const Polygon &b) { return a.points == b.points; })) ||
            clipper::diff_ex(storage, pair.second, pair.first).empty());
#endif
        boundary.push_back(pair.second);
        growth.push_back(pair);
    }
    boundary = clipper::union_ex(storage, boundary);
    // Support has no perimeter slices; reproduce the core's use of the object layer at or below it.
    if (layer.support) {
        perimeter = clipper::union_ex(storage, input.lower_perimeter_slices);
        pairs = inner_offset(storage, perimeter, 1.5 * spacing, counts);
        for (std::pair<ExPolygon, ExPolygon> &pair : pairs) {
            boundary.push_back(pair.second);
            growth.push_back(std::move(pair));
        }
        boundary = clipper::union_ex(storage, boundary);
    }
    // Exclude selected tops; with perimeters and no ironing, keep a band from a quarter spacing outside to half a spacing inside.
    // It lets detours follow the perimeter; with ironing it is useless because ironing passes over the top's edge.
    ExPolygons tops;
    if (!layer.support) {
        for (const RegionInput &region : layer.regions) {
            if (!region.avoid_crossing_top) continue;
            for (const ExPolygon &surface : region.top_surfaces) {
                ExPolygons inset = clipper::offset_ex(storage, surface, double(-spacing / 2));
                if (region.perimeters > 0 && !region.ironing) {
                    ExPolygons rim = clipper::diff_ex(storage,
                        clipper::offset_ex(storage, surface, double(spacing / 4)), inset);
                    boundary.insert(boundary.end(), rim.begin(), rim.end());
                }
                tops.insert(tops.end(), inset.begin(), inset.end());
            }
        }
    }
    if (!tops.empty()) {
        tops = clipper::union_ex(storage, tops);
        boundary = clipper::union_ex(storage, boundary);
        to_avoid.insert(to_avoid.end(), tops.begin(), tops.end());
        boundary = clipper::diff_ex(storage, boundary, tops);
    }
    // Keep this tool over its own printed area so its material does not ooze over another tool's area.
    bool multiple = false;
    for (const RegionInput &region : layer.regions) {
        multiple = region.perimeter_extruder != extruder + 1 || region.infill_extruder != extruder + 1 ||
                   region.solid_infill_extruder != extruder + 1;
        if (multiple) break;
    }
    if (multiple) {
        ExPolygons clip;
        for (const RegionInput &region : layer.regions) {
            const bool same = region.perimeter_extruder == region.infill_extruder &&
                              region.infill_extruder == region.solid_infill_extruder;
            if (same) {
                if (region.perimeter_extruder == extruder + 1)
                    clip = clipper::union_ex(storage, clip, region.raw_slices);
            } else if (region.perimeter_extruder == extruder + 1) {
                assert(region.infill_extruder != extruder + 1);
                clip = clipper::union_ex(storage, clip, clipper::diff_ex(storage, region.raw_slices, region.fill_expolygons));
            } else {
                assert(region.infill_extruder == extruder + 1);
                clip = clipper::union_ex(storage, clip, region.fill_expolygons);
            }
        }
        boundary = clipper::intersection_ex(storage, boundary, clipper::offset_ex(storage, clip, SCALED_EPSILON * 10));
    }
    return boundary;
}

Polygons get_boundary_external(storage_handle *storage, const ExternalInput &input)
{
    Polygons boundary;
    for (const ExternalObjectInput &object : input.external_objects) {
        Polygons holes = object.holes;
        for (Polygon &hole : holes) hole.reverse();
        for (const Point &shift : object.instance_shifts) {
            const size_t begin = boundary.size();
            boundary.insert(boundary.end(), holes.begin(), holes.end());
            for (size_t i = begin; i < boundary.size(); ++i)
                for (Point &point : boundary[i].points) point += shift;
        }
    }
    boundary = clipper::expand(storage, boundary, double(input.external_perimeter_spacing / 2));
    for (Polygon &polygon : boundary) polygon.reverse();
    return boundary;
}

void init_boundary(storage_handle *storage, Boundary &boundary, Polygons &&polygons, coord_t extra_available_space)
{
    ExPolygons to_avoid = std::move(boundary.to_avoid);
    boundary.clear();
    boundary.boundaries = std::move(polygons);
    BoundingBox bbox = clipper::get_extents(boundary.boundaries);
    bbox.offset(SCALED_EPSILON + extra_available_space);
    boundary.bbox = BoundingBoxf(to_vec2d(bbox.min), to_vec2d(bbox.max));
    boundary.grid.set_bbox(bbox);
    boundary.grid.create(boundary.boundaries, scale_i(1.));
    init_boundary_distances(boundary);
    boundary.to_avoid = clipper::union_ex(storage, to_avoid);
    bbox = clipper::get_extents(boundary.to_avoid);
    bbox.offset(SCALED_EPSILON + extra_available_space);
    boundary.to_avoid_grid.set_bbox(bbox);
    // Borrow each owned ring directly; a flattened temporary would leave dangling points.
    PluginCommon::polygon_work::ContourViews contours;
    for (const ExPolygon &polygon : boundary.to_avoid) {
        contours.emplace_back(polygon.contour.points);
        for (const Polygon &hole : polygon.holes) contours.emplace_back(hole.points);
    }
    boundary.to_avoid_grid.create(contours, scale_i(1.));
}

void init_boundary(storage_handle *storage, Boundary &boundary, ExPolygons &&islands, coord_t extra_available_space)
{
    ExPolygons to_avoid = std::move(boundary.to_avoid);
    boundary.clear();
    for (ExPolygon &island : islands) {
        const size_t id = boundary.boundaries.size();
        boundary.bboxes.emplace_back(island.contour.points);
        boundary.boundaries.push_back(std::move(island.contour));
        boundary.islands.push_back(id);
        for (Polygon &hole : island.holes) {
            boundary.bboxes.emplace_back(hole.points);
            boundary.boundaries.push_back(std::move(hole));
            boundary.islands.push_back(id);
        }
    }
    BoundingBox bbox = clipper::get_extents(boundary.boundaries);
    bbox.offset(SCALED_EPSILON + extra_available_space);
    boundary.bbox = BoundingBoxf(to_vec2d(bbox.min), to_vec2d(bbox.max));
    boundary.grid.set_bbox(bbox);
    boundary.grid.create(boundary.boundaries, scale_i(1.));
    init_boundary_distances(boundary);
    boundary.to_avoid = clipper::union_ex(storage, to_avoid);
    bbox = clipper::get_extents(boundary.to_avoid);
    bbox.offset(SCALED_EPSILON + extra_available_space);
    boundary.to_avoid_grid.set_bbox(bbox);
    // Borrow each owned ring directly; a flattened temporary would leave dangling points.
    PluginCommon::polygon_work::ContourViews contours;
    for (const ExPolygon &polygon : boundary.to_avoid) {
        contours.emplace_back(polygon.contour.points);
        for (const Polygon &hole : polygon.holes) contours.emplace_back(hole.points);
    }
    boundary.to_avoid_grid.create(contours, scale_i(1.));
    assert(boundary.islands.size() == boundary.boundaries.size());
}

void init_boundary_distances(Boundary &boundary)
{
    boundary.boundaries_params.assign(boundary.boundaries.size(), std::vector<float>());
    for (size_t i = 0; i < boundary.boundaries.size(); ++i)
        precompute_polygon_distances(boundary.boundaries[i], boundary.boundaries_params[i]);
}

std::vector<std::pair<ExPolygon, ExPolygon>> inner_offset(storage_handle *storage, const ExPolygons &polygons, double offset, InnerOffsetCounts &counts)
{
    const std::vector<double> widths{offset / 2., offset, 2. * offset + SCALED_EPSILON};
    std::vector<std::pair<ExPolygon, ExPolygon>> result;
    for (const ExPolygon &polygon : polygons) {
        result.emplace_back(polygon, polygon);
        resample_expolygon(result.back().second, offset / 2, scale_d(0.5));
        ExPolygons difference = clipper::diff_ex(storage, result.back().second, result.back().first);
        // Keep the original if resampling extends outside it. Otherwise keep the resampled ring directly,
        // because clipping by intersection would remove the collinear samples needed for local widths.
        if (!difference.empty()) {
            ++counts.resample_original;
            result.back().second = result.back().first;
        } else {
            ++counts.resample_kept;
        }
    }
    for (std::pair<ExPolygon, ExPolygon> &pair : result) {
        ExPolygon &polygon = pair.second;
        BoundingBox bbox = clipper::get_extents(polygon);
        bbox.offset(SCALED_EPSILON);
        const Vec2d size = to_vec2d(bbox.size());
        // Skip insets when the epsilon-grown bounding box has area below (0.1 mm)^2; keep the original or resampled candidate.
        if (size.x() * size.y() < scale_d(0.1f) * scale_d(0.1f)) { ++counts.small; continue; }
        // Try thresholds offset/2, offset, then 2*offset+epsilon, increasingly protecting narrow parts.
        for (size_t round = 0; round < widths.size(); ++round) {
            const double width = widths[round];
            const double radius = 2. * (offset + width);
            EdgeGrid::Grid grid;
            grid.set_bbox(bbox);
            grid.create(polygon, coord_t(0.7 * radius));
            std::vector<std::vector<float>> lengths;
            precompute_expolygon_distances(polygon, lengths);
            std::vector<std::vector<float>> offsets;
            offsets.reserve(polygon.holes.size() + 1);
            for (size_t i = 0; i <= polygon.holes.size(); ++i) {
                const Polygon &ring = i == 0 ? polygon.contour : polygon.holes[i - 1];
                assert((ring.area() > 0.) == (i == 0));
                std::vector<float> distances = contour_distance(grid, lengths[i], i, ring, offset, radius);
                // Use no inset below the threshold, half the excess in between, and the full inset in wide areas.
                for (float &distance : distances) {
                    if (distance < width) distance = 0.f;
                    else if (distance > width + 2. * offset) distance = -float(offset);
                    else distance = -(distance - float(width)) / 2.f;
                }
                offsets.emplace_back(std::move(distances));
            }
            ExPolygons inset = clipper::variable_offset_inner_ex(storage, polygon, offsets);
            if (inset.size() == 1 && inset.front().holes.size() == polygon.holes.size()) {
                ++counts.same_holes;
                polygon = std::move(inset.front());
                break;
            } else if (round + 1 < widths.size()) { ++counts.retry; continue; }
            else if (inset.size() == 1) {
                ++counts.changed_holes;
                polygon = std::move(inset.front());
                break;
            } else if (inset.size() > 1) {
                ++counts.multiple;
                // Keep the largest piece after the last attempt, discarding small artifacts.
                double area = pw::core_area(inset.front());
                size_t largest = 0;
                for (size_t i = 1; i < inset.size(); ++i) {
                    const double next = pw::core_area(inset[i]);
                    if (area < next) { area = next; largest = i; }
                }
                polygon = std::move(inset[largest]);
                break;
            } else ++counts.empty_original;
        }
    }
    return result;
}

void resample_polygon(Polygon &polygon, double dist_from_vertex, double max_allowed_distance)
{
    Points points;
    points.reserve(3 * polygon.size());
    for (size_t i = 0; i < polygon.size(); ++i) {
        points.emplace_back(polygon[i]);
        const Point &p1 = polygon[i];
        const Point &p2 = polygon[(i + 1) % polygon.size()];
        const Vec2d line = to_vec2d(p2 - p1);
        const double length = line.norm();
        const Point offset = to_point(line.normalized() * dist_from_vertex);
        if (length > 2 * dist_from_vertex && offset != Point(0, 0)) {
            points.emplace_back(p1 + offset);
            const Vec2d middle = to_vec2d(p2 - p1 - 2 * offset);
            const double middle_length = middle.norm();
            if (middle_length > max_allowed_distance) {
                const Vec2d previous = to_vec2d(points.back());
                const size_t parts = size_t(ceil(middle_length / max_allowed_distance));
                for (size_t part = 1; part < parts; ++part)
                    points.emplace_back(to_point(previous + middle * (double(part) / double(parts))));
            }
            points.emplace_back(p2 - offset);
        }
    }
    polygon.points = std::move(points);
}

void resample_expolygon(ExPolygon &polygon, double dist_from_vertex, double max_allowed_distance)
{
    resample_polygon(polygon.contour, dist_from_vertex, max_allowed_distance);
    for (Polygon &hole : polygon.holes) resample_polygon(hole, dist_from_vertex, max_allowed_distance);
}

void precompute_polygon_distances(const Polygon &polygon, std::vector<float> &distances)
{
    distances.assign(polygon.size() + 1, 0.f);
    for (size_t i = 1; i < polygon.size(); ++i)
        distances[i] = distances[i - 1] + float(to_vec2d(polygon[i] - polygon[i - 1]).norm());
    distances.back() = distances[polygon.size() - 1] + float(to_vec2d(polygon.points.back() - polygon.points.front()).norm());
}

void precompute_expolygon_distances(const ExPolygon &polygon, std::vector<std::vector<float>> &distances)
{
    distances.assign(polygon.holes.size() + 1, std::vector<float>());
    precompute_polygon_distances(polygon.contour, distances.front());
    for (size_t i = 0; i < polygon.holes.size(); ++i)
        precompute_polygon_distances(polygon.holes[i], distances[i + 1]);
}

double cross2(const Vec2d &a, const Vec2d &b) { return a.x() * b.y() - a.y() * b.x(); }

std::vector<float> contour_distance(const EdgeGrid::Grid &grid, const std::vector<float> &distances,
    size_t contour_idx, const Polygon &polygon, double compensation, double search_radius)
{
    assert(!polygon.empty() && polygon.size() >= 2);
    std::vector<float> out;
    if (polygon.size() > 2) {
        struct Visitor {
            const EdgeGrid::Grid &grid;
            size_t contour_idx;
            const std::vector<float> &parameters;
            double accept_distance, reject_distance;
            size_t point_idx = 0;
            Point point;
            Vec2d inward;
            bool found = false;
            double distance = std::numeric_limits<double>::max();

            // Initialize an inward bisector at a vertex of the borrowed contour.
            void init(const Points &points, const Point &vertex) {
                point_idx = &vertex - points.data();
                point = vertex;
                found = false;
                const Vec2d v1 = to_vec2d(points[point_idx] - points[(point_idx + points.size() - 1) % points.size()]);
                const Vec2d v2 = to_vec2d(points[(point_idx + 1) % points.size()] - points[point_idx]);
                inward = Vec2d(-v1.y() - v2.y(), v1.x() + v2.x());
                distance = std::numeric_limits<double>::max();
            }
            // Accept the left side of both convex edges or either concave edge.
            static bool inside_corner(const EdgeGrid::Contour &contour, size_t i, const Point &opposite) {
                const Vec2d pt = to_vec2d(opposite);
                const Vec2d previous = to_vec2d(contour.segment_prev(i));
                const Vec2d current = to_vec2d(contour.segment_start(i));
                const Vec2d next = to_vec2d(contour.segment_end(i));
                const Vec2d v1 = current - previous, v2 = next - current;
                const bool left1 = cross2(v1, pt - previous) > 0.;
                const bool left2 = cross2(v2, pt - current) > 0.;
                return cross2(v1, v2) > 0 ? left1 && left2 : left1 || left2;
            }
            // Test the left side of the directed segment.
            static bool left_of_segment(const EdgeGrid::Contour &contour, size_t i, const Point &opposite) {
                const Vec2d current = to_vec2d(contour.segment_start(i));
                return cross2(to_vec2d(contour.segment_end(i)) - current, to_vec2d(opposite) - current) > 0.;
            }
            // Inspect cell edges, update the nearest accepted inward edge, and continue traversal.
            bool operator()(coord_t row, coord_t col) {
                const auto range = grid.cell_data_range(row, col);
                for (auto it = range.first; it != range.second; ++it) {
                    const std::pair<c_point, c_point> segment = grid.segment(*it);
                    const Vec2d v = to_vec2d(Point(segment.second) - Point(segment.first));
                    const Vec2d va = to_vec2d(point - Point(segment.first));
                    const double l2 = v.squaredNorm();
                    const double t = l2 == 0. ? 0. : std::clamp(va.dot(v) / l2, 0., 1.);
                    const Vec2d foot = to_vec2d(segment.first) + t * v;
                    const Vec2d bisector = foot - to_vec2d(point);
                    const double dist = bisector.norm();
                    if ((!found || dist < distance) && inward.dot(bisector) > 0.) {
                        bool accept = true;
                        if (it->first == contour_idx) {
                            const EdgeGrid::Contour &contour = grid.contours()[it->first];
                            double lo = parameters[point_idx];
                            double hi = t * sqrt(l2);
                            const double end = parameters.back();
                            const size_t i = it->second;
                            if (i + 1 < contour.size()) hi += parameters[i];
                            if (lo > hi) std::swap(lo, hi);
                            assert(lo > -SCALED_EPSILON && lo <= end + SCALED_EPSILON);
                            assert(hi > -SCALED_EPSILON && hi <= end + SCALED_EPSILON);
                            const double along = std::min(hi - lo, lo + end - hi);
                            if (along < accept_distance) accept = false;
                            else if (dist < reject_distance + SCALED_EPSILON) {
                                const bool inside = t == 0. ? inside_corner(contour, i, point) :
                                    t == 1. ? inside_corner(contour, contour.segment_idx_next(i), point) :
                                    left_of_segment(contour, i, point);
                                accept = inside && along > 0.6 * M_PI * dist;
                            }
                        }
                        if (accept && (!found || dist < distance)) { distance = dist; found = true; }
                    }
                }
                return true;
            }
        } visitor{grid, contour_idx, distances, 0.5 * compensation * M_PI, search_radius};
        out.reserve(polygon.size());
        const Point radius(coord_t(std::round(search_radius)), coord_t(std::round(search_radius)));
        for (const Point &point : polygon.points) {
            visitor.init(polygon.points, point);
            grid.visit_cells_intersecting_box(BoundingBox(point - radius, point + radius), visitor);
            out.emplace_back(float(visitor.found ? std::min(visitor.distance, search_radius) : search_radius));
        }
    }
    return out;
}
}

void Boundary::clear()
{
    grid = PluginCommon::EdgeGrid::Grid();
    to_avoid_grid = PluginCommon::EdgeGrid::Grid();
    island_to_grid.clear();
    boundaries.clear();
    islands.clear();
    bbox = PluginCommon::polygon_work::BoundingBoxf();
    bboxes.clear();
    boundaries_params.clear();
    boundary_growth.clear();
    to_avoid.clear();
}

void TravelBoundary::init_layer(storage_handle *storage, const LayerInput &layer)
{
    m_counts = InnerOffsetCounts();
    m_internal.clear();
    m_external.clear();
    m_lslices_offset = PluginCommon::clipper::offset_ex(storage, layer.perimeter_slices, -layer.external_perimeter_width / 2);
}

void TravelBoundary::build_internal(storage_handle *storage, const Input &input, int extruder)
{
    m_counts = InnerOffsetCounts();
    std::vector<std::pair<PluginCommon::polygon_work::ExPolygon, PluginCommon::polygon_work::ExPolygon>> growth;
    PluginCommon::polygon_work::ExPolygons islands = get_boundary(storage, input, extruder, growth, m_internal.to_avoid, m_counts);
    init_boundary(storage, m_internal, std::move(islands), input.target->perimeter_spacing * 2);
    m_internal.boundary_growth = std::move(growth);
}

void TravelBoundary::build_external(storage_handle *storage, const LayerInput &layer, const ExternalInput &external)
{
    init_boundary(storage, m_external, get_boundary_external(storage, external), layer.perimeter_spacing * 2);
}

}}}
