///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ Copyright (c) Prusa Research 2023 Vojtěch Bubník @bubnikv, Pavel Mikuš @Godrak, Lukáš Hejl @hejllukas
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

// Keep the steps synchronized with FillEnsuring.cpp, AABBTreeLines.hpp and Algorithm/PathSorting.hpp.
#include "EnsuringPattern.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <optional>
#include <set>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/KDTreeIndirect.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Plugins/Infill/ArachneInfill.hpp"
#include "libslic3r/Plugins/Infill/PatternPluginHelpers.hpp"

/*
Ensuring pattern plugin
=======================

`ensuring` ports FillEnsuring, the solid infill that keeps narrow areas filled:

1. the surface is rotated so the fill direction becomes vertical;
2. the area wide enough for lines (opening by two then three spacings) is
   shrunk by half a spacing, minus the overlap;
3. vertical lines one spacing apart are cut by that area into sections; small
   clusters of short sections, which would make the nozzle vibrate, are removed;
4. the area covered by the sections is traced back; Arachne fills every gap
   left in the surface with variable-width walls, and gap walls whose ends
   nearly meet in line are joined;
5. the lines are ordered, touching lines first, then consecutive lines whose
   ends are close are joined. After rotation and rounding, adjacent identical
   point/width pairs are removed; distinct widths at one point remain transitions.

Each line becomes one extrusion with its variable widths, in one sortable
group, like the legacy Fill::fill_surface_extrusion() Arachne branch. The
boundary queries of AABBTreeLines and the path sorting of PathSorting are
ported here as linear scans with the same arithmetic, and the Clipper steps go
through the API.

The normal call flow is:

    EnsuringPattern::run_impl()
    `-- fill_ensuring_surface()
        |-- infill_direction(), rotated_surface()
        |-- inner_area()
        |-- vertical_sections()
        |-- stop_vibrations(), sort_sections(), section_lines()
        |-- fill_gaps()
        |   |-- reconstructed_area()
        |   |-- append_gap_walls()
        |   `-- connect_gap_ends()
        |-- sort_paths(), connect_extrusions()
        |-- rotate_thick_polylines()
        `-- publish()
*/

namespace slic3r_api { namespace Infill {
namespace {

// One straight section of a vertical line, or one edge of a boundary.
struct Section
{
    Slic3r::Point a;
    Slic3r::Point b;
};
using Sections = std::vector<std::vector<Section>>;

// Bounding box of a set of points.
struct Box
{
    Slic3r::Point min;
    Slic3r::Point max;
};

// Closed boundary edges, queried like AABBTreeLines::LinesDistancer.
class Boundary
{
public:
    explicit Boundary(const Slic3r::Polygons &polygons);
    // Intersections with a line, ordered by distance from its first point.
    std::vector<Slic3r::Point> intersections(const Section &line) const;
    // 1 outside, -1 inside, 0 on the boundary or undetermined.
    int outside(const Slic3r::Point &point) const;

private:
    // Ray hits above and below the point along one coordinate; {-1, -1} on the boundary.
    template<int coordinate> std::pair<int, int> ray_hits(const Slic3r::Point &point) const;

    std::vector<Section> m_edges;
};

// Fill direction like Fill::_infill_direction(), in float.
float infill_direction(const raw_infill_pattern_params &params);
// Copy the surface and rotate it around the origin like polygons_rotate().
StoredExPolygonCollection rotated_surface(storage_handle *storage, const ExPolygon &surface, double angle);
// Bounding box of polygons.
Box extents(const Slic3r::Polygons &polygons);
// The area wide enough for lines, shrunk to the line centers.
Slic3r::Polygons inner_area(storage_handle *storage, const ExPolygonCollection &filled, coord_t spacing, double overlap);
// Cut vertical lines, one spacing apart, by the inner area.
Sections vertical_sections(const Boundary &walls, const Box &box, coord_t spacing, const plugin_run_context *run_ctx);
// Remove clusters of short sections that would make the nozzle vibrate.
void stop_vibrations(Sections &sections);
// Drop removed sections and sort each vertical line from bottom to top.
void sort_sections(Sections &sections);
// One constant-width line per section.
Slic3r::ThickPolylines section_lines(const Sections &sections, coord_t spacing);
// Fill the gaps the sections leave in the surface with Arachne walls.
void fill_gaps(storage_handle *storage, const Sections &sections, const ExPolygonCollection &filled, coord_t spacing,
               const raw_infill_pattern_params &params, const Slic3r::Arachne::WallToolPathsSettings &settings,
               const plugin_run_context *run_ctx, Slic3r::ThickPolylines &lines);
// Trace the area covered by the sections.
Slic3r::Polygons reconstructed_area(const Sections &sections, coord_t spacing, double squared_reconnection);
// Append the Arachne walls of one gap.
void append_gap_walls(const ExPolygon &gap, coord_t spacing, const raw_infill_pattern_params &params,
                      const Slic3r::Arachne::WallToolPathsSettings &settings, Slic3r::ThickPolylines &lines);
// Join gap walls whose ends nearly meet in line, then drop the short thin ones.
void connect_gap_ends(Slic3r::ThickPolylines &lines, coord_t spacing);
// Bounding box corner of a line.
Slic3r::Point min_corner(const Slic3r::Points &points);
// Order the lines like Algorithm::sort_paths(): touching lines first, then the nearest one.
void sort_paths(Slic3r::ThickPolylines &lines, const Slic3r::Point &start, double touch_limit_distance);
// The consecutive segments of a line.
std::vector<Section> path_segments(const Slic3r::ThickPolyline &line);
// Whether two segment chains come closer than a distance.
bool paths_touch(const std::vector<Section> &left, const std::vector<Section> &right, double touch_limit_distance);
// Distance from a point to the nearest segment.
double distance_from_segments(const std::vector<Section> &segments, const Slic3r::Point &point);
// Squared distance from a point to a segment, like line_alg::distance_to_squared().
double distance_to_squared(const Section &segment, const Slic3r::Point &point);
// Join consecutive lines whose ends are close.
void connect_extrusions(Slic3r::ThickPolylines &lines, double squared_reconnection);
// Rotate with rounding and remove adjacent identical point/width pairs, preserving
// width transitions even when their positions coincide.
void rotate_thick_polylines(Slic3r::ThickPolylines &lines, double cos_angle, double sin_angle);
// Publish one extrusion per line in one group.
void publish(const run_ctx_infill_pattern &ctx, storage_handle *storage, const Slic3r::ThickPolylines &lines,
             const plugin_run_context *run_ctx);
// Length of a section.
double length(const Section &section);
class EnsuringPattern;
// One stateless provider serves independent surface calls.
EnsuringPattern &instance(orchestrator_handle *orchestrator);

Boundary::Boundary(const Slic3r::Polygons &polygons)
{
    for (const Slic3r::Polygon &polygon : polygons)
        for (size_t index = 0; index < polygon.points.size(); ++index)
            m_edges.push_back({polygon.points[index], polygon.points[(index + 1) % polygon.points.size()]});
}

std::vector<Slic3r::Point> Boundary::intersections(const Section &line) const
{
    // Same arithmetic as line_alg::intersection(); every edge is tested, the tree only pruned them.
    std::vector<std::pair<double, Slic3r::Point>> hits;
    const Slic3r::Vec2d v1 = (line.b - line.a).cast<double>();
    for (const Section &edge : m_edges) {
        const Slic3r::Vec2d v2 = (edge.b - edge.a).cast<double>();
        const double denom = v1.x() * v2.y() - v1.y() * v2.x();
        if (std::fabs(denom) < EPSILON)
            continue;
        const Slic3r::Vec2d v12 = (line.a - edge.a).cast<double>();
        const double t1 = (v2.x() * v12.y() - v2.y() * v12.x()) / denom;
        const double t2 = (v1.x() * v12.y() - v1.y() * v12.x()) / denom;
        if (t1 >= 0 && t1 <= 1.0f && t2 >= 0 && t2 <= 1.0f) {
            const Slic3r::Vec2d point = line.a.cast<double>() + t1 * v1;
            const Slic3r::Point hit = std::is_same<coord_t, int64_t>::value ?
                Slic3r::Point(coord_t(std::round(point.x())), coord_t(std::round(point.y()))) :
                Slic3r::Point(coord_t(point.x()), coord_t(point.y()));
            hits.emplace_back((hit - line.a).cast<double>().squaredNorm(), hit);
        }
    }
    std::sort(hits.begin(), hits.end(),
              [](const std::pair<double, Slic3r::Point> &left, const std::pair<double, Slic3r::Point> &right) {
                  return left.first < right.first;
              });
    std::vector<Slic3r::Point> out;
    out.reserve(hits.size());
    for (const std::pair<double, Slic3r::Point> &hit : hits)
        out.push_back(hit.second);
    return out;
}

int Boundary::outside(const Slic3r::Point &point) const
{
    // Same decisions as point_outside_closed_contours().
    if (m_edges.empty())
        return 1;
    std::pair<int, int> hits = ray_hits<0>(point);
    if (hits.first < 0 || hits.second < 0)
        return 0;
    if (hits.first % 2 == 1 && hits.second % 2 == 1)
        return -1;
    if (hits.first % 2 == 0 && hits.second % 2 == 0)
        return 1;
    hits = ray_hits<1>(point);
    if (hits.first < 0 || hits.second < 0)
        return 0;
    if (hits.first % 2 == 1 && hits.second % 2 == 1)
        return -1;
    if (hits.first % 2 == 0 && hits.second % 2 == 0)
        return 1;
    return 0;
}

template<int coordinate> std::pair<int, int> Boundary::ray_hits(const Slic3r::Point &point) const
{
    // Same arithmetic as coordinate_aligned_ray_hit_count().
    constexpr int other = (coordinate + 1) % 2;
    std::pair<int, int> hits{0, 0};
    for (const Section &edge : m_edges) {
        if (point[other] < std::min(edge.a[other], edge.b[other]) || point[other] >= std::max(edge.a[other], edge.b[other]))
            continue;
        const coord_t edge_max = std::max(edge.a[coordinate], edge.b[coordinate]);
        const coord_t edge_min = std::min(edge.a[coordinate], edge.b[coordinate]);
        if (point[coordinate] > edge_max) {
            ++hits.first;
        } else if (point[coordinate] < edge_min) {
            ++hits.second;
        } else {
            const double distance_other = edge.b[other] - edge.a[other];
            double value;
            if (std::abs(distance_other) < SCALED_EPSILON) {
                value = edge.a[coordinate];
            } else {
                const double t = (point[other] - edge.a[other]) / distance_other;
                value = edge.a[coordinate] + t * (edge.b[coordinate] - edge.a[coordinate]);
            }
            if (point[coordinate] > value)
                ++hits.first;
            else if (point[coordinate] < value)
                ++hits.second;
            else
                return {-1, -1};
        }
    }
    return hits;
}

float infill_direction(const raw_infill_pattern_params &params)
{
    float angle = float(params.angle);
    if (params.bridge_angle >= 0)
        angle = float(params.bridge_angle);
    else
        angle += (params.infill_flags & C_INFILL_FLAG_CAN_ANGLE_CROSS) != 0 && (params.layer_id & 1) != 0 ?
            float(PatternHelpers::k_half_pi) : 0.f;
    angle += float(PatternHelpers::k_half_pi);
    return angle;
}

StoredExPolygonCollection rotated_surface(storage_handle *storage, const ExPolygon &surface, double angle)
{
    StoredExPolygonCollection rotated(storage, surface);
    expolygon_handle *expolygon = expolygons_at(rotated.mutable_handle(), 0);
    multipoint_rotate_origin(polygon_as_multipoint(expolygon_contour(expolygon)), angle);
    for (uint32_t hole = 0; hole < expolygon_hole_size(expolygon); ++hole)
        multipoint_rotate_origin(polygon_as_multipoint(expolygon_hole_at(expolygon, hole)), angle);
    return rotated;
}

Box extents(const Slic3r::Polygons &polygons)
{
    Box box{Slic3r::Point(std::numeric_limits<coord_t>::max(), std::numeric_limits<coord_t>::max()),
            Slic3r::Point(std::numeric_limits<coord_t>::lowest(), std::numeric_limits<coord_t>::lowest())};
    for (const Slic3r::Polygon &polygon : polygons)
        for (const Slic3r::Point &point : polygon.points) {
            box.min = box.min.cwiseMin(point);
            box.max = box.max.cwiseMax(point);
        }
    return box;
}

Slic3r::Polygons inner_area(storage_handle *storage, const ExPolygonCollection &filled, coord_t spacing, double overlap)
{
    const ClipperOperand area(storage, filled);
    const ClipperOperand opened = clipper_offset2(area, -double(2 * spacing), double(3 * spacing));
    const ClipperOperand wide = clipper_intersection(area, opened);
    const StoredPolygonCollection inner =
        clipper_offset(wide, -(spacing * 0.5 - scale_d(overlap))).to_polygon_collection();
    return ArachneInfill::to_polygons(inner.readonly());
}

Sections vertical_sections(const Boundary &walls, const Box &box, coord_t spacing, const plugin_run_context *run_ctx)
{
    const size_t line_count = size_t((box.max.x() - box.min.x() + spacing - 1) / spacing);
    Sections sections(line_count);
    for (size_t index = 0; index < line_count; ++index) {
        if ((index & 63u) == 0)
            throw_if_cancelled(run_ctx);
        const coord_t x = coord_t(box.min.x() + index * double(spacing));
        const std::vector<Slic3r::Point> hits = walls.intersections({Slic3r::Point(x, box.min.y()), Slic3r::Point(x, box.max.y())});
        for (int hit = 0; hit < int(hits.size()) - 1; ++hit) {
            const Slic3r::Point &a = hits[hit];
            const Slic3r::Point &b = hits[hit + 1];
            if (walls.outside(Slic3r::Point((a.x() + b.x()) / 2, (a.y() + b.y()) / 2)) < 0 &&
                std::abs(a.y() - b.y()) > spacing)
                sections[index].push_back({a, b});
        }
    }
    return sections;
}

void stop_vibrations(Sections &sections)
{
    struct Node
    {
        int section_idx;
        int line_idx;
        int skips_taken = 0;
        bool neighbours_explored = false;
        std::vector<std::pair<int, int>> neighbours{};
    };
    const auto segments_overlap = [](coord_t alow, coord_t ahigh, coord_t blow, coord_t bhigh) {
        return (alow >= blow && alow <= bhigh) || (ahigh >= blow && ahigh <= bhigh) || (blow >= alow && blow <= ahigh) ||
               (bhigh >= alow && bhigh <= ahigh);
    };

    const coord_t length_filter = scale_i(4);
    const size_t skips_allowed = 2;
    const size_t min_removal_count = 5;
    for (int section_idx = 0; section_idx < int(sections.size()); ++section_idx) {
        for (int line_idx = 0; line_idx < int(sections[section_idx].size()); ++line_idx) {
            const Section &line = sections[section_idx][line_idx];
            if (line.a == line.b || length(line) >= length_filter)
                continue;
            std::set<std::pair<int, int>> to_remove{{section_idx, line_idx}};
            std::vector<Node> to_visit{{section_idx, line_idx}};

            bool initial_touches_long_lines = false;
            if (section_idx > 0)
                for (const Section &previous : sections[section_idx - 1])
                    if (previous.a != previous.b && segments_overlap(line.a.y(), line.b.y(), previous.a.y(), previous.b.y()))
                        initial_touches_long_lines = true;

            while (!to_visit.empty()) {
                const Node current = to_visit.back();
                const Section &current_line = sections[current.section_idx][current.line_idx];
                if (current.neighbours_explored) {
                    bool removable = length(current_line) < length_filter &&
                        ((int(to_remove.size()) - current.skips_taken > int(min_removal_count)) ||
                         (current.neighbours.empty() && !initial_touches_long_lines));
                    if (!removable)
                        for (const std::pair<int, int> &neighbour : current.neighbours)
                            if (to_remove.find(neighbour) != to_remove.end()) {
                                removable = true;
                                break;
                            }
                    if (!removable)
                        to_remove.erase({current.section_idx, current.line_idx});
                    to_visit.pop_back();
                } else {
                    to_visit.back().neighbours_explored = true;
                    const size_t current_index = to_visit.size() - 1;
                    const bool can_use_skip = length(current_line) <= length_filter && current.skips_taken < int(skips_allowed);
                    if (current.section_idx + 1 < int(sections.size()))
                        for (int next_idx = 0; next_idx < int(sections[current.section_idx + 1].size()); ++next_idx) {
                            const Section &next = sections[current.section_idx + 1][next_idx];
                            if (next.a != next.b &&
                                segments_overlap(current_line.a.y(), current_line.b.y(), next.a.y(), next.b.y()) &&
                                (length(next) < length_filter || can_use_skip)) {
                                to_visit[current_index].neighbours.push_back({current.section_idx + 1, next_idx});
                                to_remove.insert({current.section_idx + 1, next_idx});
                                to_visit.push_back(Node{current.section_idx + 1, next_idx,
                                                        current.skips_taken + (length(next) >= length_filter)});
                            }
                        }
                }
            }

            for (const std::pair<int, int> &removed : to_remove) {
                Section &section = sections[removed.first][removed.second];
                section.a = section.b;
            }
        }
    }
}

void sort_sections(Sections &sections)
{
    for (std::vector<Section> &column : sections) {
        column.erase(std::remove_if(column.begin(), column.end(), [](const Section &s) { return s.a == s.b; }),
                     column.end());
        std::sort(column.begin(), column.end(), [](const Section &a, const Section &b) { return a.a.y() < b.b.y(); });
    }
}

Slic3r::ThickPolylines section_lines(const Sections &sections, coord_t spacing)
{
    Slic3r::ThickPolylines lines;
    for (const std::vector<Section> &column : sections)
        for (const Section &section : column) {
            Slic3r::ThickPolyline &line = lines.emplace_back();
            line.points.push_back(section.a);
            line.points_width.push_back(spacing);
            line.points.push_back(section.b);
            line.points_width.push_back(spacing);
            line.endpoints = {true, true};
        }
    return lines;
}

void fill_gaps(storage_handle *storage, const Sections &sections, const ExPolygonCollection &filled, coord_t spacing,
               const raw_infill_pattern_params &params, const Slic3r::Arachne::WallToolPathsSettings &settings,
               const plugin_run_context *run_ctx, Slic3r::ThickPolylines &lines)
{
    const double reconnection = 2.0 * double(spacing);
    const StoredPolygonCollection traced =
        ArachneInfill::to_api(storage, reconstructed_area(sections, spacing, reconnection * reconnection));
    const ClipperOperand covered = clipper_union_with_safety_offset(ClipperOperand(storage, traced.readonly()));
    ClipperOperand gaps = clipper_diff(ClipperOperand(storage, filled), covered);
    if (params.overlap != 0)
        gaps = clipper_offset(gaps, scale_d(params.overlap));
    const StoredExPolygonCollection gap_areas = gaps.to_expolygon_collection();
    for (const ExPolygon gap : gap_areas) {
        throw_if_cancelled(run_ctx);
        append_gap_walls(gap, spacing, params, settings, lines);
    }

    std::sort(lines.begin(), lines.end(), [](const Slic3r::ThickPolyline &left, const Slic3r::ThickPolyline &right) {
        const Slic3r::Point left_min = min_corner(left.points);
        const Slic3r::Point right_min = min_corner(right.points);
        if (left_min.x() == right_min.x())
            return left_min.y() < right_min.y();
        return left_min.x() < right_min.x();
    });
    connect_gap_ends(lines, spacing);
}

Slic3r::Polygons reconstructed_area(const Sections &sections, coord_t spacing, double squared_reconnection)
{
    struct TracedPoly
    {
        Slic3r::Points lows;
        Slic3r::Points highs;
    };

    Sections widened = sections;
    for (std::vector<Section> &column : widened)
        for (Section &section : column) {
            section.a -= Slic3r::Point{0.0, 0.5 * spacing};
            section.b += Slic3r::Point{0.0, 0.5 * spacing};
        }

    Slic3r::Polygons area;
    std::vector<TracedPoly> current_traced_polys;
    for (const std::vector<Section> &column : widened) {
        std::unordered_set<const Section *> used_segments;
        for (TracedPoly &traced_poly : current_traced_polys) {
            const std::vector<Section>::const_iterator candidates_begin =
                std::upper_bound(column.begin(), column.end(), traced_poly.lows.back(),
                                 [](const Slic3r::Point &low, const Section &seg) { return seg.b.y() > low.y(); });
            const std::vector<Section>::const_iterator candidates_end =
                std::upper_bound(column.begin(), column.end(), traced_poly.highs.back(),
                                 [](const Slic3r::Point &high, const Section &seg) { return seg.a.y() > high.y(); });

            bool segment_added = false;
            for (std::vector<Section>::const_iterator candidate = candidates_begin;
                 candidate != candidates_end && !segment_added; candidate++) {
                if (used_segments.find(&(*candidate)) != used_segments.end())
                    continue;
                // The native code reads candidates_begin here, not candidate.
                if ((traced_poly.lows.back() - candidates_begin->a).cast<double>().squaredNorm() < squared_reconnection) {
                    traced_poly.lows.push_back(candidates_begin->a);
                } else {
                    traced_poly.lows.push_back(traced_poly.lows.back() + Slic3r::Point{spacing / 2, 0});
                    traced_poly.lows.push_back(candidates_begin->a - Slic3r::Point{spacing / 2, 0});
                    traced_poly.lows.push_back(candidates_begin->a);
                }

                if ((traced_poly.highs.back() - candidates_begin->b).cast<double>().squaredNorm() < squared_reconnection) {
                    traced_poly.highs.push_back(candidates_begin->b);
                } else {
                    traced_poly.highs.push_back(traced_poly.highs.back() + Slic3r::Point{spacing / 2, 0});
                    traced_poly.highs.push_back(candidates_begin->b - Slic3r::Point{spacing / 2, 0});
                    traced_poly.highs.push_back(candidates_begin->b);
                }
                segment_added = true;
                used_segments.insert(&(*candidates_begin));
            }

            if (!segment_added) {
                // Zero or several overlapping sections: close this polygon, new ones may open.
                traced_poly.lows.push_back(traced_poly.lows.back() + Slic3r::Point{spacing / 2, 0});
                traced_poly.highs.push_back(traced_poly.highs.back() + Slic3r::Point{spacing / 2, 0});
                Slic3r::Polygon &new_poly = area.emplace_back(std::move(traced_poly.lows));
                new_poly.points.insert(new_poly.points.end(), traced_poly.highs.rbegin(), traced_poly.highs.rend());
                traced_poly.lows.clear();
                traced_poly.highs.clear();
            }
        }

        current_traced_polys.erase(std::remove_if(current_traced_polys.begin(), current_traced_polys.end(),
                                                  [](const TracedPoly &tp) { return tp.lows.empty(); }),
                                   current_traced_polys.end());

        for (const Section &segment : column)
            if (used_segments.find(&segment) == used_segments.end()) {
                TracedPoly &new_tp = current_traced_polys.emplace_back();
                new_tp.lows.push_back(segment.a - Slic3r::Point{spacing / 2, 0});
                new_tp.lows.push_back(segment.a);
                new_tp.highs.push_back(segment.b - Slic3r::Point{spacing / 2, 0});
                new_tp.highs.push_back(segment.b);
            }
    }

    for (TracedPoly &traced_poly : current_traced_polys) {
        Slic3r::Polygon &new_poly = area.emplace_back(std::move(traced_poly.lows));
        new_poly.points.insert(new_poly.points.end(), traced_poly.highs.rbegin(), traced_poly.highs.rend());
    }
    return area;
}

void append_gap_walls(const ExPolygon &gap, coord_t spacing, const raw_infill_pattern_params &params,
                      const Slic3r::Arachne::WallToolPathsSettings &settings, Slic3r::ThickPolylines &lines)
{
    const c_bounding_box box = gap.contour().bounding_box();
    const coord_t loops_count = (std::max(box.max.x - box.min.x, box.max.y - box.min.y) + spacing - 1) / spacing;
    const Slic3r::Polygons polygons = ArachneInfill::to_polygons(gap);
    Slic3r::Arachne::WallToolPaths walls(polygons, spacing, spacing, spacing, spacing, size_t(loops_count), 0,
                                         params.layer_height, settings);
    for (const Slic3r::Arachne::VariableWidthLines &inset : walls.getToolPaths())
        for (const Slic3r::Arachne::ExtrusionLine &wall : inset) {
            if (wall.junctions.size() < 2)
                continue;
            Slic3r::ThickPolyline line = Slic3r::Arachne::to_thick_polyline(wall);
            if (wall.is_closed) {
                line.start_at_index(int(ArachneInfill::nearest_point_index(line.points, Slic3r::Point(box.min.x, box.min.y))));
                line.clip_end(spacing * 0.5);
            }
            if (line.is_valid() && line.length() > 0 && line.points.size() > 1)
                lines.push_back(line);
        }
}

void connect_gap_ends(Slic3r::ThickPolylines &lines, coord_t spacing)
{
    struct EndPoint
    {
        Slic3r::Vec2d position;
        size_t polyline_idx;
        size_t other_end_point_idx;
        bool is_first;
        bool used = false;
    };
    std::vector<EndPoint> connection_endpoints;
    connection_endpoints.reserve(lines.size() * 2);
    for (size_t index = 0; index < lines.size(); ++index) {
        const size_t current_idx = connection_endpoints.size();
        connection_endpoints.push_back({lines[index].front().cast<double>(), index, current_idx + 1, true});
        connection_endpoints.push_back({lines[index].back().cast<double>(), index, current_idx, false});
    }

    std::vector<bool> linear_segment_flags(lines.size());
    for (size_t index = 0; index < lines.size(); ++index) {
        const Slic3r::ThickPolyline &line = lines[index];
        linear_segment_flags[index] = line.points.size() == 2 && line.points.front().x() == line.points.back().x() &&
                                      line.points_width.front() == spacing && line.points_width.back() == spacing;
    }

    const auto coord_fn = [&connection_endpoints](size_t idx, size_t dim) { return connection_endpoints[idx].position[dim]; };
    // The native tree keeps its historic traversal tolerance.
    const KDTreeIndirect<2, double, decltype(coord_fn)> endpoints_tree{coord_fn, connection_endpoints.size(), EPSILON};
    for (size_t ep_idx = 0; ep_idx < connection_endpoints.size(); ++ep_idx) {
        EndPoint &ep1 = connection_endpoints[ep_idx];
        if (ep1.used)
            continue;
        const std::vector<size_t> close_endpoints = find_nearby_points(endpoints_tree, ep1.position, double(spacing));
        for (const size_t close_endpoint_idx : close_endpoints) {
            EndPoint &ep2 = connection_endpoints[close_endpoint_idx];
            if (ep2.used || ep2.polyline_idx == ep1.polyline_idx ||
                (linear_segment_flags[ep1.polyline_idx] && linear_segment_flags[ep2.polyline_idx]))
                continue;

            EndPoint &target_ep = ep1.polyline_idx > ep2.polyline_idx ? ep1 : ep2;
            EndPoint &source_ep = ep1.polyline_idx > ep2.polyline_idx ? ep2 : ep1;

            Slic3r::ThickPolyline &target_tp = lines[target_ep.polyline_idx];
            Slic3r::ThickPolyline &source_tp = lines[source_ep.polyline_idx];
            linear_segment_flags[target_ep.polyline_idx] =
                linear_segment_flags[ep1.polyline_idx] || linear_segment_flags[ep2.polyline_idx];

            // The native code measures the last segment of an end as a zero vector.
            const Slic3r::Vec2d v1 = target_ep.is_first ?
                (target_tp.points[0] - target_tp.points[1]).cast<double>() :
                (target_tp.points.back() - target_tp.points[target_tp.points.size() - 1]).cast<double>();
            const Slic3r::Vec2d v2 = source_ep.is_first ?
                (source_tp.points[1] - source_tp.points[0]).cast<double>() :
                (source_tp.points[source_tp.points.size() - 1] - source_tp.points.back()).cast<double>();
            if (std::abs(std::atan2(v1.x() * v2.y() - v1.y() * v2.x(), v1.dot(v2))) > PatternHelpers::k_pi / 6.0)
                continue;

            // Join target_ep and source_ep into target_tp; source_tp is cleared.
            if (target_ep.is_first) {
                target_tp.reverse();
                target_ep.is_first = false;
                connection_endpoints[target_ep.other_end_point_idx].is_first = true;
            }
            const size_t new_start_idx = target_ep.other_end_point_idx;
            if (!source_ep.is_first) {
                source_tp.reverse();
                source_ep.is_first = true;
                connection_endpoints[source_ep.other_end_point_idx].is_first = false;
            }
            const size_t new_end_idx = source_ep.other_end_point_idx;

            target_tp.points.insert(target_tp.points.end(), source_tp.points.begin(), source_tp.points.end());
            target_tp.points_width.insert(target_tp.points_width.end(), source_tp.points_width.begin(),
                                          source_tp.points_width.end());
            target_ep.used = true;
            source_ep.used = true;

            connection_endpoints[new_start_idx].polyline_idx = target_ep.polyline_idx;
            connection_endpoints[new_end_idx].polyline_idx = target_ep.polyline_idx;
            connection_endpoints[new_start_idx].other_end_point_idx = new_end_idx;
            connection_endpoints[new_end_idx].other_end_point_idx = new_start_idx;
            source_tp.clear();
            break;
        }
    }

    lines.erase(std::remove_if(lines.begin(), lines.end(),
                               [spacing](const Slic3r::ThickPolyline &line) {
                                   return line.length() < spacing &&
                                          std::all_of(line.points_width.begin(), line.points_width.end(),
                                                      [spacing](double width) { return width < spacing; });
                               }),
                lines.end());
}

Slic3r::Point min_corner(const Slic3r::Points &points)
{
    Slic3r::Point corner = points.front();
    for (const Slic3r::Point &point : points)
        corner = corner.cwiseMin(point);
    return corner;
}

void sort_paths(Slic3r::ThickPolylines &lines, const Slic3r::Point &start, double touch_limit_distance)
{
    const size_t paths_count = lines.size();
    if (paths_count <= 1)
        return;

    std::vector<std::vector<Section>> segments(paths_count);
    for (size_t path_idx = 0; path_idx < paths_count; ++path_idx)
        segments[path_idx] = path_segments(lines[path_idx]);

    std::vector<std::unordered_set<size_t>> dependencies(paths_count);
    for (size_t path_idx = 0; path_idx < paths_count; ++path_idx)
        for (size_t next_path_idx = path_idx + 1; next_path_idx < paths_count; ++next_path_idx)
            if (paths_touch(segments[path_idx], segments[next_path_idx], touch_limit_distance))
                dependencies[next_path_idx].insert(path_idx);

    Slic3r::Point current_point = start;
    std::vector<std::pair<size_t, bool>> correct_order_and_direction(paths_count);
    size_t unsorted_idx = 0;
    const size_t null_idx = size_t(-1);
    size_t next_idx = null_idx;
    bool reverse = false;
    while (unsorted_idx < paths_count) {
        next_idx = null_idx;
        double lines_dist = std::numeric_limits<double>::max();
        for (size_t path_idx = 0; path_idx < paths_count; ++path_idx) {
            if (!dependencies[path_idx].empty())
                continue;
            const double ldist = distance_from_segments(segments[path_idx], current_point);
            if (ldist < lines_dist) {
                const double dist_a = (segments[path_idx].front().a - current_point).cast<double>().squaredNorm();
                const double dist_b = (segments[path_idx].back().b - current_point).cast<double>().squaredNorm();
                next_idx = path_idx;
                reverse = dist_b < dist_a;
                lines_dist = ldist;
            }
        }

        correct_order_and_direction[next_idx] = {unsorted_idx, reverse};
        ++unsorted_idx;
        current_point = reverse ? segments[next_idx].front().a : segments[next_idx].back().b;

        dependencies[next_idx].insert(null_idx);
        for (size_t path_idx = 0; path_idx < paths_count; ++path_idx)
            dependencies[path_idx].erase(next_idx);
    }

    for (size_t path_idx = 0; path_idx < paths_count; ++path_idx)
        if (correct_order_and_direction[path_idx].second)
            lines[path_idx].reverse();

    for (size_t i = 0; i < correct_order_and_direction.size() - 1; ++i) {
        bool swapped = false;
        for (size_t j = 0; j < correct_order_and_direction.size() - i - 1; ++j)
            if (correct_order_and_direction[j].first > correct_order_and_direction[j + 1].first) {
                std::swap(correct_order_and_direction[j], correct_order_and_direction[j + 1]);
                std::swap(lines[j], lines[j + 1]);
                swapped = true;
            }
        if (!swapped)
            break;
    }
}

std::vector<Section> path_segments(const Slic3r::ThickPolyline &line)
{
    std::vector<Section> segments;
    Slic3r::Point previous = line.front();
    for (size_t index = 1; index < line.points.size(); ++index) {
        segments.push_back({previous, line.points[index]});
        previous = segments.back().b;
    }
    return segments;
}

bool paths_touch(const std::vector<Section> &left, const std::vector<Section> &right, double touch_limit_distance)
{
    for (const Section &segment : left)
        if (distance_from_segments(right, segment.a) < touch_limit_distance)
            return true;
    if (distance_from_segments(right, left.back().b) < touch_limit_distance)
        return true;
    for (const Section &segment : right)
        if (distance_from_segments(left, segment.a) < touch_limit_distance)
            return true;
    return distance_from_segments(left, right.back().b) < touch_limit_distance;
}

double distance_from_segments(const std::vector<Section> &segments, const Slic3r::Point &point)
{
    if (segments.empty())
        return std::numeric_limits<double>::infinity();
    double best = std::numeric_limits<double>::infinity();
    for (const Section &segment : segments)
        best = std::min(best, distance_to_squared(segment, point));
    return std::sqrt(best);
}

double distance_to_squared(const Section &segment, const Slic3r::Point &point)
{
    const Slic3r::Vec2d v = (segment.b - segment.a).cast<double>();
    const Slic3r::Vec2d va = (point - segment.a).cast<double>();
    const double l2 = v.squaredNorm();
    if (l2 == 0.0)
        return va.squaredNorm();
    const double t = va.dot(v);
    if (t <= 0.0)
        return va.squaredNorm();
    if (t >= l2)
        return (point - segment.b).cast<double>().squaredNorm();
    const Slic3r::Vec2d w = ((t / l2) * v).eval();
    return (w - va).squaredNorm();
}

void connect_extrusions(Slic3r::ThickPolylines &lines, double squared_reconnection)
{
    Slic3r::ThickPolylines connected;
    if (!lines.empty()) {
        connected.push_back(lines.front());
        for (size_t index = 1; index < lines.size(); ++index) {
            Slic3r::ThickPolyline &line = lines[index];
            Slic3r::ThickPolyline &tail = connected.back();
            const Slic3r::Point last = tail.back();
            if ((last - line.back()).cast<double>().squaredNorm() < (last - line.front()).cast<double>().squaredNorm())
                line.reverse();
            if ((last - line.front()).cast<double>().squaredNorm() < squared_reconnection) {
                tail.points.insert(tail.points.end(), line.points.begin(), line.points.end());
                tail.points_width.insert(tail.points_width.end(), line.points_width.begin(), line.points_width.end());
            } else {
                connected.push_back(line);
            }
        }
    }
    lines = connected;
}

void rotate_thick_polylines(Slic3r::ThickPolylines &lines, double cos_angle, double sin_angle)
{
    for (Slic3r::ThickPolyline &line : lines) {
        size_t count = 0;
        for (size_t index = 0; index < line.points.size(); ++index) {
            const double px = double(line.points[index].x());
            const double py = double(line.points[index].y());
            const Slic3r::Point point(coord_t(std::round(cos_angle * px - sin_angle * py)),
                                      coord_t(std::round(cos_angle * py + sin_angle * px)));
            const coord_t width = line.points_width[index];
            // A repeated join contributes neither length nor a width transition.
            if (count > 0 && line.points[count - 1] == point && line.points_width[count - 1] == width)
                continue;
            line.points[count] = point;
            line.points_width[count++] = width;
        }
        line.points.resize(count);
        line.points_width.resize(count);
    }
}

void publish(const run_ctx_infill_pattern &ctx, storage_handle *storage, const Slic3r::ThickPolylines &lines,
             const plugin_run_context *run_ctx)
{
    StoredExtrusionEntity group(storage);
    for (size_t index = 0; index < lines.size(); ++index) {
        if ((index & 255u) == 0)
            throw_if_cancelled(run_ctx);
        std::optional<StoredExtrusionEntity> path = ArachneInfill::to_extrusion(storage, lines[index], *ctx.params);
        if (path.has_value())
            group.append_child_move(path->mutable_view());
    }
    throw_if_cancelled(run_ctx);
    if (group.empty())
        return;
    // Like the legacy collection, the lines may be sorted unless the recipe is monotonic.
    if ((ctx.params->infill_flags & C_INFILL_FLAG_MONOTONIC) != 0) {
        group.disable_sort();
        group.disable_reverse();
    } else {
        group.set_flags(group.flags() | RAW_EXTRUSION_FLAG_SORTABLE | RAW_EXTRUSION_FLAG_REVERSIBLE);
    }
    MutableExtrusionEntity(ctx.output).append_child_move(group.mutable_view());
}

double length(const Section &section)
{
    return (section.b - section.a).cast<double>().norm();
}

class EnsuringPattern : public PluginBase
{
public:
    explicit EnsuringPattern(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override {
        if (keys != nullptr)
            std::copy(std::begin(ArachneInfill::k_settings_keys), std::end(ArachneInfill::k_settings_keys), keys);
        return int32_t(ArachneInfill::k_settings_key_count);
    }
private:
    const char *id_impl() const noexcept override { return "ensuring"; }
    const char *name_impl() const noexcept override { return "Ensuring"; }
    const char *description_impl() const noexcept override {
        return "Solid infill pattern that improves coverage in narrow areas.";
    }
    slicing_step_t step_impl() const noexcept override { return INFILL_PATTERN; }
    const char *const *dependencies_impl() const noexcept override {
        static const char *none[] = {nullptr};
        return none;
    }
    int32_t priority_impl() const noexcept override { return 24; }
    const char *progress_message_format_impl() const noexcept override { return "Generating ensuring infill: %u / %u surfaces"; }
    void inilialize_impl(storage_handle *) const override {
        PatternHelpers::add_self_to_pattern_list(m_orchestrator, "ensuring", "Ensuring", {"solid_fill_pattern"});
    }
    void run_impl(const plugin_run_context *run_ctx) const override {
        throw_if_cancelled(run_ctx);
        const run_ctx_infill_pattern *ctx = plugin_ctx_as_infill_pattern(run_ctx);
        if (ctx == nullptr || ctx->surface == nullptr || ctx->params == nullptr || ctx->output == nullptr ||
            ctx->primary_region == nullptr || run_ctx->plugin_storage == nullptr)
            return;
        fill_ensuring_surface(*ctx, run_ctx->plugin_storage, run_ctx, ArachneInfill::read_settings(*ctx));
        progress().increment();
    }
};

EnsuringPattern &instance(orchestrator_handle *orchestrator)
{
    static EnsuringPattern pattern(orchestrator);
    return pattern;
}
} // namespace

void fill_ensuring_surface(const run_ctx_infill_pattern &ctx, storage_handle *storage, const plugin_run_context *run_ctx,
                           const Slic3r::Arachne::WallToolPathsSettings &arachne_settings)
{
    throw_if_cancelled(run_ctx);
    const raw_infill_pattern_params &params = *ctx.params;
    const coord_t spacing = scale_i(params.spacing);
    if (spacing <= 0)
        return;
    const double reconnection = 2.0 * double(spacing);
    const double squared_reconnection = reconnection * reconnection;
    const double aligning_angle = -infill_direction(params) + PatternHelpers::k_pi;
    const StoredExPolygonCollection filled = rotated_surface(storage, Surface(ctx.surface).expolygon(), aligning_angle);
    const Slic3r::Polygons filled_area = ArachneInfill::to_polygons(filled.readonly());
    const Box box = extents(filled_area);

    throw_if_cancelled(run_ctx);
    const Boundary walls(inner_area(storage, filled.readonly(), spacing, params.overlap));
    Sections sections = vertical_sections(walls, box, spacing, run_ctx);
    stop_vibrations(sections);
    sort_sections(sections);
    Slic3r::ThickPolylines lines = section_lines(sections, spacing);
    fill_gaps(storage, sections, filled.readonly(), spacing, params, arachne_settings, run_ctx, lines);

    throw_if_cancelled(run_ctx);
    sort_paths(lines, box.min, double(spacing) * 1.2);
    connect_extrusions(lines, squared_reconnection);
    rotate_thick_polylines(lines, std::cos(-aligning_angle), std::sin(-aligning_angle));
    publish(ctx, storage, lines, run_ctx);
}

void register_ensuring_pattern_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(orchestrator, instance(orchestrator).c_instance());
}
}} // namespace slic3r_api::Infill
