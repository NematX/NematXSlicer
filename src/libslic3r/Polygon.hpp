///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ Copyright (c) Prusa Research 2016 - 2023 Tomáš Mészáros @tamasmeszaros, Vojtěch Bubník @bubnikv, Lukáš Matěna @lukasmatena, Lukáš Hejl @hejllukas, Filip Sykala @Jony01, Oleksandra Iushchenko @YuSanka
///|/ Copyright (c) Slic3r 2013 - 2016 Alessandro Ranellucci @alranel
///|/
///|/ ported from lib/Slic3r/Polygon.pm:
///|/ Copyright (c) Prusa Research 2017 - 2022 Vojtěch Bubník @bubnikv
///|/ Copyright (c) Slic3r 2011 - 2014 Alessandro Ranellucci @alranel
///|/ Copyright (c) 2012 Mark Hindess
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Polygon_hpp_
#define slic3r_Polygon_hpp_

#include <string>
#include <vector>

#include "ContainerUtils.hpp"
#include "libslic3r.h"
#include "Line.hpp"
#include "MultiPoint.hpp"
#include "Point.hpp"
#include "Polyline.hpp"

/* Polygon provides owned closed rings to slicing geometry. Construct a ring from
   scaled points without a repeated endpoint, then query its area, winding or edges.
   The move and temporary-point constructors normalize near-coincident endpoints. */
namespace Slic3r {

class Polygon;
using Polygons          = std::vector<Polygon, tbb::scalable_allocator<Polygon>>;
using PolygonPtrs       = std::vector<Polygon*, tbb::scalable_allocator<Polygon*>>;
using ConstPolygonPtrs  = std::vector<const Polygon*, tbb::scalable_allocator<const Polygon*>>;
class Line;
using Lines             = std::vector<Line>;

// Returns true if inside. Returns border_result if on boundary.
bool contains(const Polygon& polygon, const Point& p, bool border_result = true);
bool contains(const Polygons& polygons, const Point& p, bool border_result = true);

class Polygon : public MultiPoint
{
public:
    Polygon() = default;
    explicit Polygon(const Points &points) : MultiPoint(points) {
        assert(points.size() != 1);
        if (points.size() > 1 && this->front().coincides_with(this->back()))
            this->points.pop_back();
    }
    // Drop a last point within SCALED_EPSILON / 2 of the first; input coordinates must be scaled.
    explicit Polygon(Points &&points) : MultiPoint(points) {
        assert(points.size() != 1);
        if (points.size() > 1 && this->front().coincides_with_epsilon(this->back()))
            this->points.pop_back();
    }
    // Drop a last point within SCALED_EPSILON / 2 of the first; input coordinates must be scaled.
    Polygon(std::initializer_list<Point> points) : MultiPoint(points) {
        assert(this->size() != 1);
        assert(this->empty() || !this->front().coincides_with(this->back()));
        if (this->size() > 1 && this->front().coincides_with_epsilon(this->back()))
            this->points.pop_back();
    }
    Polygon(const Polygon &other) : MultiPoint(other.points) {
        assert(this->empty() || !this->front().coincides_with(this->back()));
    }
    // Drop a last point within SCALED_EPSILON / 2 of the first; input coordinates must be scaled.
    Polygon(Polygon &&other) : MultiPoint(std::move(other.points)) {
        assert(this->size() != 1);
        assert(this->empty() || !this->front().coincides_with(this->back()));
        if (this->size() > 1 && this->front().coincides_with_epsilon(this->back()))
            this->points.pop_back();
    }
    static Polygon new_scale(const std::vector<Vec2d> &points) {
        Polygon pgn;
        pgn.points.reserve(points.size());
        for (const Vec2d &pt : points)
            pgn.points.emplace_back(Point::new_scale(pt(0), pt(1)));
        return pgn;
    }
    Polygon& operator=(const Polygon &other) { points = other.points; return *this; }
    Polygon& operator=(Polygon &&other) { points = std::move(other.points); return *this; }

    Point& operator[](Points::size_type idx) { return this->points[idx]; }
    const Point& operator[](Points::size_type idx) const { return this->points[idx]; }

    // last point == first point for polygons
    //please don't use that, prefer 'is_loop', front() and back().
    const Point& last_point() const { return this->points.front(); }
    bool is_loop() const override { return true; }
    bool is_polygon() const override { return true; }; // reflection

    distf_t length() const;
    Lines lines() const;
    Polyline split_at_vertex(const Point &point) const;
    // Split a closed polygon into an open polyline, with the split point duplicated at both ends.
    Polyline split_at_index(size_t index) const;
    // Split a closed polygon into an open polyline, with the split point duplicated at both ends.
    Polyline split_at_first_point() const { return this->split_at_index(0); }
    Points   equally_spaced_points(distf_t distance) const { return this->split_at_first_point().equally_spaced_points(distance); }

    static double area(const Points &pts);
    double area() const;
    bool is_counter_clockwise() const;
    bool is_clockwise() const;
    bool make_counter_clockwise();
    bool make_clockwise();
    bool is_valid() const { assert_valid(); return this->points.size() >= 3; }
    void douglas_peucker(coord_t tolerance) override;

    // Does an unoriented polygon contain a point?
    bool contains(const Point &point) const { return Slic3r::contains(*this, point, true); }
    // Approximate on boundary test.
    bool on_boundary(const Point &point, double eps) const
        { return (this->point_projection(point).first - point).cast<double>().squaredNorm() < eps * eps; }

    // Works on CCW polygons only, CW contour will be reoriented to CCW by Clipper's simplify_polygons()!
    Polygons simplify(distf_t tolerance) const;
    void densify(float min_length, std::vector<float>* lengths = nullptr);
    void densify(distf_t min_length) override;
    void triangulate_convex(Polygons* polygons) const;
    Point centroid() const;

    // Considering CCW orientation of this polygon
    // (it means that a ccw (contour) is mostly convex, while a cw (hole) is mostly concave),
    // find all convex resp. concave points
    // with the angle at the vertex between two threshold.
    Points convex_points(double min_angle /*=0*/, double max_angle /*=PI*/) const;
    Points concave_points(double min_angle, double max_angle) const;
    std::vector<size_t> concave_points_idx(double min_angle, double max_angle) const;
    std::vector<size_t> convex_points_idx(double min_angle, double max_angle) const;
    // Projection of a point onto the polygon.
    std::pair<Point, size_t> point_projection(const Point &point) const override;
    std::vector<float> parameter_by_length() const;
    /// remove points that are (almost) on an existing line from previous & next point.
    /// return number of point removed
    size_t remove_collinear(coord_t max_offset);
    size_t remove_collinear_angle(double angle);
    void remove_point_too_close(const coord_t tolerance);

#ifdef _DEBUGINFO
    void assert_valid() const override;
#else
    void assert_valid() const;
#endif

    using iterator = Points::iterator;
    using const_iterator = Points::const_iterator;
};

bool operator==(const Polygon &lhs, const Polygon &rhs);
bool operator!=(const Polygon &lhs, const Polygon &rhs);

BoundingBox get_extents(const Polygon &poly);
BoundingBox get_extents(const Polygons &polygons);
BoundingBox get_extents_rotated(const Polygon &poly, double angle);
BoundingBox get_extents_rotated(const Polygons &polygons, double angle);
std::vector<BoundingBox> get_extents_vector(const Polygons &polygons);

// Polygon must be valid (at least three points), collinear points and duplicate points removed.
bool        polygon_is_convex(const Points &poly);
bool        polygon_is_convex(const Polygon &poly);

// Test for duplicate points. The points are copied, sorted and checked for duplicates globally.
bool has_duplicate_points(Polygon &&poly);
bool has_duplicate_points(const Polygon &poly);
bool has_duplicate_points(const Polygons &polys);

// Return True when erase some otherwise False.
bool remove_same_neighbor(Polygon &polygon);
bool remove_same_neighbor(Polygons &polygons);
// Normalize each raw mesh ring without merging rings or changing fill rules.
// Remove exact duplicate neighbors and collinear zero-area rings; split exact
// repeated-vertex contacts into separate rings, retaining each ring's winding.
// This does not resolve arbitrary segment intersections or simplify coordinates.
void normalize_mesh_polygons(Polygons &polygons);
// remove any point that are at epsilon  (or resolution) 'distance' (douglas_peuckere algo for now) and all polygons that are too small to be valid

void ensure_valid(Polygons &polygons, coord_t resolution = SCALED_EPSILON);
Polygons ensure_valid(Polygons &&polygons, coord_t resolution = SCALED_EPSILON);
Polygons ensure_valid(coord_t resolution, Polygons &&polygons);
// return false if the polygon isn't valid and need to be removed.
bool ensure_valid(Polygon &polygon, coord_t resolution = SCALED_EPSILON);
// like ensure_valid but you're sure it won't remove colinear points.
void remove_point_too_close(Polygons &polygons, coord_t resolution = SCALED_EPSILON);
#ifdef _DEBUGINFO
void assert_valid(const Polygons &polygons);
#else
inline void assert_valid(const Polygons &polygons) {}
#endif

distf_t total_length(const Polygons &polylines);
double area(const Polygon &poly);
double area(const Polygons &polys);

// Remove sticks (tentacles with zero area) from the polygon.
bool remove_sticks(Polygon &poly);
bool remove_sticks(Polygons &polys);

// Remove polygons with less than 3 edges.
bool remove_degenerate(Polygons &polys);
bool remove_small(Polygons &polys, double min_area);
void remove_collinear(Polygon &poly, coord_t max_offset = SCALED_EPSILON);
void remove_collinear(Polygons &polys, coord_t max_offset = SCALED_EPSILON);

//don't append polygons! or only not-hole polygons!
//prefer appending expolygon, it's safer
// because if you append a big hole at the end of the list, you erase evrything.

// Append a vector of polygons at the end of another vector of polygons.
void polygons_append(Polygons &dst, const Polygons &src);
void polygons_append(Polygons &dst, Polygons &&src);

Polygons polygons_simplify(Polygons &&polys, distf_t tolerance, bool strictly_simple = true);
Polygons polygons_simplify(const Polygons &polys, distf_t tolerance, bool strictly_simple = true);

void polygons_rotate(Polygons &polys, double angle);
void polygons_reverse(Polygons &polys);

Points to_points(const Polygon &poly);
size_t count_points(const Polygons &polys);
Points to_points(const Polygons &polys);

Lines to_lines(const Polygon &poly);
Lines to_lines(const Polygons &polys);

Polyline to_polyline(const Polygon &polygon);
Polylines to_polylines(const Polygon &polygon);
Polylines to_polylines(const Polygons &polygons);
Polylines to_polylines(Polygons &&polys);

Polygons to_polygons(const Polylines &polylines);
Polygons to_polygons(const VecOfPoints &paths);
Polygons to_polygons(VecOfPoints &&paths);

// Do polygons match? If they match, they must have the same topology,
// however their contours may be rotated.
bool polygons_match(const Polygon &l, const Polygon &r);

Polygon make_circle(distf_t radius, distf_t error);
Polygon make_circle_num_segments(distf_t radius, size_t num_segments);

} // Slic3r

#endif

