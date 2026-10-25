///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ Copyright (c) Prusa Research 2016 - 2023 Pavel Mikuš @Godrak, Vojtěch Bubník @bubnikv, Lukáš Matěna @lukasmatena, Enrico Turri @enricoturri1966, Filip Sykala @Jony01, Lukáš Hejl @hejllukas, Tomáš Mészáros @tamasmeszaros
///|/ Copyright (c) 2016 Sakari Kapanen @Flannelhead
///|/ Copyright (c) Slic3r 2013 - 2016 Alessandro Ranellucci @alranel
///|/
///|/ ported from lib/Slic3r/ExPolygon.pm:
///|/ Copyright (c) Prusa Research 2017 - 2022 Vojtěch Bubník @bubnikv
///|/ Copyright (c) Slic3r 2011 - 2014 Alessandro Ranellucci @alranel
///|/ Copyright (c) 2012 Mark Hindess
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_ExPolygon_hpp_
#define slic3r_ExPolygon_hpp_

#include <vector>

#include "libslic3r.h"
#include "Point.hpp"
#include "Polygon.hpp"
#include "Polyline.hpp"

namespace Slic3r {

class Polyline;
class ThickPolyline;
using Polylines = std::vector<Polyline>;
using ThickPolylines = std::vector<ThickPolyline>;
class ExPolygon;
using ExPolygons = std::vector<ExPolygon>;

class ExPolygon
{
public:
    ExPolygon() = default;
    ExPolygon(const ExPolygon &other) = default;
    ExPolygon(ExPolygon &&other) = default;
    explicit ExPolygon(const Polygon &contour) : contour(contour) {}
    explicit ExPolygon(Polygon &&contour) : contour(std::move(contour)) {}
    explicit ExPolygon(const Points &contour) : contour(contour) {}
    explicit ExPolygon(Points &&contour) : contour(std::move(contour)) {}
    explicit ExPolygon(const Polygon &contour, const Polygon &hole) : contour(contour) { holes.emplace_back(hole); }
    explicit ExPolygon(const Polygon &contour, const Polygons &holes) : contour(contour), holes(holes) {}
    explicit ExPolygon(Polygon &&contour, Polygon &&hole) : contour(std::move(contour)) {
        holes.emplace_back(std::move(hole));
    }
    explicit ExPolygon(const Points &contour, const Points &hole) : contour(contour) { holes.emplace_back(hole); }
    explicit ExPolygon(Points &&contour, Polygon &&hole) : contour(std::move(contour)) {
        holes.emplace_back(std::move(hole));
    }
    ExPolygon(std::initializer_list<Point> contour) : contour(contour) {}
    ExPolygon(std::initializer_list<Point> contour, std::initializer_list<Point> hole)
        : contour(contour), holes({hole}) {}

    ExPolygon& operator=(const ExPolygon &other) = default;
    ExPolygon& operator=(ExPolygon &&other) = default;

    Polygon  contour; //CCW
    Polygons holes; //CW

    void clear() { contour.points.clear(); holes.clear(); }
    void scale(double factor);
    void scale(double factor_x, double factor_y);
    void translate(double x, double y) { this->translate(Point(coord_t(x), coord_t(y))); }
    void translate(const Point &vector);
    void rotate(double angle);
    void rotate(double angle, const Point &center);
    double area() const;
    bool empty() const { return contour.points.empty(); }
    bool is_valid() const;
    void douglas_peucker(double tolerance);

    // Contains the line / polyline / polylines etc COMPLETELY.
    bool contains(const Line &line) const;
    bool contains(const Polyline &polyline) const;
    bool contains(const Polylines &polylines) const;
    bool contains(const Point &point, bool border_result = true) const;
    // Approximate on boundary test.
    bool on_boundary(const Point &point, double eps) const;
    // Projection of a point onto the polygon.
    Point point_projection(const Point &point) const;

    // Does this expolygon overlap another expolygon?
    // Either the ExPolygons intersect, or one is fully inside the other,
    // and it is not inside a hole of the other expolygon.
    // The test may not be commutative if the two expolygons touch by a boundary only,
    // see unit test SCENARIO("Clipper diff with polyline", "[Clipper]").
    // Namely expolygons touching at a vertical boundary are considered overlapping, while expolygons touching
    // at a horizontal boundary are NOT considered overlapping.
    bool overlaps(const ExPolygon &other) const;

    void douglas_peucker(coord_t tolerance);
    void simplify_p(coord_t tolerance, Polygons &polygons) const;
    Polygons simplify_p(coord_t tolerance) const;
    ExPolygons simplify(coord_t tolerance) const;
    void simplify(coord_t tolerance, ExPolygons &expolygons) const;
    void remove_point_too_close(const coord_t tolerance);
    void medial_axis(double max_width, double min_width, ThickPolylines &polylines) const;
    void medial_axis(double max_width, double min_width, Polylines &polylines) const;
    Lines lines() const;

    // Number of contours (outer contour with holes).
    size_t   		num_contours() const { return this->holes.size() + 1; }
    Polygon& 		contour_or_hole(size_t idx) 		{ return (idx == 0) ? this->contour : this->holes[idx - 1]; }
    const Polygon& 	contour_or_hole(size_t idx) const 	{ return (idx == 0) ? this->contour : this->holes[idx - 1]; }

#ifdef _DEBUGINFO
#pragma UNOPTIMIZE
    void assert_valid() const {
        contour.assert_valid();
        assert(contour.is_counter_clockwise());
        for (const Polygon &hole : holes) {
            hole.assert_valid();
            assert(hole.is_clockwise());
        }
    }
    // to create a cpp multipoint to create test units.
    std::string to_debug_string();
#else
    void assert_valid() const {}
#endif
};

bool operator==(const ExPolygon &lhs, const ExPolygon &rhs);
bool operator!=(const ExPolygon &lhs, const ExPolygon &rhs);

size_t count_points(const ExPolygons &expolys);
size_t count_points(const ExPolygon &expoly);
// Count a nuber of polygons stored inside the vector of expolygons.
// Useful for allocating space for polygons when converting expolygons to polygons.
size_t number_polygons(const ExPolygons &expolys);

ExPolygon to_expolygon(const Polygon &other);
ExPolygon to_expolygon(Polygon &&other);
ExPolygons convert_to_expolygons(const Polygons &other);
ExPolygons for_union(const ExPolygons &ex1, const ExPolygons &ex2);

Lines to_lines(const ExPolygon &src);
Lines to_lines(const ExPolygons &src);
// Line is from point index(see to_points) to next point.
// Next point of last point in polygon is first polygon point.
Linesf to_linesf(const ExPolygons &src, uint32_t count_lines = 0);
Linesf to_unscaled_linesf(const ExPolygons &src);
Points contours_to_points(const ExPolygons &src);
Points to_points(const ExPolygons &src);
Points to_points(const ExPolygon &expoly);

Polylines to_polylines(const ExPolygon &src);
Polylines to_polylines(const ExPolygons &src);
Polylines to_polylines(ExPolygon &&src);
Polylines to_polylines(ExPolygons &&src);

Polygons to_polygons(const ExPolygon &src);
Polygons to_polygons(const ExPolygons &src);
ConstPolygonPtrs to_polygon_ptrs(const ExPolygon &src);
ConstPolygonPtrs to_polygon_ptrs(const ExPolygons &src);
Polygons to_polygons(ExPolygon &&src);
Polygons to_polygons(ExPolygons &&src);
ExPolygons to_expolygons(const Polygons &polys);
ExPolygons to_expolygons(Polygons &&polys);

void translate(ExPolygons &expolys, const Point &p);
void polygons_append(Polygons &dst, const ExPolygon &src);
void polygons_append(Polygons &dst, const ExPolygons &src);
void polygons_append(Polygons &dst, ExPolygon &&src);
void polygons_append(Polygons &dst, ExPolygons &&src);
void expolygons_append(ExPolygons &dst, const Polygons &src);
void expolygons_append(ExPolygons &dst, Polygons &&src);
void expolygons_append(ExPolygons &dst, const ExPolygons &src);
void expolygons_append(ExPolygons &dst, ExPolygons &&src);
void expolygons_rotate(ExPolygons &expolys, double angle);
bool expolygons_contain(const ExPolygons &expolys, const Point &pt, bool border_result = true);

// expolygons_simplify will simplify the geometry via douglaspeuker.
void expolygons_simplify(ExPolygons &expolys, coord_t tolerance);

// Do expolygons match? If they match, they must have the same topology,
// however their contours may be rotated.
bool expolygons_match(const ExPolygon &l, const ExPolygon &r);

BoundingBox get_extents(const ExPolygon &expolygon);
BoundingBox get_extents(const ExPolygons &expolygons);
BoundingBox get_extents_rotated(const ExPolygon &poly, double angle);
BoundingBox get_extents_rotated(const ExPolygons &polygons, double angle);
std::vector<BoundingBox> get_extents_vector(const ExPolygons &polygons);

// Test for duplicate points. The points are copied, sorted and checked for duplicates globally.
bool has_duplicate_points(const ExPolygon &expoly);
bool has_duplicate_points(const ExPolygons &expolys);

// remove any point that are at epsilon  (or resolution) 'distance' (douglas_peuckere algo for now) and all polygons that are too small to be valid
// note: in the future, it may limited to removing points that just to close to other ones. If you want to simplify the geomtry, use expolygons_simplify.
// so it remove points that are too close, and may or may not remove colinear points.
void ensure_valid(ExPolygons &expolygons, coord_t resolution = SCALED_EPSILON);
ExPolygons ensure_valid(ExPolygons &&expolygons, coord_t resolution = SCALED_EPSILON);
ExPolygons ensure_valid(coord_t resolution, ExPolygons &&expolygons);
// like ensure_valid but you're sure it won't remove colinear points.
void remove_point_too_close(ExPolygons &expolygons, coord_t resolution = SCALED_EPSILON);
ExPolygons remove_point_too_close(ExPolygons &&expolygons, coord_t resolution = SCALED_EPSILON);
#ifdef _DEBUGINFO
void assert_valid(const ExPolygons &expolygons);
#else
inline void assert_valid(const ExPolygons &expolygons) {}
#endif

// Return True when erase some otherwise False.
bool remove_same_neighbor(ExPolygons &expolys);

bool remove_sticks(ExPolygon &poly);
// Keeps only the ExPolygon whose contour has the largest positive signed area (the first one on a tie).
// Leaves the polygons unchanged when none has a positive area; does nothing for 0 or 1 polygon.
void keep_largest_contour_only(ExPolygons &polygons);

double      area(const ExPolygon &poly);
double      area(const ExPolygons &polys);

// Removes all expolygons smaller than min_area and also removes all holes smaller than min_area
bool        remove_small_and_small_holes(ExPolygons &expolygons, double min_area);

} // namespace Slic3r

// start Boost
#include <boost/polygon/polygon.hpp>
namespace boost { namespace polygon {
    template <>
        struct polygon_traits<Slic3r::ExPolygon> {
        typedef coord_t coordinate_type;
        typedef Slic3r::Points::const_iterator iterator_type;
        typedef Slic3r::Point point_type;

        // Get the begin iterator
        static inline iterator_type begin_points(const Slic3r::ExPolygon& t) {
            return t.contour.points.begin();
        }

        // Get the end iterator
        static inline iterator_type end_points(const Slic3r::ExPolygon& t) {
            return t.contour.points.end();
        }

        // Get the number of sides of the polygon
        static inline std::size_t size(const Slic3r::ExPolygon& t) {
            return t.contour.points.size();
        }

        // Get the winding direction of the polygon
        static inline winding_direction winding(const Slic3r::ExPolygon& /* t */) {
            return unknown_winding;
        }
    };

    template <>
    struct polygon_mutable_traits<Slic3r::ExPolygon> {
        //expects stl style iterators
        template <typename iT>
        static inline Slic3r::ExPolygon& set_points(Slic3r::ExPolygon& expolygon, iT input_begin, iT input_end) {
            expolygon.contour.points.assign(input_begin, input_end);
            // skip last point since Boost will set last point = first point
            assert(expolygon.contour.points.front() == expolygon.contour.points.back());
            expolygon.contour.points.pop_back();
            return expolygon;
        }
    };
    
    
    template <>
    struct geometry_concept<Slic3r::ExPolygon> { typedef polygon_with_holes_concept type; };

    template <>
    struct polygon_with_holes_traits<Slic3r::ExPolygon> {
        typedef Slic3r::Polygons::const_iterator iterator_holes_type;
        typedef Slic3r::Polygon hole_type;
        static inline iterator_holes_type begin_holes(const Slic3r::ExPolygon& t) {
            return t.holes.begin();
        }
        static inline iterator_holes_type end_holes(const Slic3r::ExPolygon& t) {
            return t.holes.end();
        }
        static inline unsigned int size_holes(const Slic3r::ExPolygon& t) {
            return (int)t.holes.size();
        }
    };

    template <>
    struct polygon_with_holes_mutable_traits<Slic3r::ExPolygon> {
         template <typename iT>
         static inline Slic3r::ExPolygon& set_holes(Slic3r::ExPolygon& t, iT inputBegin, iT inputEnd) {
              t.holes.assign(inputBegin, inputEnd);
              return t;
         }
    };
    
    //first we register CPolygonSet as a polygon set
    template <>
    struct geometry_concept<Slic3r::ExPolygons> { typedef polygon_set_concept type; };

    //next we map to the concept through traits
    template <>
    struct polygon_set_traits<Slic3r::ExPolygons> {
        typedef coord_t coordinate_type;
        typedef Slic3r::ExPolygons::const_iterator iterator_type;
        typedef Slic3r::ExPolygons operator_arg_type;

        static inline iterator_type begin(const Slic3r::ExPolygons& polygon_set) {
            return polygon_set.begin();
        }

        static inline iterator_type end(const Slic3r::ExPolygons& polygon_set) {
            return polygon_set.end();
        }

        //don't worry about these, just return false from them
        static inline bool clean(const Slic3r::ExPolygons& /* polygon_set */) { return false; }
        static inline bool sorted(const Slic3r::ExPolygons& /* polygon_set */) { return false; }
    };

    template <>
    struct polygon_set_mutable_traits<Slic3r::ExPolygons> {
        template <typename input_iterator_type>
        static inline void set(Slic3r::ExPolygons& expolygons, input_iterator_type input_begin, input_iterator_type input_end) {
            expolygons.assign(input_begin, input_end);
        }
    };
} }
// end Boost

#endif
