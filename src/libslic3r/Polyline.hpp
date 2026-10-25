///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ Copyright (c) Prusa Research 2016 - 2023 Tomáš Mészáros @tamasmeszaros, Pavel Mikuš @Godrak, Vojtěch Bubník @bubnikv, Lukáš Hejl @hejllukas, Lukáš Matěna @lukasmatena, Oleksandra Iushchenko @YuSanka, Enrico Turri @enricoturri1966
///|/ Copyright (c) Slic3r 2013 - 2016 Alessandro Ranellucci @alranel
///|/
///|/ ported from lib/Slic3r/Polyline.pm:
///|/ Copyright (c) Prusa Research 2018 Vojtěch Bubník @bubnikv
///|/ Copyright (c) Slic3r 2011 - 2014 Alessandro Ranellucci @alranel
///|/ Copyright (c) 2012 Mark Hindess
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Polyline_hpp_
#define slic3r_Polyline_hpp_

#include <memory>
#include <string>
#include <vector>

#include "ContainerUtils.hpp"
#include "Geometry/ArcFitter.hpp"
#include "Geometry/ArcWelder.hpp"
#include "libslic3r.h"
#include "Line.hpp"
#include "MultiPoint.hpp"
#include "Point.hpp"

/*
Polyline types store scaled points and expose geometry operations. ArcPolyline
also retains segment radii and optional point Z offsets; translating its points
preserves arc geometry and permits a single point used as a deposit position.
*/
namespace Slic3r {

class Polyline;
class ThickPolyline;
//class PolylineOrArc;
class ArcPolyline;
namespace ApiInternal { struct ArcPolylineAccess; }
typedef std::vector<Polyline> Polylines;
typedef std::vector<ThickPolyline> ThickPolylines;
//typedef std::vector<PolylineOrArc> PolylinesOrArcs;
typedef std::vector<ArcPolyline> ArcPolylines;

class Polyline : public MultiPoint {
public:
    Polyline() = default;
    Polyline(const Polyline& other) : MultiPoint(other.points) {}
    Polyline(Polyline&& other) : MultiPoint(std::move(other.points)) {}
    Polyline(std::initializer_list<Point> list) : MultiPoint(list) {}
    explicit Polyline(const Point& p1, const Point& p2) { points.reserve(2); points.emplace_back(p1); points.emplace_back(p2); }
    explicit Polyline(const Points& points) : MultiPoint(points) {}
    explicit Polyline(Points&& points) : MultiPoint(std::move(points)) {}
    Polyline& operator=(const Polyline& other) { points = other.points; return *this; }
    Polyline& operator=(Polyline&& other) { points = std::move(other.points); return *this; }
    bool operator==(const Polyline& other) const { return points == other.points; }
    bool operator!=(const Polyline& other) const { return points != other.points; }
    static Polyline new_scale(const std::vector<Vec2d> &points) {
        Polyline pl;
        pl.points.reserve(points.size());
        for (const Vec2d &pt : points)
            pl.points.emplace_back(Point::new_scale(pt(0), pt(1)));
        return pl;
    }

    void append(const Point& point) { this->points.push_back(point); }
    void append(const Points& src) { this->append(src.begin(), src.end()); }
    void append(const Points::const_iterator &begin, const Points::const_iterator &end) {
        assert(!begin->coincides_with_epsilon(this->back()));
        this->points.insert(this->points.end(), begin, end);
    }
    void append(Points &&src)
    {
        if (this->points.empty()) {
            this->points = std::move(src);
        } else if (this->back().coincides_with_epsilon(src.front())) {
            this->points.insert(this->points.end(), src.begin() + 1, src.end());
            src.clear();
        } else {
            this->points.insert(this->points.end(), src.begin(), src.end());
            src.clear();
        }
    }
    void append(const Polyline& src)
    {
        assert(this->empty() || this->back().coincides_with_epsilon(src.front()));
        points.insert(points.end(), src.points.begin() + 1, src.points.end());
    }

    void append(Polyline&& src)
    {
        if (this->points.empty()) {
            this->points = std::move(src.points);
        } else {
            assert(this->back().coincides_with_epsilon(src.front()));
            this->points.insert(this->points.end(), src.points.begin() + 1, src.points.end());
            src.points.clear();
        }
    }
  
    Point& operator[](Points::size_type idx) { return this->points[idx]; }
    const Point& operator[](Points::size_type idx) const { return this->points[idx]; }

    double length() const;
    const Point &last_point() const { return this->points.back(); }
    const Point& leftmost_point() const;
    Lines lines() const;

    void clip_end(distf_t distance);
    void clip_start(distf_t distance);
    void extend_end(distf_t distance);
    void extend_start(distf_t distance);
    Points equally_spaced_points(distf_t distance) const;
    void simplify(coordf_t tolerance);
//    template <class T> void simplify_by_visibility(const T &area);
    void split_at(const Point &point, Polyline* p1, Polyline* p2) const;
    // reduce the length of this polyline at dist. return the rest after the distance.
    Polyline split_at(distf_t dist);
    bool is_straight() const;
    bool is_closed() const { return this->points.size() > 2 && this->points.front() == this->points.back(); }
    bool is_polyline() const override { return true; }; // reflection

    using iterator = Points::iterator;
    using const_iterator = Points::const_iterator;
};

//inline bool operator==(const Polyline &lhs, const Polyline &rhs) { return lhs.points == rhs.points; }
//inline bool operator!=(const Polyline &lhs, const Polyline &rhs) { return lhs.points != rhs.points; }

extern BoundingBox get_extents(const Polyline& polyline);
extern BoundingBox get_extents(const Polylines& polylines);

Polyline reverse_polyline(const Polyline &polyline);

// Return True when erase some otherwise False.
bool remove_same_neighbor(Polyline &polyline);
bool remove_same_neighbor(Polylines &polylines);
// remove any point that are at epsilon  (or resolution) 'distance' (douglas_peuckere algo for now) and all polylines that are too small to be valid
void ensure_valid(Polylines &polylines, coord_t resolution = SCALED_EPSILON);
Polylines ensure_valid(Polylines &&polylines, coord_t resolution = SCALED_EPSILON);
void ensure_valid(Polyline &polyline, coord_t resolution = SCALED_EPSILON);

double total_length(const Polylines &polylines);

Lines to_lines(const Polyline &poly);
Lines to_lines(const Polylines &polys);
Polylines to_polylines(const std::vector<Points> &paths);
Polylines to_polylines(std::vector<Points> &&paths);
void polylines_append(Polylines &dst, const Polylines &src);
void polylines_append(Polylines &dst, Polylines &&src);

// Merge polylines at their respective end points.
// dst_first: the merge point is at dst.begin() or dst.end()?
// src_first: the merge point is at src.begin() or src.end()?
// The orientation of the resulting polyline is unknown, the output polyline may start
// either with src piece or dst piece.
template<typename PointsType>
inline void polylines_merge(PointsType &dst, bool dst_first, PointsType &&src, bool src_first)
{
    if (dst_first) {
        if (src_first)
            std::reverse(dst.begin(), dst.end());
        else
            std::swap(dst, src);
    } else if (! src_first)
        std::reverse(src.begin(), src.end());
    // Merge src into dst.
    append(dst, std::move(src));
}

const Point& leftmost_point(const Polylines &polylines);

bool remove_degenerate(Polylines &polylines);

#ifdef _DEBUGINFO
void assert_valid(const Polylines &polylines);
#else
inline void assert_valid(const Polylines &polylines) {}
#endif

// Returns index of a segment of a polyline and foot point of pt on polyline.
//std::pair<int, Point> foot_pt(const Points &polyline, const Point &pt);

/// ThickPolyline : a polyline with a width for each point
/// This class has a vector of coordf_t, it must be the same size as points.
/// it's used to store the size of the line at this point.
/// Also, the endpoint let us know if the front() and back() of the polyline 
/// join something or is a dead-end.
class ThickPolyline {
public:
    Points                points;
    /// width size must be == point size
    std::vector<coord_t>  points_width;
    /// if true => it's an endpoint, if false it join an other ThickPolyline. first is at front(), second is at back()
    std::pair<bool, bool> endpoints;
    //if it's important to begin at a specific bit.
    enum StartPos : int8_t { tpspBegin = -1, tpspBoth = 0, tpspEnd = 1 };
    StartPos              start_at  = tpspBoth;

    ThickPolyline() : endpoints(std::make_pair(false, false)) {}
    ThickLines thicklines() const;

    const Point& front()        const { assert(points.size() == points_width.size()); return this->points.front(); }
    const Point& back()         const { assert(points.size() == points_width.size()); return this->points.back(); }
    size_t       size()         const { assert(points.size() == points_width.size()); return this->points.size(); }
    bool         is_valid()     const;
    bool         empty()        const { assert(points.size() == points_width.size()); return this->points.empty(); }
    double       length()       const;

    void         clear() { this->points.clear(); this->points_width.clear(); }

    void reverse();

    void clip_end(distf_t distance);
    void extend_end(distf_t distance);
    void extend_start(distf_t distance);

    // Make this closed ThickPolyline starting in the specified index.
    // Be aware that this method can be applicable just for closed ThickPolyline.
    // On open ThickPolyline make no effect.
    void start_at_index(int index);

};

ThickPolylines to_thick_polylines(Polylines &&polylines, const coordf_t width);

/// concatenate poylines if possible and refresh the endpoints
void concatThickPolylines(ThickPolylines& polylines);

class Polyline3 : public MultiPoint3
{
public:
    double length() const;
    Lines3 lines() const;
};

typedef std::vector<Polyline3> Polylines3;

class ArcPolyline
{
    friend struct ApiInternal::ArcPolylineAccess;
protected:
    // each segment is strait if it's radius ==0 (orientation should be unknown in this case)
    // radius is negative if the arc betweent he two point is the longest of the two. it's positive if it's the shortest.
    // the sign of the radius and the orientation are two different way to get the same information. They MUST be in synch.
    // note: first Segment in Path is always "strait", as it's the starting point of the following segment. 
    Geometry::ArcWelder::Path m_path;
    //std::vector<Point> m_points;
    std::unique_ptr<std::vector<coord_t>> m_z_offset;
    //NOTe: maybe later, the vector of thign can be extended by many point-property, not only z & Segment.
    //bool cache_valid = true; // cache
    bool m_only_strait = true; // cache
    //note: i don't keep the polyline, as it's easy to reconstruct from arc (to_polyline).
    // if it creates a big preformance problem, then add a cache here.

    static Geometry::ArcWelder::Path _from_polyline(const Points &poly);
    static Geometry::ArcWelder::Path _from_polyline(std::initializer_list<Point> poly);
public:
    static constexpr coord_t INVALID_COORD = std::numeric_limits<coord_t>::max();

#ifdef _DEBUG
    bool is_3D = false; // to deactivate assert about epsilon dist
#endif
    ArcPolyline(){};
    ArcPolyline(const ArcPolyline &other);
    ArcPolyline(ArcPolyline &&)      = default;
    ArcPolyline(const Polyline &other) : m_path(_from_polyline(other.points)) {}
    ArcPolyline(const Points &other) : m_path(_from_polyline(other)) {}
    ArcPolyline(const Geometry::ArcWelder::Path &other);
    ArcPolyline &operator=(const ArcPolyline &other);
    ArcPolyline &operator=(ArcPolyline &&) = default;

    void append(const Point &point);
    void append_before(const Point &point);
    // Only use this append if and only if you're sure that the previous point is the same as one from another good ArcPolyline. First point need to be added via append(Point).
    void append(const Geometry::ArcWelder::Segment &to_copy_arc);
    void append(const Points &src);
    void append(Points &&src);
    void append(const Points::const_iterator &begin, const Points::const_iterator &end);
    void append(const ArcPolyline &src);
    void append(ArcPolyline &&src);
    void clear();
    void swap(ArcPolyline &other);
    void reverse();

    // multipoint methods
    const Point &front() const { return m_path.front().point; }
    const Point &middle() const { return m_path[m_path.size() / 2].point; }
    const Point &back() const { return m_path.back().point; }
    bool         empty() const { return m_path.empty(); }
    bool         is_valid() const;
    bool         is_closed() const { return this->m_path.front().point == this->m_path.back().point; }

    bool         has_arc() const;
    // point count in the path
    size_t                              size() const { return m_path.size(); }
    const Geometry::ArcWelder::Path &   get_arc() const { return m_path; }
    // get the point at index i in the path (i<size())
    const Point &                       get_point(size_t i) const { return m_path[i].point; }
    const Geometry::ArcWelder::Segment &get_arc(size_t i) const { return m_path[i]; }

    //works on points only (be careful)
    bool                  split_at_index(const size_t index, ArcPolyline &p1, ArcPolyline &p2) const;
    void                  pop_front();
    void                  pop_back();
    // need some work to work better on arc
    void                  set_front(const Point &p);
    void                  set_back(const Point &p);
    // this one give you the index of the nearest point, or -1 if none is at epsilon.
    // if on an arc, you may want to call foot_pt to have the projection
    int                   find_point(const Point &point, coordf_t epsilon) const;
    bool                  has_z_offset() const { return m_z_offset != nullptr; }
    coord_t               z_offset(size_t idx) const;
    // set z_offset. !has_z_offset, then the array is created with 0 for all point beforehand.
    void                  set_z_offset(size_t idx, coord_t z_offset);

    // Works on points & arc
    distf_t               length() const;
    bool                  at_least_length(distf_t length) const;
    std::pair<int, Point> foot_pt(const Point &pt) const;
    void                  split_at(Point &point, ArcPolyline &p1, ArcPolyline &p2) const;
    void                  split_at(distf_t distance, ArcPolyline &p1, ArcPolyline &p2) const;
    void                  clip_start(distf_t dist);
    void                  clip_end(distf_t dist);
    Polyline              to_polyline(coord_t deviation = 0) const;
    // Shift all stored points and arc centers, including empty and one-point paths.
    void                  translate(const Vector &vector);
    void                  rotate(double angle); // to test for arc, but should be okay
    Point                 get_point_from_begin(distf_t distance) const;
    Point                 get_point_from_end(distf_t distance) const;

    // !!!!!!!!!!!
    // == following public methods will delete z_offset, do not use them if you have it and want to keep it ===
    // !!!!!!!!!!!

    // TODO == > remove from here, use them as algorithm that creates a new ArcPolygon.

    // douglas_peuker and create arc if with_fitting_arc (don't touch the current arcs, only try in-between)
    void make_arc(ArcFittingType with_fitting_arc, coordf_t tolerance, double fit_percent_tolerance);
    // remove strait segemnts that are too near each other, and will overlaod the firmware. Return the buffer lines it still uses a t the end.
    int simplify_straits(coordf_t min_tolerance, coordf_t min_point_distance, coordf_t mean_dist_per_line, const int buffer_size, const int buffer_init);
    void simplify_straits(const coordf_t min_tolerance, const coordf_t min_point_distance);

    // remove points that are too near each other, and return false if the whole path is too small
    bool normalize();

protected:
    // works on points only
    int          find_point(const Point &point) const;
    std::vector<coord_t>& z_offsets_raw_mutable();
    std::vector<coord_t>& z_offsets_mutable();
};

Polylines to_polylines(const ArcPolylines &polys_or_arcs, coord_t deviation = 0);

} //namespace slic3r

#endif
