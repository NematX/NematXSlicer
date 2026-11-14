///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ Copyright (c) Prusa Research 2016 - 2023 Vojtěch Bubník @bubnikv, Lukáš Matěna @lukasmatena, Lukáš Hejl @hejllukas
///|/ Copyright (c) Slic3r 2013 - 2016 Alessandro Ranellucci @alranel
///|/ Copyright (c) 2015 Maksim Derbasov @ntfshard
///|/ Copyright (c) 2014 Petr Ledvina @ledvinap
///|/
///|/ ported from lib/Slic3r/ExPolygon.pm:
///|/ Copyright (c) Prusa Research 2017 - 2022 Vojtěch Bubník @bubnikv
///|/ Copyright (c) Slic3r 2011 - 2014 Alessandro Ranellucci @alranel
///|/ Copyright (c) 2012 Mark Hindess
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "ExPolygon.hpp"

#include <algorithm>
#include <cassert>
#include <iterator>
#include <list>

#include <ankerl/unordered_dense.h>

#include "BoundingBox.hpp"
#include "ClipperUtils.hpp"
#include "Exception.hpp"
#include "Geometry/MedialAxis.hpp"
#include "Line.hpp"
#include "PointUtils.hpp"
#include "Polygon.hpp"
#include "SVG.hpp"

/* ExPolygon keeps an exterior and its holes together. Mesh-ring normalization
   preserves this grouping when exact degeneracies collapse or split a contour.
   General simplification still uses the caller's existing resolution.

       ensure_valid -> normalize_mesh_expolygons -> normalize_mesh_polygons
                    -> expolygons_simplify -> normalize_mesh_expolygons
*/

namespace Slic3r {

void normalize_mesh_expolygons(ExPolygons &expolys)
{
    std::vector<ExPolygons> groups;
    groups.reserve(expolys.size());
    size_t count = 0;
    for (ExPolygon &source : expolys) {
        Polygons contours(1);
        contours.front().points.swap(source.contour.points);
        normalize_mesh_polygons(contours);
        if (contours.empty())
            continue;
        normalize_mesh_polygons(source.holes);
        if (contours.size() == 1) {
            ExPolygons group(1);
            group.front().contour.points.swap(contours.front().points);
            group.front().holes.swap(source.holes);
            groups.emplace_back(std::move(group));
        } else {
            // Use Clipper directly: the general surface wrappers also remove
            // narrow areas by tolerance, which exact normalization must retain.
            ClipperLib::Clipper clipper(ClipperLib::ioPreserveCollinear);
            clipper.CleanDistance(0.);
            clipper.AddPaths(ClipperUtils::PolygonsProvider(contours), ClipperLib::ptSubject, true);
            clipper.AddPaths(ClipperUtils::PolygonsProvider(source.holes), ClipperLib::ptClip, true);
            ClipperLib::PolyTree tree;
            if (!clipper.Execute(ClipperLib::ctDifference, tree, ClipperLib::pftNonZero))
                throw Slic3r::RuntimeError("Cannot assign holes to normalized mesh contours.");
            std::vector<ClipperLib::PolyNode *> exteriors;
            for (ClipperLib::PolyNode *node = tree.GetFirst(); node != nullptr; node = node->GetNext())
                if (!node->IsHole())
                    exteriors.push_back(node);
            ExPolygons surfaces(exteriors.size());
            for (size_t i = 0; i < exteriors.size(); ++i) {
                ClipperLib::PolyNode &node = *exteriors[i];
                surfaces[i].contour.points.swap(node.Contour);
                surfaces[i].holes.resize(node.ChildCount());
                for (size_t j = 0; j < node.Childs.size(); ++j)
                    surfaces[i].holes[j].points.swap(node.Childs[j]->Contour);
            }
            groups.emplace_back(std::move(surfaces));
        }
        count += groups.back().size();
    }
    // Allocate once and swap coordinates, bypassing Polygon's tolerant move
    // constructor for both untouched contours and their normalized holes.
    ExPolygons normalized(count);
    size_t index = 0;
    for (ExPolygons &group : groups)
        for (ExPolygon &surface : group) {
            normalized[index].contour.points.swap(surface.contour.points);
            normalized[index].holes.swap(surface.holes);
            ++index;
        }
    expolys.swap(normalized);
}

bool operator==(const ExPolygon &lhs, const ExPolygon &rhs)
{
    return lhs.contour == rhs.contour && lhs.holes == rhs.holes;
}

bool operator!=(const ExPolygon &lhs, const ExPolygon &rhs)
{
    return lhs.contour != rhs.contour || lhs.holes != rhs.holes;
}

size_t count_points(const ExPolygons &expolys)
{
    size_t n_points = 0;
    for (const auto &expoly : expolys) {
        n_points += expoly.contour.points.size();
        for (const auto &hole : expoly.holes) {
            n_points += hole.points.size();
        }
    }
    return n_points;
}

size_t count_points(const ExPolygon &expoly)
{
    size_t n_points = expoly.contour.points.size();
    for (const auto &hole : expoly.holes) {
        n_points += hole.points.size();
    }
    return n_points;
}

size_t number_polygons(const ExPolygons &expolys)
{
    size_t n_polygons = 0;
    for (const ExPolygon &ex : expolys) {
        n_polygons += ex.holes.size() + 1;
    }
    return n_polygons;
}

ExPolygon to_expolygon(const Polygon &other)
{
    assert(other.is_counter_clockwise());
    ExPolygon ex;
    ex.contour = other;
    return ex;
}

ExPolygon to_expolygon(Polygon &&other)
{
    assert(other.is_counter_clockwise());
    ExPolygon ex;
    ex.contour = std::move(other);
    return ex;
}

ExPolygons convert_to_expolygons(const Polygons &other)
{
    ExPolygons exs;
    for (size_t i = 0; i < other.size(); i++) {
        if (other[i].is_counter_clockwise()) {
            exs.emplace_back(other[i]);
        } else {
            assert(!exs.empty());
            exs.back().holes.emplace_back(other[i]);
        }
    }
    return exs;
}

ExPolygons for_union(const ExPolygons &ex1, const ExPolygons &ex2)
{
    ExPolygons out = ex1;
    append(out, ex2);
    return out;
}

Lines to_lines(const ExPolygon &src)
{
    Lines lines;
    lines.reserve(count_points(src));
    for (size_t i = 0; i <= src.holes.size(); ++i) {
        const Polygon &poly = (i == 0) ? src.contour : src.holes[i - 1];
        for (Points::const_iterator it = poly.points.begin(); it != poly.points.end() - 1; ++it) {
            lines.push_back(Line(*it, *(it + 1)));
        }
        lines.push_back(Line(poly.points.back(), poly.points.front()));
    }
    return lines;
}

Lines to_lines(const ExPolygons &src)
{
    Lines lines;
    lines.reserve(count_points(src));
    for (ExPolygons::const_iterator it_expoly = src.begin(); it_expoly != src.end(); ++it_expoly) {
        for (size_t i = 0; i <= it_expoly->holes.size(); ++ i) {
            const Points &points = ((i == 0) ? it_expoly->contour : it_expoly->holes[i - 1]).points;
            for (Points::const_iterator it = points.begin(); it != points.end()-1; ++it) {
                lines.push_back(Line(*it, *(it + 1)));
            }
            lines.push_back(Line(points.back(), points.front()));
        }
    }
    return lines;
}

Linesf to_linesf(const ExPolygons &src, uint32_t count_lines)
{
    assert(count_lines == 0 || count_lines == count_points(src));
    if (count_lines == 0) count_lines = count_points(src);
    Linesf lines;
    lines.reserve(count_lines);
    Vec2d prev_pd;
    auto to_lines = [&lines, &prev_pd](const Points &pts) {
        assert(pts.size() >= 3);
        if (pts.size() < 2) return;
        bool is_first = true;
        for (const Point &p : pts) {
            Vec2d pd = p.cast<double>();
            if (is_first) is_first = false;
            else lines.emplace_back(prev_pd, pd);
            prev_pd = pd;
        }
        lines.emplace_back(prev_pd, pts.front().cast<double>());
    };
    for (const ExPolygon& expoly : src) {
        to_lines(expoly.contour.points);
        for (const Polygon &hole : expoly.holes) {
            to_lines(hole.points);
        }
    }
    assert(lines.size() == count_lines);
    return lines;
}

Linesf to_unscaled_linesf(const ExPolygons &src)
{
    Linesf lines;
    lines.reserve(count_points(src));
    for (ExPolygons::const_iterator it_expoly = src.begin(); it_expoly != src.end(); ++it_expoly) {
        for (size_t i = 0; i <= it_expoly->holes.size(); ++ i) {
            const Points &points = ((i == 0) ? it_expoly->contour : it_expoly->holes[i - 1]).points;
            Vec2d unscaled_a = unscale_p(points.front());
            Vec2d unscaled_b = unscaled_a;
            for (Points::const_iterator it = points.begin() + 1; it != points.end(); ++it) {
                unscaled_b = unscale_p(*(it));
                lines.push_back(Linef(unscaled_a, unscaled_b));
                unscaled_a = unscaled_b;
            }
            lines.push_back(Linef(unscaled_a, unscale_p(points.front())));
        }
    }
    return lines;
}

Points contours_to_points(const ExPolygons &src)
{
    Points points;
    size_t count = 0;
    for (const ExPolygon &expolygon : src) {
        count += expolygon.contour.points.size();
    }
    points.reserve(count);
    for (const ExPolygon &expolygon : src) {
        append(points, expolygon.contour.points);
    }
    return points;
}

Points to_points(const ExPolygons &src)
{
    Points points;
    size_t count = count_points(src);
    points.reserve(count);
    for (const ExPolygon &expolygon : src) {
        append(points, expolygon.contour.points);
        for (const Polygon &hole : expolygon.holes) {
            append(points, hole.points);
        }
    }
    return points;
}

Points to_points(const ExPolygon &expoly)
{
    Points out;
    out.reserve(count_points(expoly));
    append(out, expoly.contour.points);
    for (const Polygon &hole : expoly.holes) {
        append(out, hole.points);
    }
    return out;
}

Polygons to_polygons(const ExPolygon &src)
{
    assert(src.contour.is_counter_clockwise());
    assert(src.holes.empty() || src.holes.front().is_clockwise());
    Polygons polygons;
    polygons.reserve(src.holes.size() + 1);
    polygons.push_back(src.contour);
    polygons.insert(polygons.end(), src.holes.begin(), src.holes.end());
    return polygons;
}

Polygons to_polygons(const ExPolygons &src)
{
    // FIXME: put "inside" polygon after the "outside" ones, so the holes of the "outside" don't erase the "inside" contour
    Polygons polygons;
    polygons.reserve(number_polygons(src));
    for (const ExPolygon& ex_poly : src) {
        assert(ex_poly.contour.is_counter_clockwise());
        assert(ex_poly.holes.empty() || ex_poly.holes.front().is_clockwise());
        polygons.push_back(ex_poly.contour);
        polygons.insert(polygons.end(), ex_poly.holes.begin(), ex_poly.holes.end());
    }
#ifdef _DEBUG
    // check hole ordering
    Polygons holes;
    for (size_t i = src.size() - 1; i < src.size(); i--) {
        for (Polygon &hole : holes) {
            // a big hole need to be before than the contour that lie inside.
            assert(!hole.contains(src[i].contour.front()));
        }
        for (Polygon hole : src[i].holes) {
            hole.make_counter_clockwise();
            holes.push_back(std::move(hole));
        }
    }
#endif
    return polygons;
}

ConstPolygonPtrs to_polygon_ptrs(const ExPolygon &src)
{
    assert(src.contour.is_counter_clockwise());
    assert(src.holes.empty() || src.holes.front().is_clockwise());
    ConstPolygonPtrs polygons;
    polygons.reserve(src.holes.size() + 1);
    polygons.emplace_back(&src.contour);
    for (const Polygon &hole : src.holes) {
        polygons.emplace_back(&hole);
    }
    return polygons;
}

ConstPolygonPtrs to_polygon_ptrs(const ExPolygons &src)
{
    ConstPolygonPtrs polygons;
    polygons.reserve(number_polygons(src));
    for (const ExPolygon &expoly : src) {
        assert(expoly.contour.is_counter_clockwise());
        assert(expoly.holes.empty() || expoly.holes.front().is_clockwise());
        polygons.emplace_back(&expoly.contour);
        for (const Polygon &hole : expoly.holes) {
            polygons.emplace_back(&hole);
        }
    }
    return polygons;
}

Polygons to_polygons(ExPolygon &&src)
{
    Polygons polygons;
    polygons.reserve(src.holes.size() + 1);
    polygons.push_back(std::move(src.contour));
    polygons.insert(polygons.end(),
        std::make_move_iterator(src.holes.begin()),
        std::make_move_iterator(src.holes.end()));
    return polygons;
}

Polygons to_polygons(ExPolygons &&src)
{
    Polygons polygons;
    polygons.reserve(number_polygons(src));
    for (ExPolygon& expoly : src) {
        assert(expoly.contour.is_counter_clockwise());
        assert(expoly.holes.empty() || expoly.holes.front().is_clockwise());
        polygons.push_back(std::move(expoly.contour));
        polygons.insert(polygons.end(),
            std::make_move_iterator(expoly.holes.begin()),
            std::make_move_iterator(expoly.holes.end()));
    }
    return polygons;
}

ExPolygons to_expolygons(const Polygons &polys)
{
    ExPolygons ex_polys;
    ex_polys.assign(polys.size(), ExPolygon());
    for (size_t idx = 0; idx < polys.size(); ++idx) {
        assert(polys[idx].is_counter_clockwise());
        ex_polys[idx].contour = polys[idx];
    }
    return ex_polys;
}

ExPolygons to_expolygons(Polygons &&polys)
{
    ExPolygons ex_polys;
    ex_polys.assign(polys.size(), ExPolygon());
    for (size_t idx = 0; idx < polys.size(); ++idx) {
        assert(polys[idx].is_counter_clockwise());
        ex_polys[idx].contour = std::move(polys[idx]);
    }
    return ex_polys;
}

void translate(ExPolygons &expolys, const Point &p)
{
    for (ExPolygon &expoly : expolys) {
        expoly.translate(p);
    }
}

void polygons_append(Polygons &dst, const ExPolygon &src)
{
    assert(src.contour.is_counter_clockwise());
    assert(src.holes.empty() || src.holes.front().is_clockwise());
    dst.reserve(dst.size() + src.holes.size() + 1);
    dst.push_back(src.contour);
    dst.insert(dst.end(), src.holes.begin(), src.holes.end());
}

void polygons_append(Polygons &dst, const ExPolygons &src)
{
    dst.reserve(dst.size() + number_polygons(src));
    for (ExPolygons::const_iterator it = src.begin(); it != src.end(); ++it) {
        assert(it->contour.is_counter_clockwise());
        assert(it->holes.empty() || it->holes.front().is_clockwise());
        dst.push_back(it->contour);
        dst.insert(dst.end(), it->holes.begin(), it->holes.end());
    }
}

void polygons_append(Polygons &dst, ExPolygon &&src)
{
    assert(src.contour.is_counter_clockwise());
    assert(src.holes.empty() || src.holes.front().is_clockwise());
    dst.reserve(dst.size() + src.holes.size() + 1);
    dst.push_back(std::move(src.contour));
    dst.insert(dst.end(),
        std::make_move_iterator(src.holes.begin()),
        std::make_move_iterator(src.holes.end()));
}

void polygons_append(Polygons &dst, ExPolygons &&src)
{
    dst.reserve(dst.size() + number_polygons(src));
    for (ExPolygon& expoly : src) {
        assert(expoly.contour.is_counter_clockwise());
        assert(expoly.holes.empty() || expoly.holes.front().is_clockwise());
        dst.push_back(std::move(expoly.contour));
        dst.insert(dst.end(),
            std::make_move_iterator(expoly.holes.begin()),
            std::make_move_iterator(expoly.holes.end()));
    }
}

void expolygons_append(ExPolygons &dst, const Polygons &src)
{
    for (const Polygon& poly : src) {
        assert(poly.is_counter_clockwise());
        dst.emplace_back(poly);
    }
}

void expolygons_append(ExPolygons &dst, Polygons &&src)
{
    for (Polygon& poly : src) {
        assert(poly.is_counter_clockwise());
        dst.emplace_back(std::move(poly));
    }
}

void expolygons_append(ExPolygons &dst, const ExPolygons &src)
{
    dst.insert(dst.end(), src.begin(), src.end());
}

void expolygons_append(ExPolygons &dst, ExPolygons &&src)
{
    if (dst.empty()) {
        dst = std::move(src);
    } else {
        dst.insert(dst.end(),
            std::make_move_iterator(src.begin()),
            std::make_move_iterator(src.end()));
    }
}

void expolygons_rotate(ExPolygons &expolys, double angle)
{
    for (ExPolygon &expoly : expolys) {
        expoly.rotate(angle);
    }
}

bool expolygons_contain(const ExPolygons &expolys, const Point &pt, bool border_result)
{
    for (const ExPolygon &expoly : expolys) {
        if (expoly.contains(pt, border_result)) {
            return true;
        }
    }
    return false;
}

double area(const ExPolygon &poly) { return poly.area(); }

double area(const ExPolygons &polys)
{
    double s = 0.;
    for (auto &p : polys) {
        s += p.area();
    }
    return s;
}

void ExPolygon::scale(double factor)
{
    contour.scale(factor);
    for (Polygon &hole : holes)
        hole.scale(factor);
}

void ExPolygon::scale(double factor_x, double factor_y)
{
    contour.scale(factor_x, factor_y);
    for (Polygon &hole : holes)
        hole.scale(factor_x, factor_y);
}

void ExPolygon::translate(const Point &p)
{
    contour.translate(p);
    for (Polygon &hole : holes)
        hole.translate(p);
}

void ExPolygon::rotate(double angle)
{
    contour.rotate(angle);
    for (Polygon &hole : holes)
        hole.rotate(angle);
}

void ExPolygon::rotate(double angle, const Point &center)
{
    contour.rotate(angle, center);
    for (Polygon &hole : holes)
        hole.rotate(angle, center);
}

double ExPolygon::area() const
{
    double a = this->contour.area();
    for (const Polygon &hole : holes)
        a -= - hole.area();  // holes have negative area
    return a;
}

bool ExPolygon::is_valid() const
{
    if (!this->contour.is_valid() || !this->contour.is_counter_clockwise()) return false;
    for (Polygons::const_iterator it = this->holes.begin(); it != this->holes.end(); ++it) {
        if (!(*it).is_valid() || (*it).is_counter_clockwise()) return false;
    }
    return true;
}

void ExPolygon::douglas_peucker(double tolerance)
{
    this->contour.douglas_peucker(tolerance);
    for (Polygon &poly : this->holes)
        poly.douglas_peucker(tolerance);
}

bool ExPolygon::contains(const Line &line) const
{
    return this->contains(Polyline(line.a, line.b));
}

bool ExPolygon::contains(const Polyline &polyline) const
{
    return diff_pl(polyline, *this).empty();
}

bool ExPolygon::contains(const Polylines &polylines) const
{
    #if 0
    BoundingBox bbox = get_extents(polylines);
    bbox.merge(get_extents(*this));
    SVG svg(debug_out_path("ExPolygon_contains.svg"), bbox);
    svg.draw(*this);
    svg.draw_outline(*this);
    svg.draw(polylines, "blue");
    #endif
    Polylines pl_out = diff_pl(polylines, *this);
    #if 0
    svg.draw(pl_out, "red");
    #endif
    return pl_out.empty();
}

bool ExPolygon::contains(const Point &point, bool border_result /* = true */) const
{
    if (! Slic3r::contains(contour, point, border_result))
        // Outside the outer contour, not on the contour boundary.
        return false;
    for (const Polygon &hole : this->holes)
        if (Slic3r::contains(hole, point, ! border_result))
            // Inside a hole, not on the hole boundary.
            return false;
    return true;
}

bool ExPolygon::on_boundary(const Point &point, double eps) const
{
    if (this->contour.on_boundary(point, eps))
        return true;
    for (const Polygon &hole : this->holes)
        if (hole.on_boundary(point, eps))
            return true;
    return false;
}

// Projection of a point onto the polygon.
Point ExPolygon::point_projection(const Point &point) const
{
    if (this->holes.empty()) {
        return this->contour.point_projection(point).first;
    } else {
        double dist_min2 = std::numeric_limits<double>::max();
        Point closest_pt_min;
        for (size_t i = 0; i < this->num_contours(); ++i) {
            Point closest_pt = this->contour_or_hole(i).point_projection(point).first;
            double d2 = (closest_pt - point).cast<double>().squaredNorm();
            if (d2 < dist_min2) {
                dist_min2 = d2;
                closest_pt_min = closest_pt;
            }
        }
        return closest_pt_min;
    }
}

bool ExPolygon::overlaps(const ExPolygon &other) const
{
    if (this->empty() || other.empty())
        return false;

    #if 0
    BoundingBox bbox = get_extents(other);
    bbox.merge(get_extents(*this));
    static int iRun = 0;
    SVG svg(debug_out_path("ExPolygon_overlaps-%d.svg", iRun ++), bbox);
    svg.draw(*this);
    svg.draw_outline(*this);
    svg.draw_outline(other, "blue");
    #endif

    Polylines pl_out = intersection_pl(to_polylines(other), *this);

    #if 0
    svg.draw(pl_out, "red");
    #endif

    // See unit test SCENARIO("Clipper diff with polyline", "[Clipper]")
    // for in which case the intersection_pl produces any intersection.
    return ! pl_out.empty() ||
           // If *this is completely inside other, then pl_out is empty, but the expolygons overlap. Test for that situation.
           other.contains(this->contour.points.front());
}

// @Deprecated. please don't use it. a simplification can cut a thin isma.
void ExPolygon::douglas_peucker(coord_t tolerance) {
    assert(false); //deprecated
    bool need_union = false;
    assert(this->contour.size() < 3 || this->contour.is_counter_clockwise());
    for(auto &hole :this->holes) assert(hole.is_clockwise());
    this->contour.douglas_peucker(tolerance);
    for(auto &hole :this->holes) assert(hole.is_clockwise());
    if (this->contour.size() < 3) {
        this->clear();
    } else {
        if (!this->contour.is_counter_clockwise()) {
            this->contour.reverse();
            need_union = true;
        }
        for(auto &hole :this->holes) assert(hole.is_clockwise());
        for (size_t i_hole = 0; i_hole < this->holes.size(); ++i_hole) {
            assert(this->holes[i_hole].size() < 3 || this->holes[i_hole].is_clockwise());
            this->holes[i_hole].douglas_peucker(tolerance);
            if (this->holes[i_hole].size() < 3) {
                this->holes.erase(this->holes.begin() + i_hole);
                --i_hole;
            } else {
                if (!this->holes[i_hole].is_clockwise()) {
                    this->holes[i_hole].reverse();
                    need_union = true;
                }
            }
        }
    }
    // do we need to do an union_ex() here? -> it's possible that the new holes cut into the new perimeter, so yes... even if unlikely
    
    assert_valid();
    if (need_union) {
        ExPolygons expolygons = union_ex(expolygons);
        assert(expolygons.size() == 1);
        if (expolygons.size() > 0) {
            //TODO choose biggest
            *this = expolygons.front();
            this->douglas_peucker(tolerance);
        } else {
            clear();
        }

    }
    assert_valid();
}

//FIXME: dangerous, please not use it (polygons may be not ordered correctly).
void
ExPolygon::simplify_p(coord_t tolerance, Polygons &polygons) const
{
    Polygons pp = this->simplify_p(tolerance);
    polygons.insert(polygons.end(), pp.begin(), pp.end());
}

Polygons
ExPolygon::simplify_p(coord_t tolerance) const
{
    //Polygons pp;
    //pp.reserve(this->holes.size() + 1);
    //// contour
    //{
    //    Polygon p = this->contour;
    //    p.points.push_back(p.points.front());
    //    p.points = MultiPoint::douglas_peucker(p.points, tolerance);
    //    p.points.pop_back();
    //    pp.emplace_back(std::move(p));
    //}
    //// holes
    //for (Polygon p : this->holes) {
    //    p.points.push_back(p.points.front());
    //    p.points = MultiPoint::douglas_peucker(p.points, tolerance);
    //    p.points.pop_back();
    //    pp.emplace_back(std::move(p));
    //}
    //return simplify_polygons(pp);
    Polygons pp;
    pp.reserve(this->holes.size() + 1);
    // contour
    {
        Polygon p = this->contour;
        assert(p.is_counter_clockwise());
        p.douglas_peucker(tolerance);
        if (!p.is_counter_clockwise()) {
            p.reverse();
        }
        if (p.size() >= 2) {
            pp.push_back(std::move(p));
        }
    }
    if(pp.empty()) return pp;
    // holes
    for (Polygon polygon : this->holes) {
        Polygon oldp = polygon;
        assert(oldp.is_clockwise());
        polygon.douglas_peucker(tolerance);
#ifdef _DEBUG
        if (polygon.size() > 2 && polygon.is_counter_clockwise()) {
            static int aodfjiaqsdz = 0;
            std::stringstream stri;
            stri <<  "_hourglass_" << (aodfjiaqsdz++) << ".svg";
            SVG svg(stri.str());
            oldp.scale(1000,1000);
            polygon.scale(1000,1000);
            svg.draw(oldp, "grey");
            svg.draw(oldp.split_at_first_point(), "orange", scale_i(0.05));
            svg.draw(polygon, "black");
            svg.draw(polygon.split_at_first_point(), "red", scale_i(0.04));
            Polygons polys = union_(Polygons{oldp});
            svg.draw(to_polylines(polys), "cyan", scale_i(0.032));
            polys = union_(Polygons{polygon});
            svg.draw(to_polylines(polys), "blue", scale_i(0.025));
            svg.Close();
            assert(false);
        }
#endif
        // if polygon began to be counnter-clockwise, then it means that the fucked up part of the
        //   hourglass is the only part / dominant part left
        if (polygon.size() > 2 && polygon.is_clockwise()) {
            // size == 2 => triangle
            pp.push_back(std::move(polygon));
        }
    }
    // union
    return simplify_polygons(pp);
}

ExPolygons
ExPolygon::simplify(coord_t tolerance) const
{
    //return union_ex(this->simplify_p(tolerance));
    ExPolygons expolys;
    this->simplify(tolerance, expolys);
    return expolys;
}

void
ExPolygon::simplify(coord_t tolerance, ExPolygons &expolygons) const
{
    //append(*expolygons, this->simplify(tolerance));
    
    bool need_union = false;
    assert(this->contour.size() < 3 || this->contour.is_counter_clockwise());
    for(auto &hole :this->holes) assert(hole.is_clockwise());
    expolygons.push_back(*this);
    expolygons.back().contour.douglas_peucker(tolerance);
    for(auto &hole :expolygons.back().holes) assert(hole.is_clockwise());
    if (expolygons.back().contour.size() < 3) {
        expolygons.pop_back();
    } else {
        if (!expolygons.back().contour.is_counter_clockwise()) {
            expolygons.back().contour.reverse();
            need_union = true;
        }
        for(auto &hole :expolygons.back().holes) assert(hole.is_clockwise());
        for (size_t i_hole = 0; i_hole < expolygons.back().holes.size(); ++i_hole) {
            assert(expolygons.back().holes[i_hole].size() < 3 || expolygons.back().holes[i_hole].is_clockwise());
            expolygons.back().holes[i_hole].douglas_peucker(tolerance);
            if (expolygons.back().holes[i_hole].size() < 3) {
                expolygons.back().holes.erase(expolygons.back().holes.begin() + i_hole);
                --i_hole;
            } else {
                if (!expolygons.back().holes[i_hole].is_clockwise()) {
                    expolygons.back().holes[i_hole].reverse();
                    need_union = true;
                }
            }
        }
    }
    // do we need to do an union_ex() here? -> it's possible that the new holes cut into the new perimeter, so yes... even if unlikely
    
    Slic3r::assert_valid(expolygons);
    if (need_union) {
        expolygons = union_ex(expolygons);
        ensure_valid(expolygons, tolerance);
    }
    Slic3r::assert_valid(expolygons);
}

/// remove point that are at SCALED_EPSILON * 2 distance.
//simplier than simplify
void ExPolygon::remove_point_too_close(const coord_t tolerance) {
    this->contour.remove_point_too_close(tolerance);
    if (contour.empty()) {
        this->holes.clear();
    }
    for (Polygon &hole : this->holes) {
        hole.remove_point_too_close(tolerance);
    }
    //note: may need a union_ex(), as contour may now cross a hole.
}

void ExPolygon::medial_axis(double min_width, double max_width, ThickPolylines &polylines) const
{
    ThickPolylines tp;
    Geometry::MedialAxis{ *this, coord_t(max_width), coord_t(min_width), coord_t(max_width / 2.0) }.build(tp);
    polylines.insert(polylines.end(), tp.begin(), tp.end());
}

void ExPolygon::medial_axis(double min_width, double max_width, Polylines &polylines) const
{
    ThickPolylines tp;
    this->medial_axis(min_width, max_width, tp);
    polylines.reserve(polylines.size() + tp.size());
    for (auto &pl : tp)
        polylines.emplace_back(pl.points);
}

Lines ExPolygon::lines() const
{
    Lines lines = this->contour.lines();
    for (Polygons::const_iterator h = this->holes.begin(); h != this->holes.end(); ++h) {
        Lines hole_lines = h->lines();
        lines.insert(lines.end(), hole_lines.begin(), hole_lines.end());
    }
    return lines;
}

#ifdef _DEBUG
// to create a cpp multipoint to create test units.
std::string ExPolygon::to_debug_string()
{
    std::string ret("ExPolygon expoly(");
    ret += contour.to_debug_string();
    if (!holes.empty() && !holes.front().empty()) {
        ret += std::string(",");
        ret += holes.front().to_debug_string();
    } else {
        ret += std::string(",{}");
    }
    ret += std::string(");\n");
    for (size_t i = 1; i < holes.size(); ++i) {
        ret += std::string("expoly.holes.push_back(Polygon");
        ret += holes.front().to_debug_string();
        ret += std::string(")\n");
    }
    return ret;
}
#endif

Polylines to_polylines(const ExPolygon &src)
{
    Polylines polylines;
    polylines.assign(src.holes.size() + 1, Polyline());
    size_t idx = 0;
    Polyline &pl = polylines[idx ++];
    pl.points = src.contour.points;
    pl.points.push_back(pl.points.front());
    for (Polygons::const_iterator ith = src.holes.begin(); ith != src.holes.end(); ++ith) {
        Polyline &pl = polylines[idx ++];
        pl.points = ith->points;
        pl.points.push_back(ith->points.front());
    }
    assert(idx == polylines.size());
    return polylines;
}

Polylines to_polylines(const ExPolygons &src)
{
    Polylines polylines;
    polylines.assign(number_polygons(src), Polyline());
    size_t idx = 0;
    for (ExPolygons::const_iterator it = src.begin(); it != src.end(); ++it) {
        Polyline &pl = polylines[idx ++];
        pl.points = it->contour.points;
        pl.points.push_back(pl.points.front());
        for (Polygons::const_iterator ith = it->holes.begin(); ith != it->holes.end(); ++ith) {
            Polyline &pl = polylines[idx ++];
            pl.points = ith->points;
            pl.points.push_back(ith->points.front());
        }
    }
    assert(idx == polylines.size());
    return polylines;
}

Polylines to_polylines(ExPolygon &&src)
{
    Polylines polylines;
    polylines.assign(src.holes.size() + 1, Polyline());
    size_t idx = 0;
    Polyline &pl = polylines[idx ++];
    pl.points = std::move(src.contour.points);
    pl.points.push_back(pl.points.front());
    for (Polygon& ith : src.holes) {
        Polyline &pl = polylines[idx ++];
        pl.points = std::move(ith.points);
        pl.points.push_back(pl.points.front());
    }
    assert(idx == polylines.size());
    return polylines;
}

Polylines to_polylines(ExPolygons &&src)
{
    Polylines polylines;
    polylines.assign(number_polygons(src), Polyline());
    size_t idx = 0;
    for (ExPolygon& ex_poly : src) {
        Polyline &pl = polylines[idx ++];
        pl.points = std::move(ex_poly.contour.points);
        pl.points.push_back(pl.points.front());
        for (Polygon& ith : ex_poly.holes) {
            Polyline &pl = polylines[idx ++];
            pl.points = std::move(ith.points);
            pl.points.push_back(pl.points.front());
        }
    }
    assert(idx == polylines.size());
    return polylines;
}

// Do expolygons match? If they match, they must have the same topology,
// however their contours may be rotated.
bool expolygons_match(const ExPolygon &l, const ExPolygon &r)
{
    if (l.holes.size() != r.holes.size() || ! polygons_match(l.contour, r.contour))
        return false;
    for (size_t hole_idx = 0; hole_idx < l.holes.size(); ++ hole_idx)
        if (! polygons_match(l.holes[hole_idx], r.holes[hole_idx]))
            return false;
    return true;
}

BoundingBox get_extents(const ExPolygon &expolygon)
{
    return get_extents(expolygon.contour);
}

BoundingBox get_extents(const ExPolygons &expolygons)
{
    BoundingBox bbox;
    if (! expolygons.empty()) {
        for (size_t i = 0; i < expolygons.size(); ++ i)
			if (! expolygons[i].contour.points.empty())
				bbox.merge(get_extents(expolygons[i]));
    }
    return bbox;
}

BoundingBox get_extents_rotated(const ExPolygon &expolygon, double angle)
{
    return get_extents_rotated(expolygon.contour, angle);
}

BoundingBox get_extents_rotated(const ExPolygons &expolygons, double angle)
{
    BoundingBox bbox;
    if (! expolygons.empty()) {
        bbox = get_extents_rotated(expolygons.front().contour, angle);
        for (size_t i = 1; i < expolygons.size(); ++ i)
            bbox.merge(get_extents_rotated(expolygons[i].contour, angle));
    }
    return bbox;
}

extern std::vector<BoundingBox> get_extents_vector(const ExPolygons &polygons)
{
    std::vector<BoundingBox> out;
    out.reserve(polygons.size());
    for (ExPolygons::const_iterator it = polygons.begin(); it != polygons.end(); ++ it)
        out.push_back(get_extents(*it));
    return out;
}

bool has_duplicate_points(const ExPolygon &expoly)
{
#if 1
    // Check globally.
    size_t cnt = expoly.contour.points.size();
    for (const Polygon &hole : expoly.holes)
        cnt += hole.points.size();
    Points allpts;
    allpts.reserve(cnt);
    allpts.insert(allpts.begin(), expoly.contour.points.begin(), expoly.contour.points.end());
    for (const Polygon &hole : expoly.holes)
        allpts.insert(allpts.end(), hole.points.begin(), hole.points.end());
    return has_duplicate_points(std::move(allpts));
#else
    // Check per contour.
    if (has_duplicate_points(expoly.contour))
        return true;
    for (const Polygon &hole : expoly.holes)
        if (has_duplicate_points(hole))
            return true;
    return false;
#endif
}

bool has_duplicate_points(const ExPolygons &expolys)
{
#if 1
    // Check globally.
#if 0
    // Detect duplicates by sorting with quicksort. It is quite fast, but ankerl::unordered_dense is around 1/4 faster.
    Points allpts;
    allpts.reserve(count_points(expolys));
    for (const ExPolygon &expoly : expolys) {
        allpts.insert(allpts.begin(), expoly.contour.points.begin(), expoly.contour.points.end());
        for (const Polygon &hole : expoly.holes)
            allpts.insert(allpts.end(), hole.points.begin(), hole.points.end());
    }
    return has_duplicate_points(std::move(allpts));
#else
    // Detect duplicates by inserting into an ankerl::unordered_dense hash set, which is is around 1/4 faster than qsort.
    struct PointHash {
        using is_avalanching = void;
        uint64_t operator()(const Point &p) const noexcept
        {
#if COORD_64B
        uint64_t data[2] = {
            static_cast<uint64_t>(p.x()),
            static_cast<uint64_t>(p.y())
        };
        return ankerl::unordered_dense::detail::wyhash::hash(data, sizeof(data));
#else
            uint64_t h;
            static_assert(sizeof(h) == sizeof(p));
            memcpy(&h, &p, sizeof(p));
            return ankerl::unordered_dense::detail::wyhash::hash(h);
#endif
        }
    };
    ankerl::unordered_dense::set<Point, PointHash> allpts;
    allpts.reserve(count_points(expolys));
    for (const ExPolygon &expoly : expolys)
        for (size_t icontour = 0; icontour < expoly.num_contours(); ++ icontour)
            for (const Point &pt : expoly.contour_or_hole(icontour).points)
                if (! allpts.insert(pt).second)
                    // Duplicate point was discovered.
                    return true;
    return false;
#endif
#else
    // Check per contour.
    for (const ExPolygon &expoly : expolys)
        if (has_duplicate_points(expoly))
            return true;
    return false;
#endif
}

void ensure_valid(ExPolygons &expolygons, coord_t resolution /*= SCALED_EPSILON*/) {
    normalize_mesh_expolygons(expolygons);
    expolygons_simplify(expolygons, resolution);
    normalize_mesh_expolygons(expolygons);
}

void expolygons_simplify(ExPolygons &expolygons, coord_t resolution) {
    for (ExPolygon &poly : expolygons)
        for (auto &hole : poly.holes)
            assert(hole.is_clockwise());
    bool need_union = false;
    for (size_t i = 0; i < expolygons.size(); ++i) {
        assert(expolygons[i].contour.size() < 3 || expolygons[i].contour.is_counter_clockwise());
        for (auto &hole : expolygons[i].holes)
            assert(hole.is_clockwise());
        expolygons[i].contour.douglas_peucker(resolution);
        for (auto &hole : expolygons[i].holes)
            assert(hole.is_clockwise());
        if (expolygons[i].contour.size() < 3) {
            expolygons.erase(expolygons.begin() + i);
            --i;
        } else {
            if (!expolygons[i].contour.is_counter_clockwise()) {
                expolygons[i].contour.reverse();
                need_union = true;
            }
            for (auto &hole : expolygons[i].holes)
                assert(hole.is_clockwise());
            for (size_t i_hole = 0; i_hole < expolygons[i].holes.size(); ++i_hole) {
                assert(expolygons[i].holes[i_hole].size() < 3 || expolygons[i].holes[i_hole].is_clockwise());
                expolygons[i].holes[i_hole].douglas_peucker(resolution);
                if (expolygons[i].holes[i_hole].size() < 3) {
                    expolygons[i].holes.erase(expolygons[i].holes.begin() + i_hole);
                    --i_hole;
                } else {
                    if (!expolygons[i].holes[i_hole].is_clockwise()) {
                        expolygons[i].holes[i_hole].reverse();
                        need_union = true;
                    }
                }
            }
        }
        // do we need to do an union_ex() here? -> it's possible that the new holes cut into the new perimeter, so
        // yes... even if unlikely
    }
    // assert_valid(expolygons);
    for (size_t i = 0; i < expolygons.size(); ++i) {
        for (size_t i_pt = 1; i_pt < expolygons[i].contour.size(); ++i_pt) {
            if (expolygons[i].contour.points[i_pt - 1].coincides_with_epsilon(expolygons[i].contour.points[i_pt])) {
                expolygons[i].contour.douglas_peucker(resolution);
                //auto it_end = douglas_peucker_old<coord_t>(expolygons[i].contour.points.begin(),
                //                                                   expolygons[i].contour.points.end(),
                //                                                   expolygons[i].contour.points.begin(),
                //                                                   double(resolution),
                //                                           [](const Point &p) { return p; });
                //expolygons[i].contour.points.resize(std::distance(expolygons[i].contour.points.begin(), it_end));
            }
        }
    }
    if (need_union) {
        expolygons = union_ex(expolygons);
        ensure_valid(expolygons, resolution);
    }
    assert_valid(expolygons);
}

ExPolygons ensure_valid(ExPolygons &&expolygons, coord_t resolution /*= SCALED_EPSILON*/)
{
    ensure_valid(expolygons, resolution);
    return std::move(expolygons);
}

ExPolygons ensure_valid(coord_t resolution, ExPolygons &&expolygons) {
    return ensure_valid(std::move(expolygons), resolution);
}

void remove_point_too_close(ExPolygons &expolygons, coord_t resolution) {
    for (ExPolygon &expoly : expolygons) {
        expoly.remove_point_too_close(resolution);
    }
}

//note: test if a ExPolygons remove_point_too_close(ExPolygons expolygons) isn't more efficient, if the copy elision can be performed.
// ie test if b = remove_point_too_close(offset(a, 1)) (by rvalue and by value) copies more/less than
// b = offset(a, 1); remove_point_too_close(b)
ExPolygons remove_point_too_close(ExPolygons &&expolygons, coord_t resolution) {
    for (ExPolygon &expoly : expolygons) {
        expoly.remove_point_too_close(resolution);
    }
    return std::move(expolygons);
}

#ifdef _DEBUGINFO
void assert_valid(const ExPolygons &expolygons) {
    for (const ExPolygon &expolygon : expolygons) {
        expolygon.assert_valid();
    }
}
#endif

bool remove_same_neighbor(ExPolygons &expolygons)
{
    if (expolygons.empty())
        return false;
    bool remove_from_holes   = false;
    bool remove_from_contour = false;
    for (ExPolygon &expoly : expolygons) {
        remove_from_contour |= remove_same_neighbor(expoly.contour);
        remove_from_holes |= remove_same_neighbor(expoly.holes);
    }
    // Removing of expolygons without contour
    if (remove_from_contour)
        expolygons.erase(std::remove_if(expolygons.begin(), expolygons.end(),
                                        [](const ExPolygon &p) { return p.contour.points.size() <= 2; }),
                         expolygons.end());
    return remove_from_holes || remove_from_contour;
}

bool remove_sticks(ExPolygon &poly)
{
    // Clean the holes even when the contour changed: both cleanups apply to
    // independent point arrays, and the returned flag has to report either.
    bool modified = remove_sticks(poly.contour);
    modified |= remove_sticks(poly.holes);
    return modified;
}

bool remove_small_and_small_holes(ExPolygons &expolygons, double min_area)
{
    bool   modified = false;
    size_t free_idx = 0;
    for (size_t expoly_idx = 0; expoly_idx < expolygons.size(); ++expoly_idx) {
        if (std::abs(expolygons[expoly_idx].area()) >= min_area) {
            // Expolygon is big enough, so also check all its holes
            modified |= remove_small(expolygons[expoly_idx].holes, min_area);
            if (free_idx < expoly_idx) {
                std::swap(expolygons[expoly_idx].contour, expolygons[free_idx].contour);
                std::swap(expolygons[expoly_idx].holes, expolygons[free_idx].holes);
            }
            ++free_idx;
        } else
            modified = true;
    }
    if (free_idx < expolygons.size())
        expolygons.erase(expolygons.begin() + free_idx, expolygons.end());
    return modified;
}

void keep_largest_contour_only(ExPolygons &polygons)
{
	if (polygons.size() > 1) {
	    double     max_area = 0.;
	    ExPolygon* max_area_polygon = nullptr;
	    for (ExPolygon& p : polygons) {
	        double a = p.contour.area();
	        if (a > max_area) {
	            max_area         = a;
	            max_area_polygon = &p;
	        }
	    }
	    // No contour with a positive area (all zero or clockwise): there is no largest one, keep the polygons as they are.
	    if (max_area_polygon != nullptr) {
	        ExPolygon p(std::move(*max_area_polygon));
	        polygons.clear();
	        polygons.emplace_back(std::move(p));
	    }
	}
}

} // namespace Slic3r
