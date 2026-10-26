///|/ Copyright (c) 2026 NematX
///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ Copyright (c) Prusa Research 2016 - 2023 Vojtech Bubnik @bubnikv, Lukas Hejl @hejllukas, Lukas Matena @lukasmatena
///|/ Copyright (c) Slic3r 2016 Alessandro Ranellucci @alranel
///|/
///|/ Released under the GNU Affero General Public License, version 3 or later.

/*
Rectilinear "around holes" infill kernel
========================================

Host-independent part of the plugin: plain C++ on integer points, no SDK call.
The adapter (AroundHolesPattern.cpp) prepares the surface and its two offset
contour sets with the public Clipper API, then calls build_chains().

The infill is computed in a rotated frame where every infill line is vertical.
Each vertical line is cut by the contours. Where a line meets a hole (or a notch
of the outer contour) it is split; instead of jumping over the hole, the two
pieces are joined by a "hole bypass" that follows the hole contour, pushed half a
spacing into the material, choosing the shorter way round.

Origin of the code:
- vertical-line rasterisation and intersection classification are a minimal copy
  of the SuperSlicer plugin port (Plugins/Infill/Rectilinear/RectilinearFills.cpp:
  SegmentIntersection, vert_lines_for_polygon, slice_region_by_vertical_lines);
- splitting and bypass construction port NematXSlicer 2.7
  FillRectilinearAroundHoles::fill_surface_extrusion, with three intended fixes:
  the bypass contour is offset vertex by vertex (Clipper may renumber vertices,
  breaking the index walk), vertex indices are walked without the legacy
  off-by-one, and a bypass targets the next kept piece instead of a removed one.

The copy is deliberate: the host rectilinear code is private to the slicer, so
this plugin keeps working, unchanged, whatever the host does to its own copy.
All coordinates are scaled slicer units (integers), not millimetres.
*/

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace nematx_aroundholes {

using std::size_t;
using coord = int64_t;

struct Point
{
    coord x = 0, y = 0;
    bool operator==(const Point &o) const { return x == o.x && y == o.y; }
    bool operator!=(const Point &o) const { return !(*this == o); }
};

inline double distance(const Point &a, const Point &b) { return std::hypot(double(a.x - b.x), double(a.y - b.y)); }

// Rotate around the origin with rounding, like the slicer's Point::rotate.
inline Point rotated(const Point &p, double angle)
{
    const double c = std::cos(angle), s = std::sin(angle);
    return { coord(std::round(c * double(p.x) - s * double(p.y))), coord(std::round(c * double(p.y) + s * double(p.x))) };
}

using Points  = std::vector<Point>;
using Polygon = Points;               // closed, last point not repeated
using Polygons = std::vector<Polygon>;

inline double signed_area(const Polygon &poly)
{
    double a = 0.;
    for (size_t i = 0; i < poly.size(); ++i) {
        const Point &p = poly[i], &q = poly[(i + 1) % poly.size()];
        a += double(p.x) * double(q.y) - double(q.x) * double(p.y);
    }
    return 0.5 * a;
}

// One extrusion element of a chain: a printed line, or a hole bypass.
struct Element
{
    Points points;
    bool bypass = false;
};
// Elements printed one after the other without leaving the material.
using Chain = std::vector<Element>;

struct KernelInput
{
    // All polygons are already rotated by -angle (vertical-line frame).
    // outer: material boundary moved 5% of a spacing inwards (line ends).
    // inner: 48% of a spacing inwards (presence of room for a line).
    Polygons outer;
    Polygons inner;
    coord x0 = 0;            // abscissa of the first vertical line
    coord line_spacing = 0;  // distance between vertical lines
    size_t line_count = 0;
    coord min_length = 0;    // pieces shorter than this are dropped (one extrusion spacing)
    coord bypass_offset = 0; // distance of the bypass into the material (half a spacing)
    double angle = 0.;       // rotation applied back to the output points
};

namespace detail {

// Intersection of a vertical line with a contour segment (copied from the host port).
struct Intersection
{
    size_t contour = 0;  // index in outer + inner
    size_t segment = 0;  // segment (point[segment-1], point[segment])
    int64_t pos_p = 0;   // y = pos_p / pos_q, exact rational
    uint32_t pos_q = 1;
    enum Type : char { UNKNOWN, OUTER_LOW, OUTER_HIGH, INNER_LOW, INNER_HIGH } type = UNKNOWN;

    coord pos() const
    {
        int64_t p = pos_p;
        p += p < 0 ? -int64_t(pos_q >> 1) : int64_t(pos_q >> 1);
        return coord(p / int64_t(pos_q));
    }
    bool is_inner() const { return type == INNER_LOW || type == INNER_HIGH; }
    bool operator<(const Intersection &o) const
    {
        // pos_p * o.pos_q < o.pos_p * pos_q without overflow, as in the host code.
        if (pos_p == 0 || o.pos_p == 0)
            return pos_p < o.pos_p;
        const int s1 = pos_p > 0 ? 1 : -1, s2 = o.pos_p > 0 ? 1 : -1;
        if (s1 != s2)
            return s1 < 0;
        const uint64_t p1 = uint64_t(s1 > 0 ? pos_p : -pos_p), p2 = uint64_t(s2 > 0 ? o.pos_p : -o.pos_p);
        uint64_t l_hi = (p1 >> 32) * uint64_t(o.pos_q), l_lo = (p1 & 0xffffffffull) * uint64_t(o.pos_q);
        l_hi += l_lo >> 32;
        uint64_t r_hi = (p2 >> 32) * uint64_t(pos_q), r_lo = (p2 & 0xffffffffull) * uint64_t(pos_q);
        r_hi += r_lo >> 32;
        if (l_hi == r_hi) {
            l_lo &= 0xffffffffull;
            r_lo &= 0xffffffffull;
            return s1 < 0 ? l_lo > r_lo : l_lo < r_lo;
        }
        return s1 < 0 ? l_hi > r_hi : l_hi < r_hi;
    }
};

struct VerticalLine
{
    coord x = 0;
    std::vector<Intersection> hits;  // sorted by y after classify()
};

class Contours
{
public:
    explicit Contours(const KernelInput &in) : m_in(in) {}
    size_t count() const { return m_in.outer.size() + m_in.inner.size(); }
    bool is_outer(size_t i) const { return i < m_in.outer.size(); }
    const Polygon &at(size_t i) const { return is_outer(i) ? m_in.outer[i] : m_in.inner[i - m_in.outer.size()]; }

private:
    const KernelInput &m_in;
};

// vert_lines_for_polygon(): every crossing of the vertical lines with the contours.
inline std::vector<VerticalLine> rasterize(const KernelInput &in, const Contours &contours)
{
    std::vector<VerticalLine> lines(in.line_count);
    for (size_t i = 0; i < lines.size(); ++i)
        lines[i].x = in.x0 + coord(i) * in.line_spacing;
    for (size_t ic = 0; ic < contours.count(); ++ic) {
        const Polygon &c = contours.at(ic);
        if (c.size() < 2)
            continue;
        for (size_t is = 0; is < c.size(); ++is) {
            const size_t iprev = (is == 0 ? c.size() : is) - 1;
            const Point &p1 = c[iprev], &p2 = c[is];
            coord l = std::min(p1.x, p2.x), r = std::max(p1.x, p2.x);
            int64_t il = (l - in.x0) / in.line_spacing;
            while (il * in.line_spacing + in.x0 < l)
                ++il;
            il = std::max<int64_t>(0, il);
            int64_t ir = (r - in.x0 + in.line_spacing) / in.line_spacing;
            while (ir * in.line_spacing + in.x0 > r)
                --ir;
            ir = std::min<int64_t>(int64_t(lines.size()) - 1, ir);
            for (int64_t i = il; i <= ir; ++i) {
                const coord x = lines[size_t(i)].x;
                Intersection hit;
                hit.contour = ic;
                hit.segment = is;
                if (p1.x == x) {
                    if (p2.x == x)
                        continue;  // vertical segment
                    const Point &p0 = c[(iprev == 0 ? c.size() : iprev) - 1];
                    if ((p0.x - p1.x) * (p2.x - p1.x) > 0)
                        continue;  // vertex touching the line from one side
                    hit.pos_p = p1.y;
                } else if (p2.x == x) {
                    const Point &p3 = c[(is + 1) % c.size()];
                    if ((p3.x - p2.x) * (p1.x - p2.x) > 0)
                        continue;
                    hit.pos_p = p2.y;
                } else {
                    const bool up = p2.x > p1.x;
                    hit.pos_p = up ? x - p1.x : p1.x - x;
                    hit.pos_q = uint32_t(up ? p2.x - p1.x : p1.x - p2.x);
                    hit.pos_p *= (p2.y - p1.y);
                    hit.pos_p += p1.y * int64_t(hit.pos_q);
                }
                lines[size_t(i)].hits.push_back(hit);
            }
        }
    }
    return lines;
}

// slice_region_by_vertical_lines(): sort the crossings, type them, drop the
// duplicates created at vertices, then check the LOW/HIGH pairing.
inline void classify(std::vector<VerticalLine> &lines, const Contours &contours)
{
    for (VerticalLine &line : lines) {
        auto &hits = line.hits;
        // Stable: equal heights keep the contour order, the same on every compiler.
        std::stable_sort(hits.begin(), hits.end());
        size_t j = 0;
        for (size_t i = 0; i < hits.size(); ++i) {
            Intersection &h = hits[i];
            const Polygon &c = contours.at(h.contour);
            const size_t iprev = (h.segment == 0 ? c.size() : h.segment) - 1;
            const bool low = c[h.segment].x - c[iprev].x > 0;
            h.type = contours.is_outer(h.contour) ? (low ? Intersection::OUTER_LOW : Intersection::OUTER_HIGH)
                                                  : (low ? Intersection::INNER_LOW : Intersection::INNER_HIGH);
            bool take = true;
            if (j > 0) {
                Intersection &h2 = hits[j - 1];
                if (h.contour == h2.contour && h.pos_q == 1 && h2.pos_q == 1) {
                    if (h.pos_p == h2.pos_p) {
                        take = false;  // two segments meeting on the line
                    } else if (h.type == h2.type) {
                        if (!low)
                            h2 = h;    // Z shape: keep the longest vertical run
                        take = false;
                    }
                }
            }
            if (take) {
                if (j < i)
                    hits[j] = hits[i];
                ++j;
            }
        }
        hits.resize(j);
        // Robustness from the host port: odd counts come from degenerate input.
        if ((hits.size() & 1) == 1 && hits.size() > 1 && hits.back().contour == hits[hits.size() - 2].contour)
            hits.pop_back();
        if (hits.size() == 1)
            hits.clear();
        // A line whose crossings do not pair up (degenerate contour) is left
        // empty instead of failing the whole export.
        for (size_t i = 0; i < hits.size();) {
            size_t k = i + 1;
            while (k < hits.size() && hits[k].is_inner())
                ++k;
            if (hits[i].type != Intersection::OUTER_LOW || k >= hits.size() || hits[k].type != Intersection::OUTER_HIGH) {
                hits.clear();
                break;
            }
            i = k + 1;
        }
    }
}

// Move every vertex of a contour by `distance` towards the material, keeping its
// numbering. Material is on the left of every edge (contours CCW, holes CW).
// Returns the source when an edge would turn back (contour too small).
inline Polygon offset_into_material(const Polygon &c, coord dist)
{
    const size_t n = c.size();
    if (n < 3 || dist == 0)
        return c;
    std::vector<double> nx(n), ny(n);  // left normal of edge i -> i+1
    for (size_t i = 0; i < n; ++i) {
        const Point &a = c[i], &b = c[(i + 1) % n];
        const double dx = double(b.x - a.x), dy = double(b.y - a.y), len = std::hypot(dx, dy);
        if (len == 0.)
            return c;
        nx[i] = -dy / len;
        ny[i] = dx / len;
    }
    Polygon out(n);
    for (size_t i = 0; i < n; ++i) {
        const size_t ip = (i + n - 1) % n;
        const double cos1 = nx[ip] * nx[i] + ny[ip] * ny[i];
        // Miter length d / cos(half angle), limited to 3 d like the host offsets.
        const double scale = double(dist) / std::max(1. + cos1, 2. / 9.);
        out[i] = { c[i].x + coord(std::llround((nx[ip] + nx[i]) * scale)),
                   c[i].y + coord(std::llround((ny[ip] + ny[i]) * scale)) };
    }
    for (size_t i = 0; i < n; ++i) {
        const Point &a = c[i], &b = c[(i + 1) % n], &oa = out[i], &ob = out[(i + 1) % n];
        if (double(b.x - a.x) * double(ob.x - oa.x) + double(b.y - a.y) * double(ob.y - oa.y) <= 0.)
            return c;
    }
    return out;
}

// Path from `from` (on segment a) to `to` (on segment b) along `poly`, through
// the shorter of the two directions. Segment k joins vertices k-1 and k.
inline Points walk_contour(const Polygon &poly, size_t a, const Point &from, size_t b, const Point &to)
{
    const size_t n = poly.size();
    Points forward{ from }, backward{ from };
    for (size_t k = a % n; k != b % n; k = (k + 1) % n)
        forward.push_back(poly[k]);
    size_t k = a % n;
    do {
        k = (k + n - 1) % n;
        backward.push_back(poly[k]);
    } while (k != b % n);
    forward.push_back(to);
    backward.push_back(to);
    auto length = [](const Points &p) {
        double l = 0.;
        for (size_t i = 1; i < p.size(); ++i)
            l += distance(p[i - 1], p[i]);
        return l;
    };
    return length(backward) < length(forward) ? backward : forward;
}

} // namespace detail

// Build the chains of one surface, already rotated back to the surface frame.
inline std::vector<Chain> build_chains(const KernelInput &in)
{
    using namespace detail;
    std::vector<Chain> out;
    if (in.inner.empty() || in.line_count == 0 || in.line_spacing <= 0)
        return out;
    const Contours contours(in);
    std::vector<VerticalLine> lines = rasterize(in, contours);
    classify(lines, contours);

    std::vector<Polygon> offset_cache(contours.count());
    std::vector<bool> offset_done(contours.count(), false);
    auto bypass_contour = [&](size_t end_hit_contour, const VerticalLine &line, size_t end) -> const Polygon & {
        // Legacy heuristic: use the neighbouring (inner) contour when it runs
        // within one spacing of this one. Its numbering is only trusted when every
        // vertex has a counterpart closer than one spacing.
        size_t ic = end_hit_contour;
        if (end > 0) {
            const Intersection &prev = line.hits[end - 1], &cur = line.hits[end];
            const Polygon &a = contours.at(prev.contour), &b = contours.at(cur.contour);
            bool near = prev.contour != cur.contour && std::abs(prev.pos() - cur.pos()) < in.min_length &&
                        a.size() == b.size();
            for (size_t k = 0; near && k < a.size(); ++k)
                near = distance(a[k], b[k]) < double(in.min_length);
            if (near)
                ic = prev.contour;
        }
        if (!offset_done[ic]) {
            offset_cache[ic] = offset_into_material(contours.at(ic), in.bypass_offset);
            offset_done[ic] = true;
        }
        return offset_cache[ic];
    };

    for (const VerticalLine &line : lines) {
        // 1. Cut the line each time it leaves the material (OUTER_HIGH). The
        //    legacy code only cut when the next crossing was on the same contour,
        //    which printed across the void between two islands.
        struct Piece { Points points; size_t first = 0, last = 0; };
        std::vector<Piece> pieces;
        Piece current;
        for (size_t k = 0; k < line.hits.size(); ++k) {
            const Intersection &h = line.hits[k];
            if (current.points.empty())
                current.first = k;
            current.points.push_back({ line.x, h.pos() });
            const bool cut = k + 1 < line.hits.size() && h.type == Intersection::OUTER_HIGH;
            if (cut || k + 1 == line.hits.size()) {
                current.last = k;
                pieces.push_back(std::move(current));
                current = Piece();
            }
        }
        // 2. Drop pieces too short to print.
        pieces.erase(std::remove_if(pieces.begin(), pieces.end(), [&](const Piece &p) {
                         return p.points.size() < 2 || distance(p.points.front(), p.points.back()) < double(in.min_length);
                     }), pieces.end());
        // 3. Chain the kept pieces; bypass when two consecutive pieces border the
        //    same contour, otherwise start a new chain (ordering will travel).
        Chain chain;
        for (size_t i = 0; i < pieces.size(); ++i) {
            chain.push_back({ pieces[i].points, false });
            if (i + 1 == pieces.size())
                break;
            const Intersection &end = line.hits[pieces[i].last];
            const Intersection &next = line.hits[pieces[i + 1].first];
            if (end.type == Intersection::OUTER_HIGH && next.type == Intersection::OUTER_LOW &&
                end.contour == next.contour) {
                const Polygon &poly = bypass_contour(end.contour, line, pieces[i].last);
                chain.push_back({ walk_contour(poly, end.segment, pieces[i].points.back(), next.segment,
                                               pieces[i + 1].points.front()), true });
            } else {
                out.push_back(std::move(chain));
                chain.clear();
            }
        }
        if (!chain.empty())
            out.push_back(std::move(chain));
    }

    // 4. Back to the surface frame; remove repeated points.
    for (Chain &chain : out)
        for (Element &e : chain) {
            Points cleaned;
            for (const Point &p : e.points) {
                const Point q = rotated(p, in.angle);
                if (cleaned.empty() || cleaned.back() != q)
                    cleaned.push_back(q);
            }
            e.points = std::move(cleaned);
        }
    // A bypass is kept even when tiny, to keep the chain continuous; empty
    // elements (a single point) are removed.
    for (Chain &chain : out)
        chain.erase(std::remove_if(chain.begin(), chain.end(), [](const Element &e) { return e.points.size() < 2; }),
                    chain.end());
    out.erase(std::remove_if(out.begin(), out.end(), [](const Chain &c) { return c.empty(); }), out.end());
    return out;
}

} // namespace nematx_aroundholes
