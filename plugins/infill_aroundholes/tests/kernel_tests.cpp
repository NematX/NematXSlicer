///|/ Copyright (c) 2026 NematX
///|/
///|/ Released under the GNU Affero General Public License, version 3 or later.

// Kernel tests: plain C++, no host library. Coordinates are scaled units.
#include "AroundHolesKernel.hpp"

#include <iostream>
#include <stdexcept>

using namespace nematx_aroundholes;

namespace {

void require(bool condition, const char *message)
{
    if (!condition)
        throw std::runtime_error(message);
}

Polygon square(coord a, coord b, bool ccw)
{
    Polygon p{ { a, a }, { b, a }, { b, b }, { a, b } };
    if (!ccw)
        std::reverse(p.begin(), p.end());
    return p;
}

KernelInput plate(bool with_hole)
{
    KernelInput in;
    in.outer.push_back(square(0, 20000, true));
    in.inner.push_back(square(500, 19500, true));
    if (with_hole) {
        in.outer.push_back(square(8000, 12000, false));
        in.inner.push_back(square(7500, 12500, false));
    }
    in.x0 = 500;
    in.line_spacing = 1000;
    in.line_count = 19;
    in.min_length = 400;
    in.bypass_offset = 200;
    return in;
}

// Clip the parameter interval of a segment against an OPEN rectangle. Testing
// vertices or only the midpoint misses a segment crossing a small part of a void.
bool crosses_box(const Point &a, const Point &b, coord xmin, coord ymin, coord xmax, coord ymax)
{
    double lo = 0., hi = 1.;
    auto clip = [&](coord start, coord end, coord min, coord max) {
        if (start == end)
            return min < start && start < max;
        double t0 = double(min - start) / double(end - start);
        double t1 = double(max - start) / double(end - start);
        if (t0 > t1)
            std::swap(t0, t1);
        lo = std::max(lo, t0);
        hi = std::min(hi, t1);
        return lo < hi;
    };
    return clip(a.x, b.x, xmin, xmax) && clip(a.y, b.y, ymin, ymax);
}

void check_chains(const std::vector<Chain> &chains, bool has_hole)
{
    for (const Chain &chain : chains) {
        require(!chain.empty() && !chain.front().bypass && !chain.back().bypass, "chain must start and end with a line");
        for (size_t i = 0; i + 1 < chain.size(); ++i)
            require(chain[i].points.back() == chain[i + 1].points.front(), "chain is not continuous");
        for (const Element &e : chain)
            for (size_t i = 1; i < e.points.size(); ++i) {
                const Point &a = e.points[i - 1], &b = e.points[i];
                require(!has_hole || !crosses_box(a, b, 8000, 8000, 12000, 12000), "path crosses the hole");
            }
    }
}

} // namespace

int main()
{
    try {
        // Segment indices refer to their END vertex, in both traversal
        // directions, including a walk crossing the last/first vertex.
        const Polygon contour = square(0, 100, true);
        require(detail::walk_contour(contour, 1, { 80, 0 }, 2, { 100, 30 }) ==
                    Points({ { 80, 0 }, { 100, 0 }, { 100, 30 } }), "forward contour walk");
        require(detail::walk_contour(contour, 2, { 100, 30 }, 1, { 80, 0 }) ==
                    Points({ { 100, 30 }, { 100, 0 }, { 80, 0 } }), "backward contour walk");
        require(detail::walk_contour(contour, 0, { 0, 20 }, 1, { 30, 0 }) ==
                    Points({ { 0, 20 }, { 0, 0 }, { 30, 0 } }), "wrapped contour walk");

        // Without hole: one straight line per vertical, no bypass.
        std::vector<Chain> chains = build_chains(plate(false));
        require(chains.size() == 19, "one chain per line expected");
        for (const Chain &chain : chains)
            require(chain.size() == 1 && chain[0].points.size() >= 2, "single line expected");
        check_chains(chains, false);

        // With a hole: lines x = 8500 .. 11500 are split and bypass the hole.
        chains = build_chains(plate(true));
        check_chains(chains, true);
        size_t bypasses = 0;
        for (const Chain &chain : chains)
            for (const Element &e : chain)
                if (e.bypass) {
                    ++bypasses;
                    // Shorter way round: never more than half the contour plus the offset.
                    double length = 0.;
                    for (size_t i = 1; i < e.points.size(); ++i)
                        length += distance(e.points[i - 1], e.points[i]);
                    require(length < 2. * 4000. + 4. * 1000., "bypass took the long way");
                    // Intermediate vertices are pushed into the material.
                    for (size_t i = 1; i + 1 < e.points.size(); ++i) {
                        const Point &p = e.points[i];
                        const bool outside = p.x <= 7800 || p.x >= 12200 || p.y <= 7800 || p.y >= 12200;
                        require(outside, "bypass vertex not offset into the material");
                    }
                }
        require(bypasses == 4, "four lines cross the hole");
        require(chains.size() == 19, "bypass keeps one chain per line");

        // Rotation is applied back to the output.
        KernelInput rotated_in = plate(true);
        rotated_in.angle = 3.14159265358979323846 / 2.;
        for (const Chain &chain : build_chains(rotated_in))
            for (const Element &e : chain)
                for (const Point &p : e.points)
                    require(p.x <= 0 && p.y >= 0, "points not rotated back");

        // Too short pieces are removed and the chain is broken instead of bypassing to them.
        KernelInput thin = plate(true);
        thin.min_length = 9000;
        for (const Chain &chain : build_chains(thin))
            for (const Element &e : chain)
                require(!e.bypass, "no bypass towards a removed piece");

        // Two islands stacked on the same lines: never print across the void.
        KernelInput islands;
        islands.outer = { square(0, 8000, true), { { 0, 12000 }, { 8000, 12000 }, { 8000, 20000 }, { 0, 20000 } } };
        islands.inner = { square(500, 7500, true), { { 500, 12500 }, { 7500, 12500 }, { 7500, 19500 }, { 500, 19500 } } };
        islands.x0 = 500;
        islands.line_spacing = 1000;
        islands.line_count = 8;
        islands.min_length = 400;
        islands.bypass_offset = 200;
        const std::vector<Chain> island_chains = build_chains(islands);
        require(island_chains.size() == 16, "two separate island chains per line expected");
        for (const Chain &chain : island_chains)
            for (const Element &e : chain) {
                require(!e.bypass, "no bypass between two islands");
                for (size_t i = 1; i < e.points.size(); ++i)
                    require(!crosses_box(e.points[i - 1], e.points[i], -1, 8000, 8001, 12000),
                            "line printed across the void between islands");
            }

        // Two close holes: the piece between them (1000 long) is dropped; the
        // chain is split rather than bypassing towards the dropped piece.
        KernelInput twin = plate(false);
        twin.outer.push_back(square(8000, 9000, false));
        twin.inner.push_back(square(7500, 9500, false));
        twin.outer.push_back({ { 8000, 10000 }, { 8000, 12000 }, { 10000, 12000 }, { 10000, 10000 } });
        twin.inner.push_back({ { 7500, 9500 }, { 7500, 12500 }, { 10500, 12500 }, { 10500, 9500 } });
        twin.min_length = 1500;
        twin.x0 = 8500;
        twin.line_count = 1;
        const std::vector<Chain> twin_chains = build_chains(twin);
        check_chains(twin_chains, false);
        require(twin_chains.size() == 2, "dropped middle piece must split the chain");
        for (const Chain &chain : twin_chains)
            for (const Element &e : chain)
                require(!e.bypass, "no bypass towards or across a dropped piece");

        // Crossings on exact vertices/vertical edges must not erase nearby
        // valid lines. Translating across the origin must not change the result.
        KernelInput aligned = plate(true);
        aligned.x0 = 7000;
        aligned.line_spacing = 500;
        aligned.line_count = 13;
        const std::vector<Chain> aligned_chains = build_chains(aligned);
        require(aligned_chains.size() == 13, "edge-aligned lines disappeared");
        check_chains(aligned_chains, true);
        const coord shift = 30000;
        KernelInput negative = aligned;
        for (Polygons *set : { &negative.outer, &negative.inner })
            for (Polygon &polygon : *set)
                for (Point &p : polygon) {
                    p.x -= shift;
                    p.y -= shift;
                }
        negative.x0 -= shift;
        const std::vector<Chain> negative_chains = build_chains(negative);
        require(negative_chains.size() == aligned_chains.size(), "negative coordinates changed chain count");
        for (size_t i = 0; i < aligned_chains.size(); ++i) {
            require(negative_chains[i].size() == aligned_chains[i].size(), "negative coordinates changed elements");
            for (size_t j = 0; j < aligned_chains[i].size(); ++j) {
                const Element &a = aligned_chains[i][j], &b = negative_chains[i][j];
                require(a.bypass == b.bypass && a.points.size() == b.points.size(), "translation changed topology");
                for (size_t k = 0; k < a.points.size(); ++k)
                    require(b.points[k] == Point{ a.points[k].x - shift, a.points[k].y - shift },
                            "negative-coordinate intersection differs");
            }
        }

        std::cout << "infill_aroundholes_kernel: passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
