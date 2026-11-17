///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* Choose the pitch and phase of full-density bridge rows (see BridgeRows.hpp).

   The legacy filler fitted a whole number of rows into the rotated bounding box of the bridge. This search
   scores the actual area instead, so a triangle or an L gets the rows that cover it best.

   The rows follow the bridge angle. FillRectilinear rotates the area by -(angle + 90 degrees) so that the rows
   are vertical, puts the first row at the left of the rotated bounding box plus bridge_offset, keeps a row where
   it reaches the area shrunk by 0.48 spacing and draws it to the area shrunk by 0.05 spacing. fit() rebuilds
   these two contours and the area in the same frame as rings of points, then scores a candidate pitch and phase:
   each kept row covers a band one pitch wide along its drawn length. The area of the bridge inside that band and the
   rest of the band, both divided by the bridge area, are the coverage and the overflow.

   A candidate is valid only when its overflow is at most the uncovered part, 1 - coverage. Rows across a slanted
   edge then overflow on one side to compensate the other side they leave bare, which discrete rows cannot avoid,
   but rows parallel to an edge cannot spill beyond it, for example over the inner corner of an L.

   Between parallel edges, several phases often cover the area equally well. The rows are then centered: the space
   between the extent of the area across the rows and the outer kept rows is split between both sides.

       fit()
       |-- clipper_offset() [the 0.05 and 0.48 spacing contours of the rows]
       |-- append_rings() [rotate the area and both contours into the row frame]
       `-- score() [kept rows, their bands and the covered and overflowing areas]
           |-- intervals() [crossings of a vertical cut with rings, even-odd]
           `-- clipped_area() [exact area of the bridge inside one band]

   All work vectors are local to one call. */
#include "BridgeRows.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace slic3r_api { namespace BridgeRows {
namespace {
using Ring = std::vector<std::pair<double, double>>;
using Rings = std::vector<Ring>;
// Append the contour and holes of area rotated by angle around the origin, as the rectilinear pattern rotates them.
void append_rings(Rings &out, const ExPolygon &area, double angle);
// Sorted filled intervals of the vertical cut at x through rings, read with the even-odd rule.
std::vector<std::pair<double, double>> intervals(const Rings &rings, double x);
// A scored candidate and the positions across the rows of its first and last kept rows, to center equally good rows.
struct Candidate {
    Fit fit;
    double first = 0., last = 0.;
    bool kept = false;
};
// Coverage and overflow, in fractions of total, of the rows at phase, phase + pitch... from the left of the area.
Candidate score(const Rings &area, const Rings &outer, const Rings &inner, double left, double right, double total, double pitch,
                double phase);
// Signed area of rings inside the rectangle [x0, x1] x [y0, y1]; the sign follows the ring orientation, so holes
// subtract from their contour. Each ring is clipped by the convex window (Sutherland-Hodgman), which gives the exact
// area even when the clipped ring degenerates along the window.
double clipped_area(const Rings &rings, double x0, double x1, double y0, double y1);

void append_rings(Rings &out, const ExPolygon &area, double angle)
{
    const double c = std::cos(angle), s = std::sin(angle);
    std::vector<Polygon> polygons{area.contour()};
    for (const Polygon hole : area.holes()) polygons.push_back(hole);
    for (const Polygon polygon : polygons) {
        Ring ring;
        for (const c_point &point : polygon.points())
            ring.emplace_back(std::round(c * double(point.x) - s * double(point.y)), std::round(c * double(point.y) + s * double(point.x)));
        out.push_back(std::move(ring));
    }
}

std::vector<std::pair<double, double>> intervals(const Rings &rings, double x)
{
    std::vector<double> crossings;
    for (const Ring &ring : rings)
        for (size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
            const std::pair<double, double> &a = ring[j], &b = ring[i];
            if ((a.first <= x) != (b.first <= x))
                crossings.push_back(a.second + (b.second - a.second) * (x - a.first) / (b.first - a.first));
        }
    std::sort(crossings.begin(), crossings.end());
    std::vector<std::pair<double, double>> out;
    for (size_t i = 0; i + 1 < crossings.size(); i += 2) out.emplace_back(crossings[i], crossings[i + 1]);
    return out;
}

Candidate score(const Rings &area, const Rings &outer, const Rings &inner, double left, double right, double total, double pitch,
                double phase)
{
    Candidate out;
    double covered = 0., overflow = 0.;
    for (double x = left + phase; x <= right; x += pitch) {
        const std::vector<std::pair<double, double>> reached = intervals(inner, x);
        bool kept = false;
        for (const std::pair<double, double> &row : intervals(outer, x)) {
            // A drawn row is an interval of the 0.05 spacing contour that meets the 0.48 spacing contour.
            if (std::none_of(reached.begin(), reached.end(), [&row](const std::pair<double, double> &r) { return r.second > row.first && r.first < row.second; }))
                continue;
            kept = true;
            const double inside = std::abs(clipped_area(area, x - pitch / 2., x + pitch / 2., row.first, row.second));
            covered += inside;
            overflow += pitch * (row.second - row.first) - inside;
        }
        if (kept) {
            if (!out.kept) out.first = x;
            out.last = x;
            out.kept = true;
        }
    }
    out.fit.pitch = pitch;
    out.fit.phase = coord_t(std::llround(phase));
    out.fit.coverage = covered / total;
    out.fit.overflow = overflow / total;
    return out;
}

double clipped_area(const Rings &rings, double x0, double x1, double y0, double y1)
{
    const double bounds[4] = {x0, x1, y0, y1};
    double area = 0.;
    Ring current, next;
    for (const Ring &ring : rings) {
        current = ring;
        for (int side = 0; side < 4 && !current.empty(); ++side) {
            const bool vertical = side < 2;
            const double limit = bounds[side];
            const auto inside = [vertical, side, limit](const std::pair<double, double> &p) {
                const double value = vertical ? p.first : p.second;
                return side % 2 == 0 ? value >= limit : value <= limit;
            };
            const auto cross = [vertical, limit](const std::pair<double, double> &a, const std::pair<double, double> &b) {
                if (vertical) return std::make_pair(limit, a.second + (b.second - a.second) * (limit - a.first) / (b.first - a.first));
                return std::make_pair(a.first + (b.first - a.first) * (limit - a.second) / (b.second - a.second), limit);
            };
            next.clear();
            for (size_t i = 0; i < current.size(); ++i) {
                const std::pair<double, double> &a = current[(i + current.size() - 1) % current.size()];
                const std::pair<double, double> &b = current[i];
                if (inside(b)) {
                    if (!inside(a)) next.push_back(cross(a, b));
                    next.push_back(b);
                } else if (inside(a))
                    next.push_back(cross(a, b));
            }
            std::swap(current, next);
        }
        for (size_t i = 0, j = current.empty() ? 0 : current.size() - 1; i < current.size(); j = i++)
            area += current[j].first * current[i].second - current[i].first * current[j].second;
    }
    return area / 2.;
}
} // namespace

PitchRange pitch_range(const Config &region_config, double spacing, double nozzle_diameter)
{
    const double densest = region_config.get("bridge_overlap").get_effective_value(1.);
    const double sparsest = region_config.get("bridge_overlap_min").get_effective_value(1.);
    PitchRange range;
    range.minimum = spacing / densest;
    range.maximum = sparsest < densest ? spacing / sparsest : range.minimum;
    range.minimum = std::clamp(range.minimum, 0.5 * nozzle_diameter, 2. * nozzle_diameter);
    range.maximum = std::clamp(range.maximum, range.minimum, 2. * nozzle_diameter);
    return range;
}

Fit fit(storage_handle *storage, const ExPolygon &area, double angle, double spacing, const PitchRange &range)
{
    const ClipperContext clipper(storage);
    const StoredExPolygonCollection outer_areas = clipper_offset(clipper(area), -scale_d(0.05 * spacing), CLIPPER_JOIN_MITER, 3.).to_expolygon_collection();
    const StoredExPolygonCollection inner_areas = clipper_offset(clipper(area), -scale_d(0.48 * spacing), CLIPPER_JOIN_MITER, 3.).to_expolygon_collection();
    // The frame of FillRectilinear: rows along the bridge angle become vertical, in its float precision.
    float turn = float(angle);
    turn += float(PI / 2.);
    Rings area_rings, outer, inner;
    append_rings(area_rings, area, -double(turn));
    for (const ExPolygon part : outer_areas.readonly()) append_rings(outer, part, -double(turn));
    for (const ExPolygon part : inner_areas.readonly()) append_rings(inner, part, -double(turn));
    double left = 0., right = 0.;
    bool first = true;
    for (const Ring &ring : area_rings)
        for (const std::pair<double, double> &point : ring) {
            left = first ? point.first : std::min(left, point.first);
            right = first ? point.first : std::max(right, point.first);
            first = false;
        }
    const double total = unscaled(unscaled(area.area()));
    if (inner.empty() || !(total > 0.)) return Fit{};
    // Areas below are in scaled units squared; convert the bridge area once.
    const double total_scaled = scale_d(scale_d(total));
    const double low = scale_d(range.minimum), high = scale_d(range.maximum);
    // Difference between the space left before the first kept row and after the last one, across the rows.
    const auto imbalance = [left, right](const Candidate &c) { return c.kept ? std::abs((c.first - left) - (right - c.last)) : 0.; };
    const auto better = [&imbalance](const Candidate &a, const Candidate &b) {
        if (std::abs(a.fit.coverage - b.fit.coverage) > 1e-9) return a.fit.coverage > b.fit.coverage;
        if (std::abs(a.fit.overflow - b.fit.overflow) > 1e-9) return a.fit.overflow < b.fit.overflow;
        if (a.fit.pitch != b.fit.pitch) return a.fit.pitch < b.fit.pitch;
        // Equally good rows are centered, so that both sides share the space they leave.
        if (std::abs(imbalance(a) - imbalance(b)) > 1.) return imbalance(a) < imbalance(b);
        return a.fit.phase < b.fit.phase;
    };
    Candidate best;
    bool any = false;
    const auto consider = [&](double pitch, double phase) {
        const Candidate candidate = score(area_rings, outer, inner, left, right, total_scaled, pitch, phase);
        // Material outside the bridge may only stand for what its rows leave uncovered inside it.
        if (candidate.fit.overflow > 1. - candidate.fit.coverage + 1e-9) return;
        if (!any || better(candidate, best)) best = candidate;
        any = true;
    };
    const double pitch_step = (high - low) / 9.;
    for (int i = 0; i < 10; ++i) {
        const double pitch = low + pitch_step * i;
        for (int j = 0; j < 10; ++j) consider(pitch, pitch * j / 10.);
    }
    if (!any) return Fit{};
    const Fit coarse = best.fit;
    for (int i = -2; i <= 2; ++i) {
        const double pitch = std::clamp(coarse.pitch + pitch_step / 10. * i, low, high);
        for (int j = -2; j <= 2; ++j) {
            double phase = double(coarse.phase) + coarse.pitch / 100. * j;
            phase -= std::floor(phase / pitch) * pitch;
            consider(pitch, phase);
        }
    }
    // The sampled phases only approach the centered rows: also try the phase that centers the best rows exactly.
    if (best.kept) {
        const double pitch = best.fit.pitch;
        double phase = double(best.fit.phase) + ((right - best.last) - (best.first - left)) / 2.;
        phase -= std::floor(phase / pitch) * pitch;
        consider(pitch, phase);
    }
    if (!(best.fit.coverage > 0.)) return Fit{};
    Fit out = best.fit;
    out.pitch = unscaled(out.pitch);
    return out;
}
}}
