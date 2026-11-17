///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* Choose the pitch and phase of full-density bridge rows (see BridgeRows.hpp).

   The legacy filler fitted a whole number of rows into the rotated bounding box of the bridge. This search
   scores the actual area instead, so a triangle or an L gets the rows that cover it best.

   The rows follow the bridge angle. FillRectilinear rotates the area by -(angle + 90 degrees) so that the rows
   are vertical, puts the first row at the left of the rotated bounding box plus bridge_offset, keeps a row where
   it reaches the area shrunk by 0.48 spacing and draws it to the area shrunk by 0.05 spacing. fit() rebuilds
   these two contours and the area in the same frame as rings of points, then scores a candidate pitch and phase:
   each kept row covers a band one pitch wide, sampled by four parallel cuts. The covered area inside the bridge
   and the part outside it, both divided by the bridge area, are the coverage and the overflow.

       fit()
       |-- clipper_offset() [the 0.05 and 0.48 spacing contours of the rows]
       |-- append_rings() [rotate the area and both contours into the row frame]
       `-- score() [kept rows, their bands and the covered and overflowing areas]
           `-- intervals() [crossings of a vertical cut with rings, even-odd]

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
// Coverage and overflow, in fractions of total, of the rows at phase, phase + pitch... from the left of the area.
Fit score(const Rings &area, const Rings &outer, const Rings &inner, double left, double right, double total, double pitch,
          double phase);

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

Fit score(const Rings &area, const Rings &outer, const Rings &inner, double left, double right, double total, double pitch,
          double phase)
{
    constexpr int cuts = 4;
    double covered = 0., overflow = 0.;
    for (double x = left + phase; x <= right; x += pitch) {
        const std::vector<std::pair<double, double>> reached = intervals(inner, x);
        for (const std::pair<double, double> &row : intervals(outer, x)) {
            // A drawn row is an interval of the 0.05 spacing contour that meets the 0.48 spacing contour.
            if (std::none_of(reached.begin(), reached.end(), [&row](const std::pair<double, double> &r) { return r.second > row.first && r.first < row.second; }))
                continue;
            for (int cut = 0; cut < cuts; ++cut) {
                double inside = 0.;
                for (const std::pair<double, double> &part : intervals(area, x + pitch * ((cut + 0.5) / cuts - 0.5)))
                    inside += std::max(0., std::min(part.second, row.second) - std::max(part.first, row.first));
                covered += inside * pitch / cuts;
                overflow += (row.second - row.first - inside) * pitch / cuts;
            }
        }
    }
    Fit fit;
    fit.pitch = pitch;
    fit.phase = coord_t(std::llround(phase));
    fit.coverage = covered / total;
    fit.overflow = overflow / total;
    return fit;
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
    const auto better = [](const Fit &a, const Fit &b) {
        if (std::abs(a.coverage - b.coverage) > 1e-9) return a.coverage > b.coverage;
        if (std::abs(a.overflow - b.overflow) > 1e-9) return a.overflow < b.overflow;
        if (a.pitch != b.pitch) return a.pitch < b.pitch;
        return a.phase < b.phase;
    };
    Fit best;
    bool any = false;
    const auto consider = [&](double pitch, double phase) {
        const Fit candidate = score(area_rings, outer, inner, left, right, total_scaled, pitch, phase);
        if (!any || better(candidate, best)) best = candidate;
        any = true;
    };
    const double pitch_step = (high - low) / 9.;
    for (int i = 0; i < 10; ++i) {
        const double pitch = low + pitch_step * i;
        for (int j = 0; j < 10; ++j) consider(pitch, pitch * j / 10.);
    }
    const Fit coarse = best;
    for (int i = -2; i <= 2; ++i) {
        const double pitch = std::clamp(coarse.pitch + pitch_step / 10. * i, low, high);
        for (int j = -2; j <= 2; ++j) {
            double phase = double(coarse.phase) + coarse.pitch / 100. * j;
            phase -= std::floor(phase / pitch) * pitch;
            consider(pitch, phase);
        }
    }
    if (!(best.coverage > 0.)) return Fit{};
    best.pitch = unscaled(best.pitch);
    return best;
}
}}
