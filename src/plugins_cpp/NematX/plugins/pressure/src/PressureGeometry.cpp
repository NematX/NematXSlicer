///|/ Released under the GNU Affero General Public License, version 3 or later.

#include "PressureGeometry.hpp"
#include "PressurePlanner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "libslic3r/Api/plugin/cpp/gcode/GCodeFormatter.hpp"

// Prepares owned spatial groups for a caller that already planned one source.
// First validate source arithmetic and quantize every XYZ boundary with the inline
// SDK formatter. Interpolation uses dominant model distance, including dominant E.
// Clamping interpolated values to their source interval prevents rounded fractions
// from crossing the exact final endpoint. Nominal filament is not quantized here.
// Then partition phases: a collapsed head or middle phase joins the next spatial
// group; a collapsed tail joins the last group before publication. No source is mutated.
// An entirely collapsed source retains one aggregate, which the caller must not
// emit or pass to Extrusion. Resolving it across sources is not this operation.
//
// Call tree:
//   project_geometry: validate, interpolate boundaries, then partition all phases.

namespace nematx_pressure {

// Validates source arithmetic, quantizes all XYZ cuts, then groups phases before
// returning owned data. A valid planned profile and coherent, nondecreasing nominal
// endpoints are required. Invalid precision or unrepresentable source arithmetic
// throws invalid_argument without modifying borrowed inputs or returning partial data.
GeometryResult project_geometry(const PlannedMove &planned, const SourceGeometry &source,
                                int xyz_digits);

GeometryResult project_geometry(const PlannedMove &planned, const SourceGeometry &source,
                                int xyz_digits)
{
    using slic3r_api::GCodeGeneration::GCodeFormatter;
    if (xyz_digits < 0 || xyz_digits >= static_cast<int>(GCodeFormatter::pow_10.size()))
        throw std::invalid_argument("XYZ precision must be between 0 and 9");
    const double scale = GCodeFormatter::pow_10[xyz_digits];
    Point difference{};
    for (std::size_t axis = 0; axis < 3; ++axis) {
        difference[axis] = source.end[axis] - source.start[axis];
        if (!std::isfinite(source.start[axis]) || !std::isfinite(source.end[axis]) ||
            !std::isfinite(difference[axis]) || !std::isfinite(source.start[axis] * scale) ||
            !std::isfinite(source.end[axis] * scale))
            throw std::invalid_argument("Source XYZ is not representable at this precision");
    }
    const double nominal_difference = source.nominal_end - source.nominal_start;
    if (!std::isfinite(source.nominal_start) || !std::isfinite(source.nominal_end) ||
        !std::isfinite(nominal_difference))
        throw std::invalid_argument("Source nominal filament is not representable");

    const GCodeFormatter formatter(static_cast<std::uint16_t>(xyz_digits), 0);
    std::array<Point, 4> points{};
    std::array<double, 4> nominals{};
    for (std::size_t axis = 0; axis < 3; ++axis)
        points[0][axis] = formatter.quantize_xyzf(source.start[axis]);
    nominals[0] = source.nominal_start;
    // Bound interpolation before quantization so rounded fractions cannot overshoot the source.
    double prefix = 0.;
    for (std::size_t i = 0; i < planned.profile.count; ++i) {
        prefix += planned.profile.phases[i].distance;
        const bool final = i + 1 == planned.profile.count;
        const double fraction = std::min(prefix, planned.move.distance) / planned.move.distance;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const double coordinate = final ? source.end[axis] :
                std::clamp(source.start[axis] + fraction * difference[axis],
                           std::min(source.start[axis], source.end[axis]),
                           std::max(source.start[axis], source.end[axis]));
            points[i + 1][axis] = formatter.quantize_xyzf(coordinate);
        }
        nominals[i + 1] = final ? source.nominal_end :
            std::clamp(source.nominal_start + fraction * nominal_difference,
                       source.nominal_start, source.nominal_end);
    }

    // Keep collapsed phases pending so their data joins the next spatially representable group.
    GeometryResult result;
    GeometryGroup pending{};
    pending.start = points[0];
    pending.nominal_start = source.nominal_start;
    for (std::size_t i = 0; i < planned.profile.count; ++i) {
        const Phase &phase = planned.profile.phases[i];
        if (pending.phase_count == 0) {
            pending.first_phase = i;
            pending.speed_start = phase.speed_start;
        }
        ++pending.phase_count;
        pending.end = points[i + 1];
        pending.nominal_end = nominals[i + 1];
        pending.distance += phase.distance;
        pending.duration += phase.duration;
        pending.speed_end = phase.speed_end;
        if (pending.phase_count == 1) {
            switch (phase.kind) {
            case PhaseKind::Accel: pending.kind = SegmentKind::Accel; break;
            case PhaseKind::Steady: pending.kind = SegmentKind::Steady; break;
            case PhaseKind::Decel: pending.kind = SegmentKind::Decel; break;
            default: throw std::invalid_argument("Unknown analytical phase kind");
            }
        } else {
            pending.kind = SegmentKind::Merged;
        }
        if (pending.start != pending.end) {
            result.groups[result.count++] = pending;
            pending = {};
            pending.start = points[i + 1];
            pending.nominal_start = nominals[i + 1];
        }
    }
    // Preserve the final remainder within this source, even when no spatial group survives.
    if (pending.phase_count != 0) {
        if (result.count == 0) {
            result.status = GeometryStatus::EntireSourceCollapsed;
            result.groups[result.count++] = pending;
        } else {
            GeometryGroup &last = result.groups[result.count - 1];
            last.end = pending.end;
            last.nominal_end = pending.nominal_end;
            last.distance += pending.distance;
            last.duration += pending.duration;
            last.speed_end = pending.speed_end;
            last.phase_count += pending.phase_count;
            last.kind = SegmentKind::Merged;
        }
    }
    return result;
}

} // namespace nematx_pressure
