///|/ Released under the GNU Affero General Public License, version 3 or later.

// Projects one planned printing source onto the configured XYZ output grid.
// PressurePlanner.hpp defines PlannedMove and its Profile of 1..3 positive phases.
// Their model distance is the largest absolute XYZ or nominal filament delta,
// in mm; model speeds are mm/s and durations are seconds.
// Call project_geometry with absolute source XYZ and cumulative nominal filament
// endpoints in mm. Read only count groups; all values are owned. Check status
// before using any group for output or Extrusion::advance. Only Representable
// groups describe spatial output. Keep source K, tool and annotations separately.
// No extrusion state, text, configuration or inter-source merging is involved.
#pragma once

#include <array>
#include <cstddef>

namespace nematx_pressure {
struct PlannedMove;

// Absolute X, Y, Z coordinates in mm, in that order.
using Point = std::array<double, 3>;

// Absolute source endpoints and unquantized cumulative nominal filament values.
struct SourceGeometry
{
    Point start;
    Point end;
    double nominal_start;
    double nominal_end;
};

// Merged identifies two or more original phases, not constant acceleration.
enum class SegmentKind { Accel, Steady, Decel, Merged };
// Representable groups may be emitted; an entirely collapsed aggregate may not.
enum class GeometryStatus { Representable, EntireSourceCollapsed };

// Consecutive original phases assigned exactly once to a spatial group or aggregate.
// Duration and distance are sums; endpoint speeds do not define a new motion law.
// Start/end XYZ are quantized; nominal endpoints remain unquantized filament mm.
// Distance uses dominant model mm, duration seconds and endpoint speeds mm/s.
// first_phase and phase_count select a zero-based range in original source order.
struct GeometryGroup
{
    Point start;
    Point end;
    double nominal_start;
    double nominal_end;
    double distance;
    double duration;
    double speed_start;
    double speed_end;
    std::size_t first_phase;
    std::size_t phase_count;
    SegmentKind kind;
};

// EntireSourceCollapsed owns one non-emittable aggregate retaining all source data.
struct GeometryResult
{
    GeometryStatus status = GeometryStatus::Representable;
    std::array<GeometryGroup, 3> groups{};
    std::size_t count = 0;
};

// Quantizes all XYZ boundaries before returning, merging collapsed phases within this
// source only. Requires a valid planner move/profile (1..3 phases), coherent source
// geometry and nondecreasing nominal endpoints. Rejects invalid digits (0..9),
// nonfinite coordinates/nominals or unrepresentable arithmetic with invalid_argument;
// borrowed inputs remain unchanged and no partial result is returned.
GeometryResult project_geometry(const PlannedMove &planned, const SourceGeometry &source,
                                int xyz_digits);

} // namespace nematx_pressure
