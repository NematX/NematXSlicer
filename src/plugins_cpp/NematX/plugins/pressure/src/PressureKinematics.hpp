///|/ Released under the GNU Affero General Public License, version 3 or later.

// Analytical motion phases for callers with an already normalized movement.
// Durations are in seconds. All distances (mm), speeds (mm/s), and acceleration
// magnitudes (mm/s^2) use the same dominant-axis coordinate chosen by the caller. Construct a Motion,
// call solve_phases, then read the first count entries of the returned Profile.
// Inputs and results are owned values; no configuration or G-code is accessed.
#pragma once

#include <array>
#include <cstddef>

namespace nematx_pressure {

// Distinguishes increasing speed, constant speed, and decreasing speed.
enum class PhaseKind { Accel, Steady, Decel };

// Describes one normalized movement; acceleration and deceleration are positive magnitudes.
struct Motion
{
    double distance;
    double speed_limit;
    double speed_start;
    double speed_end;
    double acceleration;
    double deceleration;
};

// One positive-distance, positive-duration interval, with speeds at both ends.
struct Phase
{
    PhaseKind kind;
    double distance;
    double duration;
    double speed_start;
    double speed_end;
};

// Owns at most three ordered phases; entries beyond count are unused.
struct Profile
{
    std::array<Phase, 3> phases{};
    std::size_t count = 0;
};

// Returns analytical phases in model units; throws std::invalid_argument for
// nonfinite inputs, nonpositive distance/limit/accelerations, endpoints outside
// [0, limit], infeasible endpoint changes, or nonrepresentable phase results.
Profile solve_phases(const Motion &motion);

} // namespace nematx_pressure
