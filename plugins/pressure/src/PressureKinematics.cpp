///|/ Released under the GNU Affero General Public License, version 3 or later.

#include "PressureKinematics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

// Solves one movement supplied in a single normalized dominant-axis coordinate.
// The caller supplies distance in mm, speeds in mm/s, and acceleration magnitudes
// in mm/s^2; returned durations are in seconds. Acceleration, steady motion,
// and deceleration stay separate because they have different speed laws.
// No parsing, normalization, or output occurs.
// The returned phases own their values and retain no reference to the input.
//
// Call tree:
//   solve_phases: validate endpoints, solve the peak and distances, compute durations.

namespace nematx_pressure {

// Uses endpoint reachability to reject impossible moves, then solves a capped
// trapezoid or a triangle. A distance deficit within eight relative double epsilons
// is accepted as a rounded monotone boundary. Results preserve the supplied
// endpoints; adjacent phases share exactly the same speed value. Invalid or
// nonrepresentable values throw std::invalid_argument without changing the borrowed input.
Profile solve_phases(const Motion &motion);

Profile solve_phases(const Motion &motion)
{
    const double distance = motion.distance;
    const double limit = motion.speed_limit;
    const double start = motion.speed_start;
    const double end = motion.speed_end;
    const double acceleration = motion.acceleration;
    const double deceleration = motion.deceleration;
    if (!std::isfinite(distance) || !std::isfinite(limit) || !std::isfinite(start) ||
        !std::isfinite(end) || !std::isfinite(acceleration) || !std::isfinite(deceleration) ||
        distance <= 0. || limit <= 0. || acceleration <= 0. || deceleration <= 0. ||
        start < 0. || end < 0. || start > limit || end > limit)
        throw std::invalid_argument("Invalid normalized motion inputs");

    constexpr double roundoff_relative = 8. * std::numeric_limits<double>::epsilon();
    const double minimum_distance = end >= start ?
        (end - start) * (end + start) / (2. * acceleration) :
        (start - end) * (start + end) / (2. * deceleration);
    const double distance_tolerance = roundoff_relative * std::max(distance, minimum_distance);
    if (!std::isfinite(minimum_distance) || minimum_distance - distance > distance_tolerance)
        throw std::invalid_argument("Motion endpoints cannot be reached within the distance");

    Profile result;
    if (minimum_distance > distance && start + end > 0.) {
        result.phases[0] = { end > start ? PhaseKind::Accel : PhaseKind::Decel,
                             distance, 2. * distance / (start + end), start, end };
        result.count = 1;
    } else {
        double accel_distance = (limit - start) * (limit + start) / (2. * acceleration);
        double decel_distance = (limit - end) * (limit + end) / (2. * deceleration);
        double steady_distance = 0.;
        double peak = limit;
        if (accel_distance + decel_distance <= distance) {
            steady_distance = distance - accel_distance - decel_distance;
        } else {
            // Solve from the higher endpoint to avoid cancellation near a monotone boundary.
            const double excess_distance = distance - minimum_distance;
            if (end >= start) {
                decel_distance = acceleration / (acceleration + deceleration) * excess_distance;
                accel_distance = distance - decel_distance;
                peak = std::sqrt(end * end + 2. * deceleration * decel_distance);
            } else {
                accel_distance = deceleration / (acceleration + deceleration) * excess_distance;
                decel_distance = distance - accel_distance;
                peak = std::sqrt(start * start + 2. * acceleration * accel_distance);
            }
            peak = std::min(limit, std::max(peak, std::max(start, end)));
        }

        // An unrepresentable speed change contributes its distance to the surviving phase.
        if (peak == start && peak == end) {
            steady_distance = distance;
            accel_distance = decel_distance = 0.;
        } else if (peak == start) {
            decel_distance += accel_distance;
            accel_distance = 0.;
        } else if (peak == end) {
            accel_distance += decel_distance;
            decel_distance = 0.;
        }
        if (accel_distance > 0. && peak > start)
            result.phases[result.count++] = { PhaseKind::Accel, accel_distance,
                2. * accel_distance / (start + peak), start, peak };
        if (steady_distance > 0.)
            result.phases[result.count++] = { PhaseKind::Steady, steady_distance,
                2. * steady_distance / (peak + peak), peak, peak };
        if (decel_distance > 0. && peak > end)
            result.phases[result.count++] = { PhaseKind::Decel, decel_distance,
                2. * decel_distance / (peak + end), peak, end };
    }
    if (result.count == 0)
        throw std::invalid_argument("Motion has no representable positive phase");
    for (std::size_t i = 0; i < result.count; ++i) {
        const Phase &phase = result.phases[i];
        if (!std::isfinite(phase.distance) || !std::isfinite(phase.duration) ||
            !std::isfinite(phase.speed_start) || !std::isfinite(phase.speed_end) ||
            phase.distance <= 0. || phase.duration <= 0.)
            throw std::invalid_argument("Motion phase is not representable");
    }
    return result;
}

} // namespace nematx_pressure
