///|/ Released under the GNU Affero General Public License, version 3 or later.

#include "PressurePlanner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

// Provides pure movement normalization and a per-cycle planner to numeric callers.
// D is the largest absolute XYZE delta; Dxyz is the largest absolute XYZ delta.
// Axis projection supplies a speed cap and one acceleration for both speeding up
// and braking. Only XY directions are rounded, to 1 nm, for the angular model.
// Each arrival stores its junction cap once, using the previous attained speed.
// Backward braking and forward acceleration passes rebuild effective speeds from
// those caps; an open tail does not assume a stop. A separate stopped copy proves
// that the known suffix can accept a published endpoint and brake to zero.
// Reachable speeds are rounded downward until the phase solver's distance
// expression fits. Both passes and the stopped copy share this calculation.
// Published entries are never reduced; incompatible retained endpoints raise
// logic_error. Floating-point bounds do not guarantee successful closure for
// every possible continuation.
// The planner owns retained values; publications own their moves and profiles.
// Independent instances share no mutable state; one instance is used sequentially.
//
// Call tree:
//   normalize_move: validate axes, project limits and round XY directions.
//     rounded_direction: convert one finite displacement to signed nanometres.
//     validate_move: reject invalid normalized values before planner mutation.
//   junction_factor: fold the angle between reversed previous and next XY rays.
//   Planner::push: store a cap and publish only stop-safe whole movements.
//     validate_move: check the new value without changing retained state.
//     speeds: rebuild effective endpoints from stored caps.
//       backward: propagate the braking envelope with an open or stopped tail.
//         reachable_speed: round the bound down to a solver-compatible distance.
//       reachable_speed: use the same bound for forward acceleration.
//     junction_factor: use the previous attained endpoint once.
//     retained_distance: sum the retained source XYZ reach.
//     backward: compute the separate stopped envelope for publication.
//     planned: pass shared endpoints to solve_phases in PressureKinematics.cpp.
//   Planner::finish: solve the stopped remainder, then clear the cycle.
//     speeds / planned: use the same passes and phase solver as push.

namespace nematx_pressure {

// Projects the supplied finite axis values into one owned Move. Nonpositive axis
// limits fall back before the XYZ extrusion cap; invalid data throws invalid_argument.
Move normalize_move(double dx, double dy, double dz, double de, double feed,
                    double acceleration, const AxisLimits &limits, std::size_t source_index);
// Returns the folded XY angular factor; no retained state is changed.
double junction_factor(const Move &previous, const Move &next);

namespace {
// Returns sqrt(base^2+2*a*distance), rounded down until the phase solver's
// factorized distance fits. The base is nonnegative, a/distance positive and
// intermediate values finite in the supported numeric domain. No supplied
// endpoint is changed; nextafter adjusts only the newly computed bound.
double reachable_speed(double base, double a, double distance);
// Rounds a displacement in mm symmetrically to nm, rejecting a value whose
// rounded signed representation cannot safely be negated, with invalid_argument.
std::int64_t rounded_direction(double delta);
// Rejects malformed normalized values with invalid_argument before any insertion.
void validate_move(const Move &move);
} // namespace

Move normalize_move(double dx, double dy, double dz, double de, double feed,
                    double acceleration, const AxisLimits &limits, std::size_t source_index)
{
    const std::array<double, 4> deltas = {dx, dy, dz, de};
    if (!std::isfinite(feed) || feed <= 0. || !std::isfinite(acceleration) || acceleration <= 0. ||
        !std::isfinite(limits.max_extruding_acceleration) || de < 0.)
        throw std::invalid_argument("Invalid printing movement inputs");
    for (std::size_t axis = 0; axis < deltas.size(); ++axis) {
        if (!std::isfinite(deltas[axis]) || !std::isfinite(limits.speed[axis]) ||
            !std::isfinite(limits.acceleration[axis]))
            throw std::invalid_argument("Nonfinite printing axis data");
    }
    const double xyz = std::max({std::abs(dx), std::abs(dy), std::abs(dz)});
    const double distance = std::max(xyz, de);
    if (xyz <= 0.)
        throw std::invalid_argument("Printing movement needs spatial displacement");
    double duration = distance / feed;
    double effective_acceleration = acceleration;
    for (std::size_t axis = 0; axis < deltas.size(); ++axis) {
        const double delta = std::abs(deltas[axis]);
        if (delta == 0.)
            continue;
        const double speed = limits.speed[axis] > 0. ? limits.speed[axis] : 100000.;
        double axis_acceleration = limits.acceleration[axis] > 0. ? limits.acceleration[axis] : 100000.;
        if (axis < 3 && limits.max_extruding_acceleration > 0.)
            axis_acceleration = std::min(axis_acceleration, limits.max_extruding_acceleration);
        duration = std::max(duration, delta / speed);
        effective_acceleration = std::min(effective_acceleration, axis_acceleration * (distance / delta));
    }
    const Move result = {distance, xyz, distance / duration, effective_acceleration,
                         rounded_direction(dx), rounded_direction(dy), source_index};
    validate_move(result);
    return result;
}

double junction_factor(const Move &previous, const Move &next)
{
    if ((previous.direction_x == 0 && previous.direction_y == 0) ||
        (next.direction_x == 0 && next.direction_y == 0))
        return 0.;
    const double px = static_cast<double>(-previous.direction_x);
    const double py = static_cast<double>(-previous.direction_y);
    const double nx = static_cast<double>(next.direction_x);
    const double ny = static_cast<double>(next.direction_y);
    const double pi = std::acos(-1.);
    const double angle = std::abs(std::atan2(px * ny - py * nx, px * nx + py * ny));
    return angle < pi / 2. ? 0. : (angle - pi / 2.) / (pi / 2.);
}

Publication Planner::push(const Move &move)
{
    validate_move(move);
    double cap = 0.;
    if (!m_pending.empty()) {
        const std::vector<double> previous = speeds(false);
        cap = junction_factor(m_pending.back().move, move) * std::min(previous.back(), move.speed_limit);
    }
    m_pending.push_back({move, cap});
    std::vector<double> endpoints = speeds(false);
    Publication result;
    while (m_pending.size() > 100 && retained_distance() > 100.) {
        const std::vector<double> stop = backward(true);
        if (endpoints[1] > stop[1])
            break;
        result.moves.push_back(planned(0, endpoints));
        m_entry = endpoints[1];
        m_pending.pop_front();
        endpoints.erase(endpoints.begin());
    }
    result.horizon_insufficient = m_pending.size() > 100 && retained_distance() > 100.;
    return result;
}

Publication Planner::finish()
{
    Publication result;
    if (m_pending.empty())
        return result;
    const std::vector<double> endpoints = speeds(true);
    for (std::size_t index = 0; index < m_pending.size(); ++index)
        result.moves.push_back(planned(index, endpoints));
    m_pending.clear();
    m_entry = 0.;
    return result;
}

std::vector<double> Planner::speeds(bool stopped) const
{
    std::vector<double> result = backward(stopped);
    if (m_entry > result[0])
        throw std::logic_error("Published planner entry cannot reach retained suffix at source_index=" +
                               std::to_string(m_pending.front().move.source_index));
    result[0] = m_entry;
    for (std::size_t index = 0; index < m_pending.size(); ++index) {
        const Move &move = m_pending[index].move;
        result[index + 1] = std::min(result[index + 1],
            reachable_speed(result[index], move.acceleration, move.distance));
    }
    return result;
}

std::vector<double> Planner::backward(bool stopped) const
{
    std::vector<double> result(m_pending.size() + 1);
    result[0] = m_entry;
    for (std::size_t index = 1; index < m_pending.size(); ++index)
        result[index] = std::min({m_pending[index].entry_cap,
            m_pending[index - 1].move.speed_limit, m_pending[index].move.speed_limit});
    result.back() = stopped ? 0. : m_pending.back().move.speed_limit;
    for (std::size_t index = m_pending.size(); index-- > 0;) {
        const Move &move = m_pending[index].move;
        result[index] = std::min(result[index],
            reachable_speed(result[index + 1], move.acceleration, move.distance));
    }
    return result;
}

double Planner::retained_distance() const
{
    double result = 0.;
    for (const Pending &pending : m_pending)
        result += pending.move.xyz_distance;
    return result;
}

PlannedMove Planner::planned(std::size_t index, const std::vector<double> &endpoints) const
{
    const Move &move = m_pending[index].move;
    try {
        return {move, solve_phases({move.distance, move.speed_limit, endpoints[index], endpoints[index + 1],
                                   move.acceleration, move.acceleration})};
    } catch (const std::invalid_argument &error) {
        throw std::logic_error("Planner produced infeasible phase endpoints at source_index=" +
                               std::to_string(move.source_index) + ": " + error.what());
    }
}

namespace {

double reachable_speed(double base, double a, double distance)
{
    double result = std::sqrt(base * base + 2. * a * distance);
    while ((result - base) * (result + base) / (2. * a) > distance)
        result = std::nextafter(result, 0.);
    return result;
}

std::int64_t rounded_direction(double delta)
{
    const double scaled = delta * 1000000.;
    const double bound = std::ldexp(1., 63);
    if (!std::isfinite(scaled) || scaled <= -bound || scaled >= bound)
        throw std::invalid_argument("XY direction cannot be represented in nanometres");
    return std::llround(scaled);
}

void validate_move(const Move &move)
{
    if (!std::isfinite(move.distance) || !std::isfinite(move.xyz_distance) ||
        !std::isfinite(move.speed_limit) || !std::isfinite(move.acceleration) ||
        move.distance <= 0. || move.xyz_distance <= 0. || move.xyz_distance > move.distance ||
        move.speed_limit <= 0. || move.acceleration <= 0. ||
        move.direction_x == std::numeric_limits<std::int64_t>::min() ||
        move.direction_y == std::numeric_limits<std::int64_t>::min())
        throw std::invalid_argument("Invalid normalized printing movement");
}

} // namespace
} // namespace nematx_pressure
