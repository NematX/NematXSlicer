///|/ Released under the GNU Affero General Public License, version 3 or later.

// Plans cycles of already classified printing movements in dominant-axis units.
// Supply signed XYZ deltas and nonnegative filament delta in mm to normalize_move,
// then pass its owned Move to Planner::push. Read stable movements from each
// Publication and call finish at a cycle boundary to receive the remainder.
// Speeds are mm/s and accelerations mm/s^2. source_index is an opaque caller ID.
// Initialize every AxisLimits field before normalization; use the resulting
// fully initialized Move, or initialize every Move field before passing it on.
// No G-code, pressure correction, configuration, or controller state is accessed.
#pragma once

#include "PressureKinematics.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

namespace nematx_pressure {

// Axis order is X, Y, Z, E; nonpositive speed/acceleration entries mean 100000.
// A nonpositive max_extruding_acceleration disables the extra XYZ acceleration cap.
struct AxisLimits
{
    std::array<double, 4> speed;
    std::array<double, 4> acceleration;
    double max_extruding_acceleration = 0.; // Positive values cap XYZ accelerations only.
};

// Owns only the normalized data needed to plan one source printing movement.
struct Move
{
    double distance;
    double xyz_distance;
    double speed_limit;
    double acceleration;
    std::int64_t direction_x;
    std::int64_t direction_y;
    std::size_t source_index;
};

// Projects finite deltas, feed and acceleration through axis limits; throws
// invalid_argument for invalid inputs, E-only moves, or unrepresentable results.
Move normalize_move(double dx, double dy, double dz, double de, double feed,
                    double acceleration, const AxisLimits &limits, std::size_t source_index);

// Returns the historical angular factor in [0,1] from valid normalized XY directions.
// A zero XY direction gives zero; straight directions give exactly one.
double junction_factor(const Move &previous, const Move &next);

// A stable source movement with an owned analytical profile.
struct PlannedMove
{
    Move move;
    Profile profile;
};

// Owns the stable prefix; the flag reports retained data exceeding both horizons.
struct Publication
{
    std::vector<PlannedMove> moves;
    bool horizon_insufficient = false;
};

// One independent printing cycle; retained values never borrow caller storage.
class Planner
{
public:
    // Validates before insertion and stores the new cap from the preceding attained
    // endpoint. Publishes only whole moves whose endpoints fit the stopped suffix,
    // without replacing open speeds. Invalid moves throw invalid_argument;
    // violated internal reachability throws logic_error rather than lowering a published entry.
    // After logic_error, the caller must abandon this instance and fail processing.
    Publication push(const Move &move);
    // Solves the stopped remainder into owned results, then resets the entire cycle.
    // An empty cycle returns an empty publication. Internal failures throw logic_error
    // and leave the cycle retained rather than returning a partially closed result.
    Publication finish();

private:
    struct Pending
    {
        Move move;
        double entry_cap;
    };

    // Applies forward reachability after the backward envelope and keeps the published
    // entry exactly; an infeasible fixed entry throws logic_error instead of changing it.
    std::vector<double> speeds(bool stopped) const;
    // Propagates v_start <= sqrt(v_end^2+2*a*D) from hard caps; closure and the
    // separate publication proof use identical operation order and never change caps.
    std::vector<double> backward(bool stopped) const;
    // Sums retained dominant XYZ distances without cumulative subtraction error.
    double retained_distance() const;
    // Solves an owned profile with shared endpoint doubles and equal a/b. Invalid
    // solver data indicates an internal error and is rethrown as logic_error.
    PlannedMove planned(std::size_t index, const std::vector<double> &endpoints) const;

    std::deque<Pending> m_pending;
    double m_entry = 0.;
};

} // namespace nematx_pressure
