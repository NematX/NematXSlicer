///|/ Copyright (c) 2026 NematX
///|/
///|/ Released under the GNU Affero General Public License, version 3 or later.
#pragma once

// Fan timing kernel of the fan speed-up plugin. Plain C++17: no SDK, no host.
//
// Why. A fan needs time to reach its speed. When the print asks for a higher fan
// speed at instant t, the command is better taken into account a little earlier
// ("anticipation"), and a short burst at 100% (a "kick") helps the fan to start.
// This file only decides WHEN each fan speed takes effect, on a timeline of
// seconds. The plugin adapter builds that timeline from the print tree, then
// converts the answer (seconds) into cuts and requests in the tree.
//
// Vocabulary
// - Interval: one stretch of the timeline of a tool group, in execution order
//   (for example a leaf of the print tree). It has a duration in seconds, a kind
//   and maybe a fan request (a percent). Its start is the sum of the durations of
//   the intervals before it.
// - Kind: Cuttable intervals may be split, so a command can start in their middle.
//   NonCuttable ones stay whole: a command due inside one moves to its start.
//   A Barrier is an event that this kernel never crosses and never edits (custom
//   G-code, heating, a fan command of a service event...). A service event sets the
//   target to its request and a barrier that resets the state makes it unknown. A
//   barrier with both leaves it unknown: the firmware writes the request before the
//   opaque G-code.
// - Target: the fan percent asked by the requests. It stays in force until a new
//   request. It is "unknown" at the start of the group if the caller does not know
//   it, and after a barrier that resets the state. The 100% of a pulse is never a
//   target.
// - Transition: a non-barrier interval whose request differs from the target in
//   force (or the first request after an unknown target).
// - Period: what follows a change until the next change or the next barrier. One
//   value is commanded during a period.
// - Anticipation: a rise of the target is moved earlier by the delay, so that the
//   fan has reached its speed when the transition happens. Falls are never moved.
//   The move stops at the "domain" start, the latest of: the start of the group,
//   the end of the last barrier, the end of the last pulse (by its return, or by a
//   fall that interrupted it), and the end of the last earlier period whose target
//   is at least as high, even if that period began before the span. Those periods
//   are the planned ones, the one in force at the group entry and the value set by a
//   service event. So the fan is never commanded below what the requests asked.
//   Lower changes inside the moved span are erased: their intervals now belong to the
//   new period. A change is judged where it is planned NOW: a transition that was
//   already moved earlier than the new span is kept (as in the legacy buffer, where a
//   command already written is no longer erased) and its original interval still
//   falls in the new period.
//   When the domain is shorter than the delay the anticipation is partial; no wait
//   is added and no speed is changed.
// - Pulse (kick): the new target is preceded by 100% for kickstart * delta / 100
//   seconds, delta being the rise in percent. A lower or zero target met during a
//   pulse ends it; a higher one extends it by the extra time its own rise needs (the
//   return never moves earlier). A pulse never crosses a barrier nor the end of the
//   group: it is cut before the barrier (or dropped), or the target is restored after
//   the group.
// - Position: (interval, offset), the offset being seconds from the start of the
//   interval. It is 0 or strictly inside a Cuttable interval, so the adapter knows
//   where to split. Positions are ordered in execution order; several positions can
//   share an instant when zero-duration intervals lie between them.
//
// How it works. plan_fan_timing() walks the intervals once, keeping the list of
// changes planned so far as a stack: a new rise pops the changes it replaces, then
// pushes its own (one change, or a pulse start and a pulse return). Every change
// says "from this position on, the commanded value is percent", with the mode of
// the request of interval `source`. The period of a change lasts until the next
// change or the next barrier; the adapter writes the value on each interval of the
// period that carries a request. With a zero delay and a zero kickstart the changes
// are exactly the original transitions, at offset 0.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <vector>

namespace nematx_fan_speedup {

using std::size_t;

// How the planner may treat an interval; see the vocabulary above.
enum class IntervalKind { Cuttable, NonCuttable, Barrier };

// One stretch of the timeline of a tool group.
struct Interval {
    double duration;              // s, >= 0 (0 allowed: transparent event)
    IntervalKind kind;
    bool overhang;                // overhang role (for overhangs_only)
    std::optional<float> request; // direct active request (percent 0..100), otherwise none
    bool resets_state;            // Barrier only: the target is unknown after it
};

// delay is already normalized (see normalized_delay); 0 turns anticipation off.
struct Settings {
    double delay;
    bool overhangs_only;
    double kickstart; // s for a rise of 100 points; 0 turns the kick off
};

// From (interval, offset) on, the commanded value is percent, with the mode of the
// request of interval `source`.
//
// The start of a pulse can be told from the change alone, and the tree adapter relies on
// it: the planner never opens a pulse towards 100%, so a change with percent 100 either
// is a plain transition to a request of 100% (the request of `source` is 100) or starts
// a pulse (the request of `source` is below 100%). The next change of the list, or
// restore_after, ends that pulse (the return, or the transition that stopped it) or starts
// the next one. The `source` of the return is not always that of the start: a rise during
// the pulse gives the return the source of that rise.
struct Change {
    size_t interval;
    double offset;
    float percent;
    size_t source;
};

// Result of the planning of one tool group. `changes` holds at most one change per
// position (interval, offset): zero-duration intervals can share an instant, each
// keeping its own change. `pulse_shortened` is set when a pulse was dropped, whatever
// the cause (barrier, or swallowed by a non-cuttable interval), or shortened by a
// barrier. That includes a requested extension capped by a barrier while the return
// stays at its current position. It is not set by a pulse that ends after the last
// interval (restore_after), nor by a return that is merely moved back to the start of
// a non-cuttable interval.
struct Schedule {
    std::vector<Change> changes;         // sorted by position
    std::optional<Change> restore_after; // interval = intervals.size(), offset 0
    bool pulse_shortened = false;        // see above
};

// Delay in seconds from the fan_speedup_time option: its absolute value, at least
// 0.01 s when it is not zero (legacy behavior).
inline double normalized_delay(double fan_speedup_time)
{
    if (fan_speedup_time == 0.)
        return 0.;
    return std::max(std::abs(fan_speedup_time), 0.01);
}

namespace detail {

// A point of the timeline: seconds from the start of an interval.
struct Position {
    size_t interval;
    double offset;
};

// Positions are ordered in execution order: by interval, then by offset. Several
// positions can share an instant when zero-duration intervals lie between them.
inline bool operator<(const Position &a, const Position &b)
{
    return a.interval != b.interval ? a.interval < b.interval : a.offset < b.offset;
}

// True for a fan percent within [0, 100] (false for NaN).
inline bool valid_percent(float percent) { return percent >= 0.f && percent <= 100.f; }

// Plain: a transition. PulseStart and PulseReturn: the two changes of one pulse.
enum class Role { Plain, PulseStart, PulseReturn };

// A change of the stack plus what is needed to decide later changes.
struct Planned {
    Position at;
    float percent;
    size_t source;
    Role role;
    std::optional<float> before; // target in force just before this change
};

// State of one plan_fan_timing() call. The stack m_plan always keeps the last
// change of a running pulse (its return) on top, which is how a pulse is found. A
// change that directly follows a pulse start in the stack ends that pulse (the
// return, or the transition that interrupted it).
class Planner
{
public:
    // Checks the inputs, then computes the start time of every interval and, for
    // each interval, the first barrier after it.
    Planner(const std::vector<Interval> &intervals, const Settings &settings, std::optional<float> entry);

    // Walks the intervals in order and plans every transition. A pulse that is
    // still running after the last interval becomes restore_after.
    Schedule run();

private:
    // Plans a transition met while a pulse is running: a fall ends the pulse there,
    // a rise extends it and changes its return value.
    void join_pulse(size_t index);

    // Plans a transition outside of any pulse: anticipation, erasing of the lower
    // changes it replaces, then a pulse when the kick conditions hold.
    void open_period(size_t index);

    // Earliest position where the rise to `target` of interval `index` may start:
    // one delay before the interval, but not before the domain start.
    Position anticipated_start(size_t index, float target) const;

    // Where a pulse started at `start` ends, when it would last until `end_time`
    // seconds. Nothing when the pulse cannot be placed and must be dropped.
    std::optional<Position> place_return(size_t transition, const Position &start, double end_time);

    // Position of an instant of the timeline, 0 <= time < total duration. An instant
    // inside a non-cuttable interval goes to the start of that interval.
    Position position_at(double time) const;

    // Instant of a position, in seconds from the start of the group.
    double time_of(const Position &position) const;

    const std::vector<Interval> &m_intervals; // owned by the caller of plan_fan_timing
    Settings m_settings;
    std::vector<double> m_start;        // start time of each interval, then the total duration
    std::vector<size_t> m_next_barrier; // first barrier after each interval, or the interval count
    std::optional<float> m_current;     // target in force, unknown if empty
    Position m_barrier_end{ 0, 0. };    // end of the last barrier met, or the group start
    std::vector<Planned> m_plan;        // changes planned so far, sorted by position
    bool m_pulse_shortened = false;     // a pulse dropped, or shortened by a barrier (see Schedule)
};

inline Planner::Planner(const std::vector<Interval> &intervals, const Settings &settings, std::optional<float> entry)
    : m_intervals(intervals), m_settings(settings), m_current(entry)
{
    if (!std::isfinite(settings.delay) || settings.delay < 0. || !std::isfinite(settings.kickstart) || settings.kickstart < 0.)
        throw std::invalid_argument("The fan delay and kickstart must be finite and >= 0.");
    if (entry && !valid_percent(*entry))
        throw std::invalid_argument("The entry fan percent must be within [0, 100].");

    m_start.push_back(0.);
    for (const Interval &interval : intervals) {
        if (!std::isfinite(interval.duration) || interval.duration < 0.)
            throw std::invalid_argument("An interval duration must be finite and >= 0.");
        if (interval.request && !valid_percent(*interval.request))
            throw std::invalid_argument("A requested fan percent must be within [0, 100].");
        m_start.push_back(m_start.back() + interval.duration);
    }

    // Filled backwards: the barrier found last is the nearest one after the interval.
    size_t next = intervals.size();
    m_next_barrier.resize(intervals.size());
    for (size_t i = intervals.size(); i-- > 0;) {
        m_next_barrier[i] = next;
        if (intervals[i].kind == IntervalKind::Barrier)
            next = i;
    }
}

inline Schedule Planner::run()
{
    const size_t count = m_intervals.size();
    for (size_t i = 0; i < count; ++i) {
        const Interval &interval = m_intervals[i];
        if (interval.kind == IntervalKind::Barrier) {
            // A barrier is never modified. A service event sets the target; a barrier that
            // resets the state makes it unknown. With both, the target is unknown: the
            // firmware writes the request before the custom G-code or special command.
            if (interval.request)
                m_current = interval.request;
            if (interval.resets_state)
                m_current.reset();
            m_barrier_end = { i + 1, 0. };
        } else if (interval.request && interval.request != m_current) {
            // A pulse is running when the top change is a return placed after this interval.
            if (!m_plan.empty() && m_plan.back().role == Role::PulseReturn && Position{ i, 0. } < m_plan.back().at)
                join_pulse(i);
            else
                open_period(i);
            m_current = interval.request;
        }
    }

    Schedule schedule;
    // A return placed past the last interval cannot be attached to any interval.
    if (!m_plan.empty() && m_plan.back().at.interval == count) {
        schedule.restore_after = Change{ count, 0., m_plan.back().percent, m_plan.back().source };
        m_plan.pop_back();
    }
    for (const Planned &planned : m_plan)
        schedule.changes.push_back({ planned.at.interval, planned.at.offset, planned.percent, planned.source });
    schedule.pulse_shortened = m_pulse_shortened;
    return schedule;
}

inline void Planner::join_pulse(size_t index)
{
    const float target = *m_intervals[index].request;
    Planned &pulse_return = m_plan.back();
    if (target < pulse_return.percent) {
        // The fan must slow down (or stop): the pulse ends where the transition is.
        m_plan.pop_back();
        m_plan.push_back({ { index, 0. }, target, index, Role::Plain, m_current });
        return;
    }
    // Legacy behavior: the pulse lasts as long as the sum of what each rise would have needed.
    const double extra = m_settings.kickstart * (double(target) - double(pulse_return.percent)) / 100.;
    const Position &pulse_start = m_plan[m_plan.size() - 2].at;
    // The transition lies after the pulse start, and the last interval before a
    // barrier is at least the transition itself: the return can always be placed. A
    // barrier may cap it before the current return: a rise never shortens the pulse.
    pulse_return.at = std::max(pulse_return.at, place_return(index, pulse_start, time_of(pulse_return.at) + extra).value());
    pulse_return.percent = target;
    pulse_return.source = index;
    pulse_return.before = target;
}

inline void Planner::open_period(size_t index)
{
    const Interval &interval = m_intervals[index];
    const float target = *interval.request;
    const bool rise = m_current.has_value() && target > *m_current;
    // The delay and the overhang filter decide both the anticipation and the kick
    // threshold, even when the domain leaves no room to move the transition.
    const bool anticipate = m_settings.delay > 0. && (!m_settings.overhangs_only || interval.overhang);

    Position start{ index, 0. };
    if (rise && anticipate)
        start = anticipated_start(index, target);

    // Everything planned from `start` on is replaced by this transition; the earliest
    // change removed tells which target was in force just before `start`. A change
    // already moved earlier than `start` stays, wherever its original interval is.
    std::optional<float> before = m_current;
    while (!m_plan.empty() && !(m_plan.back().at < start)) {
        before = m_plan.back().before;
        m_plan.pop_back();
    }

    // A rise implies 0 < target. The kick needs a known starting point, and a
    // target below 100 (a rise to 100 would not change anything during the pulse).
    if (m_settings.kickstart > 0. && rise && before && target < 100.f) {
        const double delta = double(target) - double(*before);
        if (delta > (anticipate ? 0. : 10.)) {
            const double end_time = time_of(start) + m_settings.kickstart * delta / 100.;
            if (const std::optional<Position> back = place_return(index, start, end_time)) {
                m_plan.push_back({ start, 100.f, index, Role::PulseStart, before });
                m_plan.push_back({ *back, target, index, Role::PulseReturn, target });
                return;
            }
        }
    }
    m_plan.push_back({ start, target, index, Role::Plain, before });
}

inline Position Planner::anticipated_start(size_t index, float target) const
{
    Position start = std::max(position_at(std::max(0., m_start[index] - m_settings.delay)), m_barrier_end);
    // Go back over the changes that are inside the span, from the latest one. Lower
    // ones will be erased by the caller. A change ends the domain when it ends a pulse
    // (it follows its start), or when the period just before it, even one that began
    // before the span, had a target at least as high (`before`: the group entry and
    // the service events are periods too).
    for (size_t k = m_plan.size(); k > 0 && !(m_plan[k - 1].at < start); --k) {
        const Planned &planned = m_plan[k - 1];
        if ((k >= 2 && m_plan[k - 2].role == Role::PulseStart) || (planned.before && *planned.before >= target))
            return planned.at;
    }
    return start;
}

inline std::optional<Position> Planner::place_return(size_t transition, const Position &start, double end_time)
{
    const size_t count = m_intervals.size();
    // m_start[barrier] is the end of the group when there is no barrier.
    const size_t barrier = m_next_barrier[transition];
    Position back{ count, 0. };
    if (end_time < m_start[barrier]) {
        back = position_at(end_time);
    } else if (barrier < count) {
        // The pulse would reach the barrier: it ends at the start of the last interval
        // before it. This also drops the pulse when no interval starts after `start`.
        back = { barrier - 1, 0. };
        m_pulse_shortened = true;
    }
    if (!(start < back)) {
        // Nothing left between the start and the return: the pulse is dropped, whether a barrier
        // left no interval for it or a non-cuttable interval swallowed it.
        m_pulse_shortened = true;
        return std::nullopt;
    }
    return back;
}

inline Position Planner::position_at(double time) const
{
    // First interval start not before `time`.
    const size_t j = std::lower_bound(m_start.begin(), m_start.end(), time) - m_start.begin();
    if (m_start[j] == time)
        return { j, 0. };
    // Otherwise `time` is strictly inside interval j - 1.
    const size_t i = j - 1;
    return { i, m_intervals[i].kind == IntervalKind::Cuttable ? time - m_start[i] : 0. };
}

inline double Planner::time_of(const Position &position) const { return m_start[position.interval] + position.offset; }

} // namespace detail

// Plans the fan changes of one tool group. `intervals` are in execution order,
// `entry` is the target in force when the group starts (empty if unknown).
// Throws std::invalid_argument for a negative or non-finite duration, delay or
// kickstart, or a percent outside [0, 100].
inline Schedule plan_fan_timing(const std::vector<Interval> &intervals, const Settings &settings, std::optional<float> entry)
{
    return detail::Planner(intervals, settings, entry).run();
}

} // namespace nematx_fan_speedup
