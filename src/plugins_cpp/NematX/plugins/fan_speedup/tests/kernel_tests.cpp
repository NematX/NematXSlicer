// Kernel tests of the fan anticipation and kickstart planner; no host is needed.
//
// Each case builds a small timeline (one interval per line, durations in seconds)
// and compares the whole schedule with explicit expected changes. The start times
// of the intervals are written in the comments to ease the reading of the offsets.
#include <cmath>
#include <cstdio>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "FanTimeline.hpp"

namespace {
using namespace nematx_fan_speedup;

int failures = 0;

// Records a failure, with its message, when the condition is false.
void check(bool condition, const char *message)
{
    if (!condition) {
        std::printf("FAILED: %s\n", message);
        ++failures;
    }
}

// Interval builders; `request` is the direct fan request of the interval, if any.
Interval cut(double duration, std::optional<float> request = std::nullopt, bool overhang = false)
{
    return { duration, IntervalKind::Cuttable, overhang, request, false };
}

Interval whole(double duration, std::optional<float> request = std::nullopt, bool overhang = false)
{
    return { duration, IntervalKind::NonCuttable, overhang, request, false };
}

// A request on a barrier is a service fan event.
Interval barrier(double duration, std::optional<float> request = std::nullopt, bool resets_state = false)
{
    return { duration, IntervalKind::Barrier, false, request, resets_state };
}

// One expected change; same fields as Change.
struct Expected
{
    size_t interval;
    double offset;
    float percent;
    size_t source;
};

// Compares a change with its expected value (offsets within a nanosecond).
bool same(const Change &change, const Expected &expected)
{
    return change.interval == expected.interval && std::abs(change.offset - expected.offset) < 1e-9 &&
           change.percent == expected.percent && change.source == expected.source;
}

// One-line text of a schedule, printed when a case fails.
std::string describe(const Schedule &schedule)
{
    std::string text;
    char line[128];
    for (const Change &change : schedule.changes) {
        std::snprintf(line, sizeof(line), " (%zu, %g, %g, src %zu)", change.interval, change.offset, double(change.percent), change.source);
        text += line;
    }
    if (schedule.restore_after) {
        std::snprintf(line, sizeof(line), " restore (%zu, %g, %g, src %zu)", schedule.restore_after->interval,
                      schedule.restore_after->offset, double(schedule.restore_after->percent), schedule.restore_after->source);
        text += line;
    }
    text += schedule.pulse_shortened ? " shortened" : "";
    return text;
}

// Guarantees of every schedule: sorted strictly increasing positions, offsets 0 or
// strictly inside a Cuttable interval, no change on a barrier.
bool well_formed(const std::vector<Interval> &intervals, const Schedule &schedule)
{
    const Change *previous = nullptr;
    for (const Change &change : schedule.changes) {
        if (change.interval >= intervals.size() || intervals[change.interval].kind == IntervalKind::Barrier)
            return false;
        if (change.offset != 0. && !(intervals[change.interval].kind == IntervalKind::Cuttable && change.offset > 0. &&
                                     change.offset < intervals[change.interval].duration))
            return false;
        if (previous && !(previous->interval < change.interval || (previous->interval == change.interval && previous->offset < change.offset)))
            return false;
        previous = &change;
    }
    return !schedule.restore_after || (schedule.restore_after->interval == intervals.size() && schedule.restore_after->offset == 0.);
}

// Plans and compares the changes, the restore_after and the pulse_shortened flag.
void expect(const char *name, const std::vector<Interval> &intervals, const Settings &settings, std::optional<float> entry,
            const std::vector<Expected> &changes, std::optional<Expected> restore = std::nullopt, bool shortened = false)
{
    const Schedule schedule = plan_fan_timing(intervals, settings, entry);
    bool ok = well_formed(intervals, schedule) && schedule.changes.size() == changes.size() &&
              schedule.restore_after.has_value() == restore.has_value() && schedule.pulse_shortened == shortened;
    for (size_t i = 0; ok && i < changes.size(); ++i)
        ok = same(schedule.changes[i], changes[i]);
    if (ok && restore)
        ok = same(*schedule.restore_after, *restore);
    if (!ok) {
        std::printf("FAILED: %s\n  actual:%s\n", name, describe(schedule).c_str());
        ++failures;
    }
}

// True when plan_fan_timing refuses the inputs with std::invalid_argument.
bool rejected(const std::vector<Interval> &intervals, const Settings &settings, std::optional<float> entry)
{
    try {
        plan_fan_timing(intervals, settings, entry);
    } catch (const std::invalid_argument &) {
        return true;
    }
    return false;
}

// Zero delay and zero kickstart: the changes are the original transitions.
void test_identity()
{
    // Start times: 0, 1, 2, 3, 4, 5, 5, 6, 7.
    const std::vector<Interval> intervals = {
        cut(1, 20.f),     // 0: rise from the entry
        cut(1),           // 1
        whole(1, 20.f),   // 2: same as the target in force, not a transition
        cut(1, 50.f),     // 3
        cut(1, 10.f),     // 4
        barrier(0, 30.f), // 5: service event, target known (30) afterwards
        cut(1, 30.f),     // 6: same as the service value, not a transition
        cut(1, 0.f),      // 7
    };
    const std::vector<Expected> all = { { 0, 0., 20.f, 0 }, { 3, 0., 50.f, 3 }, { 4, 0., 10.f, 4 }, { 7, 0., 0.f, 7 } };
    expect("identity, known entry", intervals, { 0., false, 0. }, 0.f, all);
    expect("identity, overhangs_only changes nothing", intervals, { 0., true, 0. }, 0.f, all);
    expect("identity, unknown entry", intervals, { 0., false, 0. }, std::nullopt, all);
    expect("identity, entry equal to the first request", intervals, { 0., false, 0. }, 20.f,
           { { 3, 0., 50.f, 3 }, { 4, 0., 10.f, 4 }, { 7, 0., 0.f, 7 } });
    expect("nothing to plan", {}, { 1.5, false, 10. }, 0.f, {});
    expect("no request", { cut(1), cut(2) }, { 1.5, false, 10. }, 0.f, {});
}

// Moving a rise earlier by the delay, and the barriers that stop it.
void test_anticipation()
{
    // Start times 0, 1, 3, 4, 5; the rise is on interval 3 (t = 4).
    const std::vector<Interval> plan_example = { cut(1), cut(2), cut(1), cut(1, 70.f) };
    expect("plan example: cut at 2.5 s, 1.5 s into interval 1", plan_example, { 1.5, false, 0. }, 0.f, { { 1, 1.5, 70.f, 3 } });
    expect("delay longer than the group stops at the group start", plan_example, { 10., false, 0. }, 0.f, { { 0, 0., 70.f, 3 } });
    expect("a fall is never anticipated", { cut(1), cut(1, 20.f) }, { 1., false, 0. }, 50.f, { { 1, 0., 20.f, 1 } });

    // Start times 0, 1, 3, 3.2, 3.2, 4, 5; the barrier at 3.2 leaves 0.8 s.
    expect("barrier at 3.2 s limits the anticipation to 0.8 s",
           { cut(1), cut(2), cut(0.2), barrier(0), cut(0.8), cut(1, 70.f) }, { 1.5, false, 0. }, 0.f, { { 4, 0., 70.f, 5 } });
    expect("a barrier with a duration ends the domain at its end",
           { cut(1), cut(2), cut(0.2), barrier(0.2), cut(0.6), cut(1, 70.f) }, { 1.5, false, 0. }, 0.f, { { 4, 0., 70.f, 5 } });

    // The state is unknown after the reset barrier: no anticipation (it would stop at (1, 0)) and no kick.
    expect("reset barrier: first transition after it is neither anticipated nor kicked",
           { barrier(1, std::nullopt, true), cut(1), cut(1, 60.f) }, { 5., false, 10. }, 0.f, { { 2, 0., 60.f, 2 } });
    expect("same timeline without reset: anticipated to the barrier end and kicked",
           { barrier(1), cut(1), cut(1, 60.f) }, { 5., false, 10. }, 0.f, { { 1, 0., 100.f, 2 } }, Expected{ 3, 0., 60.f, 2 });
}

// Lower transitions replaced by a rise, and higher periods that bound the domain.
void test_erasing()
{
    // Start times 0, 1, 2, 3, 4. Rises to 50 at t = 3 with a delay of 3 s: 20 and 10 are lower, erased.
    expect("lower transitions in the window are erased", { cut(1, 20.f), cut(1, 10.f), cut(1), cut(1, 50.f) }, { 3., false, 0. }, 0.f,
           { { 0, 0., 50.f, 3 } });
    // 60 is higher than 40: its period is kept and the domain starts at its end (t = 1), where 20 was.
    expect("an earlier higher period is kept, the domain starts at its end", { cut(1, 60.f), cut(1, 20.f), cut(1), cut(1, 40.f) },
           { 3., false, 0. }, 0.f, { { 0, 0., 60.f, 0 }, { 1, 0., 40.f, 3 } });
    expect("an earlier period of the same target also ends the domain", { cut(1, 60.f), cut(1, 20.f), cut(1), cut(1, 60.f) },
           { 3., false, 0. }, 0.f, { { 0, 0., 60.f, 0 }, { 1, 0., 60.f, 3 } });

    // The 30 at t = 2 is moved to 0.5 s, before the window [1.5 s, 3 s) of the 50. A change is judged
    // where it is planned now, not where its interval is: it stays (its interval joins the 50 period).
    expect("an anticipated change outside of the window of a later rise is kept", { cut(2), cut(1, 30.f), cut(1, 50.f), cut(1) },
           { 1.5, false, 0. }, 0.f, { { 0, 0.5, 30.f, 1 }, { 0, 1.5, 50.f, 2 } });

    // Start times 0, 2, 3, 4. The window of the 40 (t = 3, delay 2) is [1 s, 3 s). The 60 began before it
    // and lasts until t = 2: the 40 cannot start before, or the fan would drop from 60 to 40 at t = 1.
    expect("a higher period that began before the window ends the domain at its end", { cut(2, 60.f), cut(1, 20.f), cut(1, 40.f) },
           { 2., false, 0. }, 0.f, { { 0, 0., 60.f, 0 }, { 1, 0., 40.f, 2 } });
    // Start times 0, 1, 2, 4, 5. The window of the 50 (t = 4, delay 3) is [1 s, 4 s): the 60 lasts until t = 2.
    expect("same, with the lower change further away", { cut(1, 60.f), cut(1), cut(2, 20.f), cut(1, 50.f) }, { 3., false, 0. }, 0.f,
           { { 0, 0., 60.f, 0 }, { 2, 0., 50.f, 3 } });
    // Start times 0, 1, 2, 3, 4. The entry period (60) has no change of its own: it ends where the 20 starts.
    expect("the period in force at the entry ends the domain at its end", { cut(1), cut(1, 20.f), cut(1), cut(1, 50.f) },
           { 3., false, 0. }, 60.f, { { 1, 0., 50.f, 3 } });
    // Start times 0, 1, 1, 2, 3, 4, 5. The service event sets 80 at t = 1, and that value lasts until the 20 at
    // t = 2 (interval 3). The window of the 50 (t = 4, delay 5) reaches the end of the barrier (interval 2), but
    // the 80 period is not lower: the domain starts where it ends, at the 20.
    expect("the value of a service event bounds the domain", { cut(1), barrier(0, 80.f), cut(1), cut(1, 20.f), cut(1), cut(1, 50.f) },
           { 5., false, 0. }, std::nullopt, { { 3, 0., 50.f, 5 } });
    // Witness: the 70 is higher than the entry period, so it is anticipated to the group start and kicked
    // from 60 (delta 10: 1 s).
    expect("a rise above the entry period is anticipated and kicked", { cut(1), cut(1, 20.f), cut(1), cut(1, 70.f), cut(5) },
           { 3., false, 10. }, 60.f, { { 0, 0., 100.f, 3 }, { 1, 0., 70.f, 3 } });
}

// Where an instant lands: inside a cuttable interval, a whole one, or zero-duration ones.
void test_placement()
{
    // Start times 0, 1, 3, 4, 5. The instant 2.5 s is inside the NonCuttable interval 1.
    expect("an instant inside a non-cuttable interval goes to its start", { cut(1), whole(2), cut(1), cut(1, 70.f) },
           { 1.5, false, 0. }, 0.f, { { 1, 0., 70.f, 3 } });
    // Start times 0, 1, 1, 2, 3. The instant 1 s is the start of the zero-duration interval 1.
    expect("a zero-duration interval can hold an instant", { cut(1), cut(0), cut(1), cut(1, 60.f) }, { 1., false, 0. }, 0.f,
           { { 1, 0., 60.f, 3 } });
    expect("a zero-duration interval can hold a transition", { cut(1), cut(0, 30.f), cut(1), cut(1, 50.f) }, { 0., false, 0. }, 0.f,
           { { 1, 0., 30.f, 1 }, { 3, 0., 50.f, 3 } });
    expect("a zero-duration transition is anticipated inside the interval before", { cut(1), cut(0, 30.f), cut(1), cut(1, 50.f) },
           { 0.5, false, 0. }, 0.f, { { 0, 0.5, 30.f, 1 }, { 2, 0.5, 50.f, 3 } });
}

// The overhang filter, for the anticipation and for the kick threshold.
void test_overhangs_only()
{
    // Start times 0, 1, 2. The transition is on interval 2 (t = 2).
    expect("overhangs_only: no anticipation for another role", { cut(1), cut(1), cut(1, 70.f, false) }, { 1.5, true, 0. }, 0.f,
           { { 2, 0., 70.f, 2 } });
    expect("overhangs_only: anticipation for an overhang", { cut(1), cut(1), cut(1, 70.f, true) }, { 1.5, true, 0. }, 0.f,
           { { 0, 0.5, 70.f, 2 } });

    // Delta 5 / 15 with a kick of 10 s per 100 points; start times 0, 3, 6.
    const Settings settings = { 2., true, 10. };
    expect("overhangs_only, other role, delta 5: below the 10 point threshold", { cut(3), cut(3, 25.f, false) }, settings, 20.f,
           { { 1, 0., 25.f, 1 } });
    expect("overhangs_only, other role, delta 15: kick without anticipation", { cut(3), cut(3, 35.f, false) }, settings, 20.f,
           { { 1, 0., 100.f, 1 }, { 1, 1.5, 35.f, 1 } });
    expect("overhangs_only, overhang, delta 5: anticipated, so any rise is kicked", { cut(3), cut(3, 25.f, true) }, settings, 20.f,
           { { 0, 1., 100.f, 1 }, { 0, 1.5, 25.f, 1 } });
}

// Known and unknown targets: group entry and service fan events.
void test_entry_and_service()
{
    expect("known entry: the first rise is anticipated", { cut(1), cut(1, 40.f) }, { 1., false, 0. }, 0.f, { { 0, 0., 40.f, 1 } });
    expect("unknown entry: the first transition is not anticipated", { cut(1), cut(1, 40.f) }, { 1., false, 0. }, std::nullopt,
           { { 1, 0., 40.f, 1 } });

    // Start times 0, 1, 2, 3, 4. Unknown entry: no anticipation nor kick for the 40, both for the 80.
    const std::vector<Interval> two_rises = { cut(1), cut(1, 40.f), cut(1), cut(1, 80.f) };
    expect("unknown entry then a known rise: anticipated and kicked", two_rises, { 1., false, 10. }, std::nullopt,
           { { 1, 0., 40.f, 1 }, { 2, 0., 100.f, 3 } }, Expected{ 4, 0., 80.f, 3 });
    // The window reaches the 40, erased: nothing is known before the new start, so no delta, no kick.
    expect("unknown target before the anticipated start: no kick", two_rises, { 2., false, 10. }, std::nullopt, { { 1, 0., 80.f, 3 } });

    // Start times 0, 1, 1, 2, 3, 4. The service event makes the target known (30) and ends the domain.
    const std::vector<Interval> service = { cut(1), barrier(0, 30.f), cut(1), cut(1, 30.f), cut(1, 50.f) };
    expect("service fan event: known target, domain starts after it", service, { 5., false, 0. }, std::nullopt, { { 2, 0., 50.f, 4 } });
    expect("service fan event: the kick starts from its value", service, { 5., false, 10. }, std::nullopt,
           { { 2, 0., 100.f, 4 }, { 4, 0., 50.f, 4 } });

    // The firmware writes the request, then the custom G-code, which may change the fan: the target is unknown.
    expect("a barrier with a request and a reset leaves the target unknown", { barrier(0, 30.f, true), cut(1, 30.f) }, { 0., false, 0. },
           std::nullopt, { { 1, 0., 30.f, 1 } });
    // Control: without the reset the same request is already in force.
    expect("a barrier with a request alone makes the same request redundant", { barrier(0, 30.f), cut(1, 30.f) }, { 0., false, 0. },
           std::nullopt, {});
}

// When a pulse is started, how long it lasts and where it returns.
void test_kick_conditions()
{
    const Settings no_delay = { 0., false, 10. };
    // Start times 0, 1, 4 (or 2).
    expect("no anticipation, delta 5: no kick", { cut(1), cut(3, 25.f) }, no_delay, 20.f, { { 1, 0., 25.f, 1 } });
    expect("no anticipation, delta 10: no kick, the threshold is strict", { cut(1), cut(3, 30.f) }, no_delay, 20.f, { { 1, 0., 30.f, 1 } });
    expect("no anticipation, delta 15: kick of 1.5 s", { cut(1), cut(3, 35.f) }, no_delay, 20.f,
           { { 1, 0., 100.f, 1 }, { 1, 1.5, 35.f, 1 } });
    expect("anticipation, delta 5: kick of 0.5 s from the anticipated start", { cut(2), cut(3, 25.f) }, { 1., false, 10. }, 20.f,
           { { 0, 1., 100.f, 1 }, { 0, 1.5, 25.f, 1 } });
    // Start times 0, 0, 1, 6. The barrier right before leaves no room to move the transition, but the delay
    // still makes it an anticipated one: delta 5 is kicked, for 0.5 s.
    expect("no room to anticipate, delta 5: kicked", { barrier(0), cut(1, 25.f), cut(5) }, { 1., false, 10. }, 20.f,
           { { 1, 0., 100.f, 1 }, { 1, 0.5, 25.f, 1 } });
    expect("target 100: no kick", { cut(1), cut(3, 100.f) }, no_delay, 20.f, { { 1, 0., 100.f, 1 } });
    expect("target 0: no kick", { cut(1), cut(1, 0.f) }, no_delay, 50.f, { { 1, 0., 0.f, 1 } });
    expect("kickstart 0: no kick", { cut(1), cut(3, 80.f) }, { 0., false, 0. }, 20.f, { { 1, 0., 80.f, 1 } });

    // The anticipated start is placed at (1, 0), 1 s: the kick of 3 s ends at 4 s, start of interval 3.
    // Counted from the unplaced instant 2.5 s it would end after the group (5 s).
    expect("the kick is counted from the placed start", { cut(1), whole(2), cut(1), cut(1, 50.f) }, { 1.5, false, 6. }, 0.f,
           { { 1, 0., 100.f, 3 }, { 3, 0., 50.f, 3 } });
    // Start times 0, 1, 2, 4, 5. The return at 2.5 s is inside the NonCuttable interval 2.
    expect("a return inside a non-cuttable interval goes to its start", { cut(1), cut(1, 35.f), whole(2), cut(1) }, no_delay, 20.f,
           { { 1, 0., 100.f, 1 }, { 2, 0., 35.f, 1 } });
    // Start times 0, 1, 5. The whole pulse is inside the NonCuttable interval 1: nothing is left.
    expect("a pulse swallowed by a non-cuttable interval is dropped", { cut(1), whole(4, 35.f) }, no_delay, 20.f,
           { { 1, 0., 35.f, 1 } }, std::nullopt, true);
}

// Transitions met during a pulse, and pulses that end a domain.
void test_pulse_updates()
{
    const Settings settings = { 0., false, 10. };
    // Start times 0..6. 20 asks for 2 s of pulse, the 40 at t = 1 adds 10 * (40 - 20) / 100 = 2 s: 4 s in all.
    expect("a rise during a pulse extends it", { cut(1, 20.f), cut(1, 40.f), cut(1), cut(1), cut(1), cut(1) }, settings, 0.f,
           { { 0, 0., 100.f, 0 }, { 4, 0., 40.f, 1 } });
    // The pulse of 5 s lasts until the end of the group; the 30 at t = 1 stops it.
    expect("a fall during a pulse ends it", { cut(1, 50.f), cut(1, 30.f), cut(1), cut(1), cut(1) }, settings, 0.f,
           { { 0, 0., 100.f, 0 }, { 1, 0., 30.f, 1 } });
    expect("zero during a pulse ends it", { cut(1, 50.f), cut(1, 0.f) }, settings, 0.f, { { 0, 0., 100.f, 0 }, { 1, 0., 0.f, 1 } });
    // The pulse ends exactly at the transition at t = 2: that transition replaces the return and is kicked from 20.
    expect("a transition at the end of a pulse replaces the return", { cut(1, 20.f), cut(1), cut(1, 50.f) }, settings, 0.f,
           { { 0, 0., 100.f, 0 }, { 2, 0., 100.f, 2 } }, Expected{ 3, 0., 50.f, 2 });
    // Start times 0, 4, 5, 6, 7. The pulse of the 20 (2 s from 2 s) ends at t = 4: the 60 cannot be anticipated before it.
    expect("the end of a pulse ends the domain of the next rise", { cut(4), cut(1, 20.f), cut(1, 60.f), cut(1) }, { 2., false, 10. }, 0.f,
           { { 0, 2., 100.f, 1 }, { 1, 0., 100.f, 2 } }, Expected{ 4, 0., 60.f, 2 });
    // Start times 0, 3, 5, 6, 7. The pulse of the 20 (4 s) is interrupted by the 10 at t = 3. The window of the
    // 70 (t = 5, delay 3) starts at 2 s, inside that pulse: the domain starts at t = 3, where the pulse ended.
    // The kick of the 70 starts from the 20 (delta 50: 10 s), past the group end.
    expect("a pulse interrupted by a fall ends the domain, even if it began before the window",
           { cut(3, 20.f), cut(2, 10.f), cut(1, 70.f), cut(1) }, { 3., false, 20. }, 0.f,
           { { 0, 0., 100.f, 0 }, { 1, 0., 100.f, 2 } }, Expected{ 4, 0., 70.f, 2 });
    // Start times 0, 2, 4, 5. The pulse of the 60 (6 s) is interrupted by the 20 at t = 2. The window of the 70
    // (t = 4, delay 3) starts at 1 s: the domain starts at t = 2, and the kick is delta 70 - 60 = 10, 1 s.
    expect("a pulse interrupted by a fall: the next kick starts where it ended", { cut(2, 60.f), cut(2, 20.f), cut(1, 70.f) },
           { 3., false, 10. }, 0.f, { { 0, 0., 100.f, 0 }, { 1, 0., 100.f, 2 }, { 1, 1., 70.f, 2 } });
    // Start times 0, 1, 2, 3, 4, 9. The 60 is kicked for 6 s, the 20 at t = 1 interrupts the pulse. The 60 at
    // t = 3 (window from 0) replaces the 20: the domain starts at t = 1 and the target before it is 60, so
    // delta is 0: no pulse, and nothing is dropped.
    expect("delta 0 after an interrupted pulse: no kick", { cut(1, 60.f), cut(1, 20.f), cut(1), cut(1, 60.f), cut(5) },
           { 3., false, 10. }, 0.f, { { 0, 0., 100.f, 0 }, { 1, 0., 60.f, 3 } });
}

// Pulses reaching a barrier or the end of the group.
void test_pulse_limits()
{
    const Settings settings = { 0., false, 10. };
    expect("a pulse running after the last interval is restored after the group", { cut(1, 50.f), cut(1) }, settings, 0.f,
           { { 0, 0., 100.f, 0 } }, Expected{ 2, 0., 50.f, 0 });
    // Start times 0, 1, 11, 12. The pulse of 0.5 s ends inside interval 0.
    expect("a pulse ending before a barrier is not shortened", { cut(1, 50.f), cut(10), barrier(1) }, { 0., false, 1. }, 0.f,
           { { 0, 0., 100.f, 0 }, { 0, 0.5, 50.f, 0 } });
    // Start times 0, 1, 2, 3, 4, 5. The pulse of 5 s would cross the barrier at 3 s: it returns at the start of interval 2.
    expect("a pulse crossing a barrier is shortened to the last interval before it", { cut(1, 50.f), cut(1), cut(1), barrier(1), cut(1) },
           settings, 0.f, { { 0, 0., 100.f, 0 }, { 2, 0., 50.f, 0 } }, std::nullopt, true);
    // No interval starts between the pulse start and the barrier: the pulse is dropped, the transition stays.
    expect("a pulse crossing a barrier with no interval left is dropped", { cut(1, 50.f), barrier(1), cut(1) }, settings, 0.f,
           { { 0, 0., 50.f, 0 } }, std::nullopt, true);
    // Start times 0, 1, 2, 3, 4, 5. After the extension (2 s + 2 s) the pulse would reach the barrier at 4 s.
    expect("an extension reaching a barrier is shortened", { cut(1, 20.f), cut(1, 40.f), cut(1), cut(1), barrier(1) }, settings, 0.f,
           { { 0, 0., 100.f, 0 }, { 3, 0., 40.f, 1 } }, std::nullopt, true);
    // Start times 0, 1, 1.1, 5, 6. The pulse of the 20 lasts 4.8 s: it returns 3.7 s into interval 2. The 40 at
    // t = 1 would add 4.8 s, past the barrier at 5 s; the capped return (start of interval 2, 1.1 s) would be
    // before the current one. A rise never shortens a pulse: the return stays where it was.
    expect("an extension capped by a barrier never moves the return earlier", { cut(1., 20.f), cut(0.1, 40.f), cut(3.9), barrier(1) },
           { 0., false, 24. }, 0.f, { { 0, 0., 100.f, 0 }, { 2, 3.7, 40.f, 1 } }, std::nullopt, true);
}

// For each change, whether it starts a pulse as the tree adapter recognizes it: a change to
// 100% whose source request is below 100%.
std::vector<bool> pulse_starts(const std::vector<Interval> &intervals, const Schedule &schedule)
{
    std::vector<bool> out;
    for (const Change &change : schedule.changes)
        out.push_back(change.percent == 100.f && intervals[change.source].request && *intervals[change.source].request < 100.f);
    return out;
}

// The start of a pulse is told from its change alone (see Change in FanTimeline.hpp): the
// planner never opens a pulse towards 100%.
void test_pulse_start_marker()
{
    const Settings settings = { 0., false, 10. };
    {
        // A plain transition to 100% is not a pulse: its source request is 100.
        const std::vector<Interval> intervals = { cut(1), cut(1, 100.f) };
        const Schedule schedule = plan_fan_timing(intervals, settings, 20.f);
        check(schedule.changes.size() == 1 && schedule.changes[0].percent == 100.f && !schedule.restore_after &&
                  pulse_starts(intervals, schedule) == std::vector<bool>{ false },
              "a transition to 100% is not the start of a pulse");
    }
    {
        // A pulse and its return.
        const std::vector<Interval> intervals = { cut(1, 50.f), cut(10) };
        const Schedule schedule = plan_fan_timing(intervals, { 0., false, 1. }, 0.f);
        check(schedule.changes.size() == 2 && pulse_starts(intervals, schedule) == std::vector<bool>{ true, false },
              "a pulse starts with a change to 100%");
    }
    {
        // A rise during the pulse gives the return the source of that rise: the start cannot be
        // recognized by a return of the same source.
        const std::vector<Interval> intervals = { cut(1, 20.f), cut(1, 40.f), cut(1), cut(1), cut(1), cut(1) };
        const Schedule schedule = plan_fan_timing(intervals, settings, 0.f);
        check(schedule.changes.size() == 2 && schedule.changes[0].source != schedule.changes[1].source &&
                  pulse_starts(intervals, schedule) == std::vector<bool>{ true, false },
              "an extended pulse still starts with a change to 100% of a source below 100%");
    }
    {
        // A transition at the end of a pulse replaces its return and starts the next pulse.
        const std::vector<Interval> intervals = { cut(1, 20.f), cut(1), cut(1, 50.f) };
        const Schedule schedule = plan_fan_timing(intervals, settings, 0.f);
        check(schedule.changes.size() == 2 && schedule.restore_after && pulse_starts(intervals, schedule) == std::vector<bool>{ true, true },
              "a pulse can follow the start of another pulse");
    }
}

// Invalid inputs are rejected.
void test_validation()
{
    const std::vector<Interval> valid = { cut(1, 20.f), cut(1) };
    const double infinity = std::numeric_limits<double>::infinity();
    check(!rejected(valid, { 1., false, 1. }, 10.f), "valid inputs are accepted");
    check(rejected({ cut(-1) }, { 0., false, 0. }, std::nullopt), "negative duration");
    check(rejected({ cut(std::nan("")) }, { 0., false, 0. }, std::nullopt), "NaN duration");
    check(rejected({ cut(infinity) }, { 0., false, 0. }, std::nullopt), "infinite duration");
    check(rejected({ cut(1, 100.5f) }, { 0., false, 0. }, std::nullopt), "request above 100");
    check(rejected({ cut(1, -1.f) }, { 0., false, 0. }, std::nullopt), "negative request");
    check(rejected({ barrier(1, 101.f) }, { 0., false, 0. }, std::nullopt), "service request above 100");
    check(rejected(valid, { 0., false, 0. }, 101.f), "entry above 100");
    check(rejected(valid, { -1., false, 0. }, std::nullopt), "negative delay");
    check(rejected(valid, { std::nan(""), false, 0. }, std::nullopt), "NaN delay");
    check(rejected(valid, { 0., false, -1. }, std::nullopt), "negative kickstart");
    check(rejected(valid, { 0., false, infinity }, std::nullopt), "infinite kickstart");
}

// Normalization of fan_speedup_time into a delay.
void test_normalized_delay()
{
    check(normalized_delay(-2.) == 2., "-2 -> 2");
    check(normalized_delay(2.) == 2., "2 -> 2");
    check(normalized_delay(0.005) == 0.01, "0.005 -> 0.01");
    check(normalized_delay(-0.005) == 0.01, "-0.005 -> 0.01");
    check(normalized_delay(0.3) == 0.3, "0.3 -> 0.3");
    check(normalized_delay(0.) == 0., "0 -> 0");
}
} // namespace

int main()
{
    test_identity();
    test_anticipation();
    test_erasing();
    test_placement();
    test_overhangs_only();
    test_entry_and_service();
    test_kick_conditions();
    test_pulse_updates();
    test_pulse_limits();
    test_pulse_start_marker();
    test_validation();
    test_normalized_delay();
    if (failures == 0)
        std::printf("fan speed-up kernel tests passed\n");
    return failures == 0 ? 0 : 1;
}
