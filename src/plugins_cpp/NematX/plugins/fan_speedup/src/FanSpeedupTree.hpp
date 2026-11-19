///|/ Copyright (c) 2026 NematX
///|/
///|/ Released under the GNU Affero General Public License, version 3 or later.
#pragma once

// Tree adapter of the fan speed-up plugin, separated from the plugin so that SDK
// tests can run it on stored entities. The planning itself (WHEN each fan speed
// takes effect) is done by the kernel of FanTimeline.hpp, on a timeline of seconds.
// This file builds that timeline from the print tree and writes the answer back.
//
// How it works, in three steps for each tool group:
// 1. build_timeline() walks the tool group in the order of the G-code writer: the
//    "before" events of the tool group, then every extrusion root, then the "after"
//    events. Inside a root a node is visited before its children, and the events
//    carried by a node (custom G-code, special command, heater, fan request of an
//    empty leaf) happen when the node is entered, so they are classified at that
//    position and never copied onto the descendants. Each element becomes one kernel
//    Interval, and an Origin remembers the node it comes from:
//    - a leaf with segments lasts (3D length, arcs included) / (inherited speed),
//      as in the host time estimator, and can be cut;
//    - when two leaves are separated by at least SCALED_EPSILON, a synthetic travel
//      interval at travel_speed is placed before the second one. It belongs to that
//      leaf (the firmware writes the leaf request before the travel) and cannot be
//      cut. The first gap of a group is ignored: the position at the start of the
//      group is unknown and the layers are processed in parallel;
//    - a leaf carrying a one-shot property (custom G-code, special command, heater,
//      extrusion axis) cannot be cut: the split would repeat the event. A wipe or
//      an approach (extrusion axis) keeps its real duration and does not stop the
//      planner; an extrusion-axis event without segments lasts 0 s;
//    - a barrier is never crossed nor edited: a custom G-code, a special command,
//      a heater, a fan request on an empty leaf (a service event) and a leaf whose
//      E is replaced by a raw EXTRUSION command. Opaque events (custom G-code,
//      special command) also make the fan target unknown afterwards. The events of
//      the tool group are all barriers;
//    - the request of an interval is the fan request in force on its node (the
//      pair is inherited down the tree, as in the firmware); only a request set
//      directly on an empty leaf is an event.
// 2. plan_fan_timing() answers with changes: "from this position, the commanded
//    value is percent, in the mode of the request of interval `source`".
// 3. apply_changes() cuts the leaves with split_leaf_at_lengths() and rewrites the
//    requests: the piece that opens a period receives its pair even without a
//    request, then every leaf of the period that carries a request receives it
//    until the next change or barrier. A pulse still running after the last
//    interval is closed by an empty leaf with a direct request, which the plugin
//    appends to the "after" events of the tool group.
//
// Placement. A fan needs about 100 ms of precision, no more. A change is written at
// a "position": the start of a leaf (a synthetic gap shares the position of the leaf
// that follows it) or a cut inside a cuttable leaf. A cut that would leave a piece
// shorter than k_placement_tolerance is not made, whether the piece lies before it (up
// to the previous cut of the leaf, or its start) or after it (up to the end of the
// leaf). Neither is a cut that the host cannot make (see cut_at). The change then goes
// to the nearest boundary in time: the previous cut or the start of the leaf, or the
// start of what follows. A barrier is not a position where a change can be written:
// when the nearest boundary is a barrier or the end of the group, the change (any
// change, ordinary or the return of a kick) is cut one tolerance before it instead, or
// goes to the start of the leaf if that is not possible. This is within the tolerance
// and is not reported. Changes keep their order: a change cannot precede a position
// already written. Two changes at the same position are resolved:
// - ordinary changes: the later one wins;
// - the start of a kick and its own end (the return, or the transition that stopped
//   it): the end is moved to the next position, a cut one tolerance later in the
//   same leaf if the rest of the leaf lasts at least one more tolerance, otherwise
//   the start of the next leaf. If the next position is after a barrier or the end
//   of the tool group, the kick is dropped (the transition keeps its place, at the
//   target) and the caller is told, to warn once. Only a dropped kick, or a kick that
//   the kernel shortened by a barrier, is reported. A change or a kick moved to a boundary, even
//   a far one (a path that cannot be cut, or has a vertical move), stays silent.
// The durations of the kicks stay those of the kernel, in seconds: only their
// placement is rounded. The kernel does not mark the start of a kick: it is the
// change to 100% whose source request is below 100% (see Change in FanTimeline.hpp),
// because the kernel never opens a kick towards 100%. The change that follows it ends
// the kick, or starts the next one.
//
// Time to length. A cut is a time offset in a leaf at constant speed. The segment
// is found with the cumulative 3D lengths, then the time fraction of that segment is
// converted into a length along the XY path (curved, for an arc) in scaled units, the
// unit of split_leaf_at_lengths(). A purely vertical segment has no XY length and the
// host cannot split a leaf that contains one, wherever the segment stands (a cut before
// it fails as well): such a leaf is never cut and its changes go to a boundary as above
// (see has_vertical_move). The host also refuses pieces of at most SCALED_EPSILON: a cut
// that would leave one is not made either.
//
// Limits. The durations ignore accelerations and scripts inserted after this step
// (layer, height and between-object G-code); the speed is the one of the leaf.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "FanTimeline.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"

namespace nematx_fan_speedup {

using namespace slic3r_api;

// A fan request as it is written in the tree.
struct FanPair
{
    float percent;
    fan_request_mode mode;
};

// The tree node behind one interval, and what is needed to cut it.
struct Origin
{
    // The leaf (or the event node) the interval belongs to.
    ExtrusionEntity node;
    // Mode of the request in force on the node: the mode a change copies from its source.
    fan_request_mode mode = FAN_REQUEST_MODE_UNSPECIFIED;
    // Inherited speed in mm/s, only used to cut a leaf.
    double speed = 0.;
};

// A timeline for the kernel; origins[i] describes intervals[i].
struct Timeline
{
    std::vector<Interval> intervals;
    std::vector<Origin> origins;
};

// Lengths in mm of the segments of a leaf: along the XY path (arcs included) and in
// 3D (with the Z change of the segment).
struct LeafLengths
{
    std::vector<double> planar;
    std::vector<double> spatial;
    double total = 0.; // sum of the 3D lengths
};

// Length in mm of the XY path of a segment; a negative radius selects the long arc,
// as in the firmware and in the host time estimator.
inline double planar_length(const c_extrusion_segment &segment)
{
    const double chord = std::hypot(double(segment.point_b.x) - double(segment.point_a.x),
                                    double(segment.point_b.y) - double(segment.point_a.y));
    if (segment.radius == 0.f)
        return unscaled(chord);
    const double radius = std::abs(double(segment.radius));
    if (radius <= 0. || chord > radius * 2. + double(SCALED_EPSILON))
        throw std::invalid_argument("An extrusion arc has an invalid radius.");
    const double ratio = std::clamp(chord / (2. * radius), 0., 1.);
    double angle = 2. * std::asin(ratio);
    if (segment.radius < 0.f)
        angle = 2. * PI - angle;
    return unscaled(radius * angle);
}

// Lengths of every segment of a leaf. The Z change of a segment is the difference of
// the Z offsets of its end points, the offset inherited from the ancestors cancels out.
inline LeafLengths measure_leaf(const ExtrusionEntity &leaf)
{
    LeafLengths lengths;
    coord_t previous_z = 0;
    for (uint32_t idx = 0; idx < leaf.segment_count(); ++idx) {
        const c_extrusion_segment segment = leaf.segment(idx);
        if (idx == 0)
            previous_z = segment.z_offset_a;
        const double planar = planar_length(segment);
        const double spatial = std::hypot(planar, unscaled(segment.z_offset_b - previous_z));
        previous_z = segment.z_offset_b;
        lengths.planar.push_back(planar);
        lengths.spatial.push_back(spatial);
        lengths.total += spatial;
    }
    return lengths;
}

// Distance in mm along the XY path reached `seconds` after the start of a leaf run at
// a constant speed. The segment is found with the 3D lengths; inside it, the XY length
// is proportional to the 3D length because the speed is constant.
inline double planar_distance_at(const LeafLengths &lengths, double speed, double seconds)
{
    double remaining = seconds * speed;
    double planar = 0.;
    for (size_t idx = 0; idx < lengths.spatial.size(); ++idx) {
        if (remaining <= lengths.spatial[idx])
            return planar + (lengths.spatial[idx] > 0. ? lengths.planar[idx] * remaining / lengths.spatial[idx] : 0.);
        remaining -= lengths.spatial[idx];
        planar += lengths.planar[idx];
    }
    return planar;
}

// Builds the timeline of one or several trees, in the order they are added. With
// `measure` false the geometry is not read: every duration is 0 and there are no gaps,
// but the kinds, the roles and the requests are still collected. That is enough to
// follow the fan target.
class TimelineBuilder
{
public:
    // travel_speed (mm/s) is the speed of the synthetic gaps between leaves.
    TimelineBuilder(bool measure, double travel_speed) : m_measure(measure), m_travel_speed(travel_speed) {}

    // Appends the intervals of a tree. `events` marks the "before" and "after" events of
    // a scope: everything in them is a barrier, because they are never edited.
    void add_tree(const ExtrusionEntity &root, bool events);

    // The intervals collected so far.
    const Timeline &timeline() const { return m_timeline; }

    // Moves the intervals collected so far out of the builder. The position of the
    // previous leaf is kept: do not add trees of another group afterwards.
    Timeline take() { return std::move(m_timeline); }

private:
    // Properties a node inherits from its ancestors, resolved as the firmware does.
    struct Inherited
    {
        float speed = 0.f; // mm/s, 0 until a positive speed is met
        coord_t z_offset = 0;
        std::optional<raw_extrusion_role> role;
        EPropertyFan fan{};
    };

    // A point of the machine path in mm; the layer height is common to the tool group.
    struct Point3
    {
        double x, y, z;
    };

    // Visits a node: its entry events, then its children, then its own geometry.
    void add_node(const ExtrusionEntity &entity, Inherited state);

    // Adds the barrier (or the transparent interval) of the events carried by a node.
    void add_entry_event(const ExtrusionEntity &entity, const Inherited &state, bool event_leaf,
                         std::optional<float> own_request);

    // Adds the interval of a leaf with segments, preceded by its gap when it has one.
    void add_leaf(const ExtrusionEntity &leaf, const Inherited &state, bool raw_extrusion);

    // Adds the synthetic travel from the previous leaf, when the leaves are not joined.
    void add_gap(const ExtrusionEntity &leaf, const Inherited &state, const Point3 &start, bool barrier,
                 const std::optional<float> &request, bool overhang);

    // Stores one interval and its origin.
    void push(Interval interval, const ExtrusionEntity &node, const Inherited &state);

    // False: collect requests and barriers only (see the constructor).
    bool m_measure;
    // Speed of the synthetic gaps, in mm/s.
    double m_travel_speed;
    // The tree being added is a scope event.
    bool m_events = false;
    // End of the previous leaf, unknown at the start of the group.
    std::optional<Point3> m_position;
    // The intervals collected so far.
    Timeline m_timeline;
};

inline void TimelineBuilder::add_tree(const ExtrusionEntity &root, bool events)
{
    m_events = events;
    add_node(root, Inherited{});
}

inline void TimelineBuilder::add_node(const ExtrusionEntity &entity, Inherited state)
{
    // Direct properties apply to the node and to everything below it; the copy of
    // `state` passed to the children restores the parent values when the visit ends.
    if (const EPropertySpeed *speed = entity.get(EPropertySpeed::key); speed != nullptr && speed->speed_mm_per_s > 0.f)
        state.speed = speed->speed_mm_per_s;
    if (const EPropertyZOffset *z_offset = entity.get(EPropertyZOffset::key))
        state.z_offset = z_offset->get();
    if (const EPropertyAttributes *attributes = entity.get(EPropertyAttributes::key))
        state.role = attributes->extrusion_role();
    const EPropertyFan *direct_fan = entity.get(EPropertyFan::key);
    if (direct_fan != nullptr)
        state.fan = direct_fan->resolved_with(state.fan);

    const bool leaf = entity.child_count() == 0;
    const bool geometric = leaf && entity.segment_count() > 0;
    const EPropertySpecialCommand *command = entity.get(EPropertySpecialCommand::key);
    const bool raw_extrusion = geometric && command != nullptr && command->code == C_EXTRUSION_SPECIAL_COMMAND_EXTRUSION;
    // The firmware emits the request of a leaf without segments once, when it enters it.
    const std::optional<float> own_request = leaf && !geometric && direct_fan != nullptr && direct_fan->is_set() ?
        std::optional<float>(direct_fan->percent) : std::nullopt;

    // A raw EXTRUSION leaf is a barrier itself (add_leaf); other events come first.
    if (!raw_extrusion)
        add_entry_event(entity, state, leaf && !geometric, own_request);
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        add_node(entity.child(child_idx), state);
    if (geometric)
        add_leaf(entity, state, raw_extrusion);
}

inline void TimelineBuilder::add_entry_event(const ExtrusionEntity &entity, const Inherited &state, bool event_leaf,
                                             std::optional<float> own_request)
{
    // Custom G-code and special commands are opaque: nothing is known about the
    // fan afterwards. A heater and a fan request keep the target.
    const bool opaque = entity.has(EPropertyCustomGcode::key) || entity.has(EPropertySpecialCommand::key);
    if (opaque || entity.has(EPropertyHeater::key) || own_request)
        push({ 0., IntervalKind::Barrier, false, own_request, opaque }, entity, state);
    else if (event_leaf && entity.has(EPropertyExtrusionAxis::key))
        push({ 0., IntervalKind::NonCuttable, false, std::nullopt, false }, entity, state);
}

inline void TimelineBuilder::add_leaf(const ExtrusionEntity &leaf, const Inherited &state, bool raw_extrusion)
{
    const std::optional<float> request = state.fan.is_set() ? std::optional<float>(state.fan.percent) : std::nullopt;
    // The role selected by fan_speedup_overhangs: a perimeter over a void (overhang perimeter).
    const bool overhang = state.role && RAW_EXTRUSION_ROLE_IS_PERIMETER(*state.role) && RAW_EXTRUSION_ROLE_IS_BRIDGE(*state.role);
    const bool barrier = raw_extrusion || m_events;
    // split_leaf_at_lengths() refuses the leaves with these properties.
    const bool one_shot = leaf.has(EPropertyCustomGcode::key) || leaf.has(EPropertySpecialCommand::key) ||
                          leaf.has(EPropertyHeater::key) || leaf.has(EPropertyExtrusionAxis::key);

    double duration = 0.;
    if (m_measure) {
        if (!(state.speed > 0.f) || !std::isfinite(state.speed))
            throw std::invalid_argument("A geometric leaf has no positive effective speed.");
        duration = measure_leaf(leaf).total / double(state.speed);
        const c_extrusion_segment first = leaf.segment(0);
        const c_extrusion_segment last = leaf.segment(leaf.segment_count() - 1);
        add_gap(leaf, state,
                { unscaled(first.point_a.x), unscaled(first.point_a.y), unscaled(state.z_offset + first.z_offset_a) },
                barrier, request, overhang);
        m_position = Point3{ unscaled(last.point_b.x), unscaled(last.point_b.y), unscaled(state.z_offset + last.z_offset_b) };
    }
    const IntervalKind kind = barrier ? IntervalKind::Barrier : one_shot ? IntervalKind::NonCuttable : IntervalKind::Cuttable;
    push({ duration, kind, overhang, request, raw_extrusion }, leaf, state);
}

inline void TimelineBuilder::add_gap(const ExtrusionEntity &leaf, const Inherited &state, const Point3 &start,
                                     bool barrier, const std::optional<float> &request, bool overhang)
{
    if (!m_position)
        return;
    const double distance = std::sqrt((start.x - m_position->x) * (start.x - m_position->x) +
                                      (start.y - m_position->y) * (start.y - m_position->y) +
                                      (start.z - m_position->z) * (start.z - m_position->z));
    if (distance < unscaled(double(SCALED_EPSILON)))
        return;
    if (!(m_travel_speed > 0.) || !std::isfinite(m_travel_speed))
        throw std::invalid_argument("The fan timing needs a positive travel_speed to time the gaps between paths.");
    // The gap carries the request of the leaf that follows it: the firmware writes
    // that request before the travel to the first point.
    push({ distance / m_travel_speed, barrier ? IntervalKind::Barrier : IntervalKind::NonCuttable, overhang,
           barrier ? std::nullopt : request, false }, leaf, state);
}

inline void TimelineBuilder::push(Interval interval, const ExtrusionEntity &node, const Inherited &state)
{
    if (m_events)
        interval.kind = IntervalKind::Barrier;
    m_timeline.intervals.push_back(interval);
    m_timeline.origins.push_back({ node, state.fan.mode, double(state.speed) });
}

// Timeline of a tool group in the order of the writer: "before" events, extrusion
// roots, "after" events. See TimelineBuilder for `measure`.
inline Timeline build_timeline(const ConstPrintingToolGroup &tool_group, bool measure, double travel_speed)
{
    TimelineBuilder builder(measure, travel_speed);
    const ConstPrintingScopeEvents events = tool_group.events();
    if (events.has_before())
        builder.add_tree(events.before(), true);
    for (uint32_t extrusion_idx = 0; extrusion_idx < tool_group.extrusion_count(); ++extrusion_idx)
        builder.add_tree(tool_group.extrusion(extrusion_idx).root(), false);
    if (events.has_after())
        builder.add_tree(events.after(), true);
    return builder.take();
}

// Fan target (unknown if empty) after the intervals, following the kernel rule: a
// request sets it and an opaque barrier makes it unknown.
inline std::optional<float> target_after(const std::vector<Interval> &intervals, std::optional<float> target)
{
    for (const Interval &interval : intervals) {
        if (interval.request)
            target = interval.request;
        if (interval.kind == IntervalKind::Barrier && interval.resets_state)
            target.reset();
    }
    return target;
}

// Fan target after the "before" or the "after" events of a scope above the tool groups
// (plan, printing group, layer).
inline std::optional<float> target_after_events(const ConstPrintingScopeEvents &events, bool before,
                                                std::optional<float> target)
{
    if (before ? !events.has_before() : !events.has_after())
        return target;
    TimelineBuilder builder(false, 0.);
    builder.add_tree(before ? events.before() : events.after(), true);
    return target_after(builder.timeline().intervals, target);
}

// Position of a tool group in the plan: printing group, layer group, tool group.
using GroupKey = std::array<uint32_t, 3>;

// The fan target in force when each tool group starts, empty when it is unknown. It
// is the last target of the previous tool group of the plan (whatever its layer or
// printing group) if both print with the same extruder; unknown for the first group,
// after a change of extruder, and after an opaque event (custom G-code, special
// command). Scripts inserted after this step are not visible: they are a documented limit.
inline std::map<GroupKey, std::optional<float>> group_entries(const ConstPrintingPlan &plan)
{
    std::map<GroupKey, std::optional<float>> entries;
    std::optional<float> target;
    extruder_index_t previous = EXTRUDER_ID_UNKNOWN;
    target = target_after_events(plan.events(), true, target);
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        const ConstPrintingGroup group = plan.group(group_idx);
        target = target_after_events(group.events(), true, target);
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            const ConstPrintingLayerGroup layer = group.layer_group(layer_idx);
            target = target_after_events(layer.events(), true, target);
            for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
                const ConstPrintingToolGroup tool_group = layer.tool_group(tool_idx);
                const extruder_index_t extruder = tool_group.extruder_id();
                if (extruder == EXTRUDER_ID_UNKNOWN || extruder != previous)
                    target.reset();
                entries[{ group_idx, layer_idx, tool_idx }] = target;
                target = target_after(build_timeline(tool_group, false, 0.).intervals, target);
                previous = extruder;
            }
            target = target_after_events(layer.events(), false, target);
        }
        target = target_after_events(group.events(), false, target);
    }
    return entries;
}

// Smallest piece of leaf, in seconds, that a change may create: the precision a fan
// needs. A named constant so that it can become a setting later.
constexpr double k_placement_tolerance = 0.1;

// True when a duration reaches the tolerance. The margin absorbs the rounding noise of
// the sums of durations; it is far below anything a fan can notice.
inline bool long_enough(double seconds)
{
    return seconds >= k_placement_tolerance - 1e-9;
}

// A change of the schedule located in the tree. `interval` is the interval to edit: the
// first interval of the leaf for a position at its start (see leaf_start), the
// geometry interval for a cut. `offset` is the time of the cut from the start of the
// geometry (0 at a start), `length` the same position along the XY path in scaled
// units (0 at a start). `pulse_start` marks the start of a kick.
struct Placed
{
    size_t interval;
    double offset;
    double length;
    FanPair pair;
    bool pulse_start;
};

// First interval of the leaf an interval belongs to: a synthetic gap shares the
// position of the geometry that follows it. Only a gap and its geometry are two
// consecutive intervals of the same node that are not barriers.
inline size_t leaf_start(const Timeline &timeline, size_t idx)
{
    const bool joined = idx > 0 && timeline.intervals[idx - 1].kind != IntervalKind::Barrier &&
                        timeline.origins[idx - 1].node.same_handle(timeline.origins[idx].node);
    return joined ? idx - 1 : idx;
}

// True when a leaf has a segment without XY length (a lift or a drop: both ends are the
// same XY point). The host cannot split such a leaf: ArcPolyline::split_at asserts that
// no point after the cut coincides in XY with the one before it, so a vertical segment
// anywhere in the leaf (before, at or after the cut) makes every cut of the leaf invalid.
inline bool has_vertical_move(const LeafLengths &lengths)
{
    const double epsilon = double(SCALED_EPSILON);
    return std::any_of(lengths.planar.begin(), lengths.planar.end(),
                       [epsilon](double planar) { return scale_d(planar) <= epsilon; });
}

// A cut `offset` seconds into the geometry interval, or nothing when it is not allowed:
// it must leave at least the tolerance between `left` (the previous cut of the leaf, or
// its start) and the cut, and between the cut and the end of the leaf. It must also be
// a cut the host can make: pieces of the XY path longer than SCALED_EPSILON, in a leaf
// without a vertical move (see has_vertical_move).
inline std::optional<Placed> cut_at(const Timeline &timeline, size_t geometry, double offset, const Placed &left,
                                    const FanPair &pair)
{
    if (!long_enough(offset - left.offset) || !long_enough(timeline.intervals[geometry].duration - offset))
        return std::nullopt;
    const Origin &origin = timeline.origins[geometry];
    const LeafLengths lengths = measure_leaf(origin.node);
    if (has_vertical_move(lengths))
        return std::nullopt;
    const double length = scale_d(planar_distance_at(lengths, origin.speed, offset));
    const double epsilon = double(SCALED_EPSILON);
    if (length - left.length <= epsilon || origin.node.local_length() - length <= epsilon)
        return std::nullopt;
    return Placed{ geometry, offset, length, pair, false };
}

// Where a change of the kernel is written. At the start of an interval: the start of its
// leaf. Inside a cuttable leaf: a cut at its instant when cut_at() accepts it; else the
// nearest boundary in time, which is the previous cut (or the start) of the leaf or the
// start of the next interval. When a barrier or the end of the group follows and the
// end is the nearest, the change stays as close to the end as the tolerance allows: the
// last cut that cut_at() accepts, or else the previous cut (or the start) of the leaf.
inline Placed locate(const Timeline &timeline, const Change &change, const FanPair &pair, const std::vector<Placed> &placed)
{
    const size_t geometry = change.interval;
    const Placed start{ leaf_start(timeline, geometry), 0., 0., pair, false };
    if (change.offset == 0.)
        return start;
    // The boundary on the left: the previous cut of this leaf, or its start.
    Placed left = start;
    if (!placed.empty() && placed.back().interval == geometry) {
        left = placed.back();
        left.pair = pair;
        left.pulse_start = false;
    }
    if (const std::optional<Placed> cut = cut_at(timeline, geometry, change.offset, left, pair))
        return *cut;
    const double duration = timeline.intervals[geometry].duration;
    if (change.offset - left.offset <= duration - change.offset)
        return left;
    const size_t following = geometry + 1;
    if (following < timeline.intervals.size() && timeline.intervals[following].kind != IntervalKind::Barrier)
        return Placed{ following, 0., 0., pair, false };
    const std::optional<Placed> last = cut_at(timeline, geometry, duration - k_placement_tolerance, left, pair);
    return last ? *last : left;
}

// The next position after `from` where a change can be written: a cut one tolerance
// later in the same leaf when the leaf is cuttable and allows it, otherwise the start
// of the next interval. Nothing when that is a barrier or the end of the group.
inline std::optional<Placed> next_position(const Timeline &timeline, const Placed &from, const FanPair &pair)
{
    // `from` is at a start: its geometry is the interval after a gap that shares the position.
    size_t geometry = from.interval;
    if (geometry + 1 < timeline.intervals.size() && leaf_start(timeline, geometry + 1) == geometry)
        ++geometry;
    if (timeline.intervals[geometry].kind == IntervalKind::Cuttable)
        if (const std::optional<Placed> cut = cut_at(timeline, geometry, from.offset + k_placement_tolerance, from, pair))
            return cut;
    const size_t following = geometry + 1;
    if (following < timeline.intervals.size() && timeline.intervals[following].kind != IntervalKind::Barrier)
        return Placed{ following, 0., 0., pair, false };
    return std::nullopt;
}

// True when the change opens a kick: a change to 100% whose source request is below
// 100%. The kernel never opens a kick towards 100%, and a plain change to 100% has a
// source request of 100%. The next change of the schedule ends the kick.
inline bool is_pulse_start(const Timeline &timeline, const Change &change)
{
    const std::optional<float> &target = timeline.intervals[change.source].request;
    return change.percent == 100.f && target && *target < 100.f;
}

// The positions of the schedule in the tree, and whether a kick was dropped.
struct Placement
{
    std::vector<Placed> placed;
    bool dropped = false;
};

// Locates every change of the schedule in the tree, in order, and resolves the changes
// that end at the same position. A change that would precede the previous one takes its
// position. Two ordinary changes: the later one wins. The end of a kick at the position
// of its own start: it is moved to the next position; if there is none (a barrier or the
// end of the group comes first) the kick is dropped, the change that started it keeping
// its place with the target value.
inline Placement place_changes(const Timeline &timeline, const Schedule &schedule)
{
    Placement result;
    std::vector<Placed> &placed = result.placed;
    bool ends_kick = false;
    for (const Change &change : schedule.changes) {
        const FanPair pair{ change.percent, timeline.origins.at(change.source).mode };
        Placed next = locate(timeline, change, pair, placed);
        // A change cannot precede a position already written, which happens when the end of a
        // kick was moved forward: it takes that position and is resolved as a collision there.
        if (!placed.empty() && (next.interval < placed.back().interval ||
                                (next.interval == placed.back().interval && next.offset < placed.back().offset))) {
            next = placed.back();
            next.pair = pair;
        }
        next.pulse_start = is_pulse_start(timeline, change);
        // The change that follows a start ends its kick, unless it starts the next kick.
        const bool ending = ends_kick && !next.pulse_start;
        ends_kick = next.pulse_start;
        if (!placed.empty() && placed.back().interval == next.interval && placed.back().offset == next.offset) {
            if (!ending) {
                placed.back() = next;
                continue;
            }
            const std::optional<Placed> later = next_position(timeline, placed.back(), pair);
            if (!later) {
                placed.back().pair = pair;
                placed.back().pulse_start = false;
                result.dropped = true;
                continue;
            }
            next = *later;
        }
        placed.push_back(next);
    }
    return result;
}

// Writes a direct fan request on a node.
inline void write_pair(MutableExtrusionEntity node, const FanPair &pair)
{
    node.get_or_add(EPropertyFan::key).set(pair.percent, pair.mode);
}

// Applies the schedule to the tree: cuts the leaves and rewrites the requests. The
// intervals are walked once, keeping the pair of the period in force; a barrier ends
// the period. The piece that opens a period receives its pair even without a request;
// a leaf that carries a request receives the pair of the period it is in. The
// collection created by a cut does not keep the request it copied from the leaf: its
// pieces carry the requests. Returns true when a kick was dropped (see Placement).
// The timeline must come from the mutable extrusion roots of a tool group.
inline bool apply_changes(const Timeline &timeline, const Schedule &schedule)
{
    const Placement placement = place_changes(timeline, schedule);
    const std::vector<Placed> &placed = placement.placed;
    std::optional<FanPair> period;
    size_t next = 0;
    for (size_t idx = 0; idx < timeline.intervals.size(); ++idx) {
        if (timeline.intervals[idx].kind == IntervalKind::Barrier) {
            period.reset();
            continue;
        }
        // Changes at the start of the interval: the last one wins. Then the cuts.
        std::optional<FanPair> opening;
        for (; next < placed.size() && placed[next].interval == idx && placed[next].length == 0.; ++next)
            opening = placed[next].pair;
        const size_t first_cut = next;
        while (next < placed.size() && placed[next].interval == idx)
            ++next;
        if (opening)
            period = opening;

        const std::optional<FanPair> first = (opening || timeline.intervals[idx].request) ? period : std::nullopt;
        // The timeline holds borrowed read-only views. The cast is sound because every view of a
        // non-barrier interval comes from an extrusion root of the tool group being edited (its
        // owner is a mutable PrintingExtrusion; edit_tool_group() and the tests are the only
        // callers), and the scope events, which are read-only, were forced to barriers by
        // TimelineBuilder::push() and skipped above.
        MutableExtrusionEntity node(const_cast<extrusion_entity_handle *>(timeline.origins[idx].node.handle()));
        if (first_cut == next) {
            if (first)
                write_pair(node, *first);
            continue;
        }
        std::vector<distf_t> lengths;
        for (size_t cut_idx = first_cut; cut_idx < next; ++cut_idx)
            lengths.push_back(placed[cut_idx].length);
        const std::vector<MutableExtrusionEntity> pieces = node.split_leaf_at_lengths(lengths);
        if (pieces.size() != lengths.size() + 1)
            throw std::runtime_error("The fan timing cut a leaf into an unexpected number of pieces.");
        node.remove(EPropertyFan::key);
        node.disable_reverse();
        // The cut copies the perimeter property onto the collection and onto every piece, and the
        // writer emits one seam marker per node that carries it. The collection keeps it (it starts
        // where the loop started); the pieces do not.
        for (MutableExtrusionEntity piece : pieces)
            piece.remove(EPropertyPerimeter::key);
        if (first)
            write_pair(pieces.front(), *first);
        for (size_t cut_idx = first_cut; cut_idx < next; ++cut_idx)
            write_pair(pieces[cut_idx - first_cut + 1], placed[cut_idx].pair);
        period = placed[next - 1].pair;
    }
    return placement.dropped;
}

// The empty leaf with a direct request that restores the target after the tool group.
inline StoredExtrusionEntity restore_event(storage_handle *storage, const Timeline &timeline, const Change &restore)
{
    StoredExtrusionEntity event(storage);
    event.get_or_add(EPropertyFan::key).set(restore.percent, timeline.origins.at(restore.source).mode);
    return event;
}

// Plans and applies the fan timing of one tool group. `entry` is the target in force at
// its start (empty if unknown). Returns true when a kick was dropped, by the kernel or by the
// placement (see Placement), or shortened by a barrier beyond the placement tolerance. A return
// brought back to within the tolerance before a barrier, and any change or kick moved to a
// boundary (even a far one: a path that cannot be cut, or that has a vertical move), is silent.
inline bool edit_tool_group(storage_handle *storage, PrintingToolGroup &tool_group, const Settings &settings,
                            std::optional<float> entry, double travel_speed)
{
    const Timeline timeline = build_timeline(ConstPrintingToolGroup(tool_group), true, travel_speed);
    const Schedule schedule = plan_fan_timing(timeline.intervals, settings, entry);
    const bool dropped = apply_changes(timeline, schedule);
    if (schedule.restore_after) {
        StoredExtrusionEntity event = restore_event(storage, timeline, *schedule.restore_after);
        if (!tool_group.events().append_after_move(event.mutable_view()).valid())
            throw std::runtime_error("Unable to append the fan restore event to the tool group.");
    }
    return schedule.pulse_shortened || dropped;
}

} // namespace nematx_fan_speedup
