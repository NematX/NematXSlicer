// Copyright (c) 2026 NematX. AGPL-3.0-or-later.
// Run the fan speed-up tree adapter on stored entities through the public SDK,
// linked to the existing host library.
//
// Every case builds a small tree of straight leaves along the X axis (one leaf per
// 10 mm, a speed set on the root so that it is inherited) and runs the adapter as the
// plugin does: build the timeline, plan it with the kernel, apply the schedule. The
// durations are written in the comments: a 10 mm leaf lasts 0.2 s at 50 mm/s and
// 0.1 s at 100 mm/s. The checks read the requests and the lengths of the leaves of
// the tree afterwards.
#include <cmath>
#include <exception>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "FanSpeedupTree.hpp"

namespace api = slic3r_api;
using namespace nematx_fan_speedup;

namespace {
// Fails the test, with its message, when the condition is false.
void check(bool condition, const char *message)
{
    if (!condition)
        throw std::runtime_error(message);
}

// True when two values differ by at most the tolerance.
bool near(double lhs, double rhs, double tolerance) { return std::abs(lhs - rhs) <= tolerance; }

// A point from millimeters.
c_point cp(double x, double y) { return { scale_i(x), scale_i(y) }; }

// Two request modes to tell the copy of a pair from the copy of a percentage, and two roles.
constexpr fan_request_mode k_auto = FAN_REQUEST_MODE_AUTOMATIC;
constexpr fan_request_mode k_override = FAN_REQUEST_MODE_OVERRIDE;
constexpr raw_extrusion_role k_infill = RAW_EXTRUSION_ROLE_INTERNAL_INFILL;
constexpr raw_extrusion_role k_overhang = RAW_EXTRUSION_ROLE_OVERHANG_PERIMETER;

// A direct request of a leaf, as the tests write and read it.
struct Request
{
    float percent;
    fan_request_mode mode;
};

// What is left of a leaf after the adapter: its direct request (percent < 0: none) and length.
struct Piece
{
    float percent;
    fan_request_mode mode;
    double length_mm;
};

// A root with leaves along the X axis. Leaves are identified by their index in the root,
// because borrowed views must not be kept while the root grows.
class Scene
{
public:
    explicit Scene(storage_handle *storage, float speed = 50.f) : m_storage(storage), m_root(storage)
    {
        m_root.get_or_add(api::EPropertySpeed::key).speed(speed);
    }

    // A deposition leaf from x0 to x1 (mm) at y = 0.
    size_t line(double x0, double x1, std::optional<Request> request, raw_extrusion_role role = k_infill)
    {
        return add(api::StoredExtrusionEntity(m_storage, std::vector<c_point>{ cp(x0, 0), cp(x1, 0) }), request, role);
    }

    // An empty leaf: an event holder.
    size_t event(std::optional<Request> request = std::nullopt)
    {
        return add(api::StoredExtrusionEntity(m_storage), request, 0);
    }

    // Any leaf built by the caller.
    size_t add(api::StoredExtrusionEntity leaf, std::optional<Request> request, raw_extrusion_role role)
    {
        if (role != 0)
            leaf.get_or_add(api::EPropertyAttributes::key).extrusion_role(role).mm3_per_mm(0.04).width(0.45f).height(0.2f);
        if (request)
            leaf.get_or_add(api::EPropertyFan::key).set(request->percent, request->mode);
        const uint32_t idx = m_root.append_child_move(leaf.mutable_view());
        check(idx != UINT32_MAX, "cannot append the test leaf");
        return idx;
    }

    api::MutableExtrusionEntity node(size_t idx) { return m_root.child_mutable(uint32_t(idx)); }
    api::StoredExtrusionEntity &root() { return m_root; }
    storage_handle *storage() const { return m_storage; }

    // The leaves under child `idx` after the adapter ran: the leaf itself, or its pieces.
    std::vector<Piece> pieces(size_t idx)
    {
        std::vector<Piece> out;
        collect(node(idx).readonly(), out);
        return out;
    }

private:
    static void collect(api::ExtrusionEntity entity, std::vector<Piece> &out)
    {
        if (entity.child_count() == 0) {
            const api::EPropertyFan *fan = entity.get(api::EPropertyFan::key);
            const bool set = fan != nullptr && fan->is_set();
            out.push_back({ set ? fan->percent : -1.f, set ? fan->mode : FAN_REQUEST_MODE_UNSPECIFIED,
                            unscaled(entity.local_length()) });
            return;
        }
        for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
            collect(entity.child(child_idx), out);
    }

    storage_handle *m_storage;
    api::StoredExtrusionEntity m_root;
};

// The timeline of the scene, as the plugin builds it (travel speed 100 mm/s).
Timeline timeline_of(Scene &scene, double travel_speed = 100.)
{
    TimelineBuilder builder(true, travel_speed);
    builder.add_tree(scene.root().readonly(), false);
    return builder.timeline();
}

// Plans and applies the fan timing of the scene; returns the schedule.
Schedule run(Scene &scene, const Settings &settings, std::optional<float> entry)
{
    const Timeline timeline = timeline_of(scene);
    const Schedule schedule = plan_fan_timing(timeline.intervals, settings, entry);
    apply_changes(timeline, schedule);
    return schedule;
}

// The request of a single-piece leaf, or fail.
void expect_leaf(Scene &scene, size_t idx, float percent, fan_request_mode mode, const char *message)
{
    const std::vector<Piece> pieces = scene.pieces(idx);
    check(pieces.size() == 1 && scene.node(idx).child_count() == 0, message);
    check(pieces[0].percent == percent && (percent < 0.f || pieces[0].mode == mode), message);
}

// A travel leaf cut by an anticipated rise keeps its own request on the part before
// the cut and takes the pair of the transition (percent and mode) after it.
void test_travel_cut_and_pair_copy(storage_handle *storage)
{
    Scene scene(storage); // 50 mm/s for the deposition leaves
    const size_t a = scene.line(0, 10, Request{ 40.f, k_auto });                                            // 0.0 - 0.2 s
    const size_t b = scene.add(api::StoredExtrusionEntity(storage, std::vector<c_point>{ cp(10, 0), cp(50, 0) }),
                               Request{ 40.f, k_auto }, RAW_EXTRUSION_ROLE_TRAVEL);                         // 0.2 - 1.0 s
    scene.node(b).get_or_add(api::EPropertySpeed::key).speed(100.f);                                        // 0.2 - 0.6 s
    const size_t c = scene.line(50, 60, Request{ 70.f, k_override });                                       // 0.6 - 0.8 s

    // Rise at 0.6 s, anticipated by 0.2 s: 0.2 s into the travel, i.e. 20 mm.
    const Schedule schedule = run(scene, { 0.2, false, 0. }, std::nullopt);
    check(schedule.changes.size() == 2 && !schedule.restore_after, "two periods: A, then the rise moved into the travel");
    expect_leaf(scene, a, 40.f, k_auto, "the leaf before the travel is unchanged");
    const std::vector<Piece> parts = scene.pieces(b);
    check(parts.size() == 2 && scene.node(b).child_count() == 2, "the travel is cut in two");
    check(parts[0].percent == 40.f && parts[0].mode == k_auto && near(parts[0].length_mm, 20., 1e-4), "the first part keeps its request");
    check(parts[1].percent == 70.f && parts[1].mode == k_override && near(parts[1].length_mm, 20., 1e-4),
          "the second part takes the pair of the transition");
    check(scene.node(b).get(api::EPropertyFan::key) == nullptr, "the collection of the cut keeps no request");
    expect_leaf(scene, c, 70.f, k_override, "the transition leaf keeps its pair");
}

// A cut inside an arc with a Z change, at a speed inherited from the root, lands at the
// exact length along the arc, and the pieces keep the arc and the interpolated Z.
void test_arc_cut_with_z(storage_handle *storage)
{
    Scene scene(storage, 30.f);
    c_extrusion_segment arc = {};
    arc.point_a = cp(10, 0);
    arc.point_b = cp(0, 10);
    arc.radius = float(scale_d(10.));
    arc.orientation = RAW_EXTRUSION_ARC_ORIENTATION_CCW;
    arc.z_offset_a = 0;
    arc.z_offset_b = scale_i(0.8);
    api::StoredExtrusionEntity arc_leaf(storage);
    check(arc_leaf.set_segments(&arc, 1), "cannot build the arc");
    const size_t x = scene.add(std::move(arc_leaf), Request{ 40.f, k_auto }, k_infill);
    // The next leaf starts where the arc ends, also in Z, so that no gap is inserted.
    api::StoredExtrusionEntity next(storage, std::vector<c_point>{ cp(0, 10), cp(0, 20) });
    next.get_or_add(api::EPropertyZOffset::key).set(scale_i(0.8));
    const size_t y = scene.add(std::move(next), Request{ 70.f, k_auto }, k_infill);

    // A quarter of a circle of radius 10 mm rising 0.8 mm: 3D length of the helix.
    const double arc_length = PI * 10. / 2.;
    const double spatial = std::hypot(arc_length, 0.8);
    const Timeline timeline = timeline_of(scene);
    check(timeline.intervals.size() == 2 && near(timeline.intervals[0].duration, spatial / 30., 1e-9),
          "the duration is the 3D length over the inherited speed");
    // The rise is anticipated by half of the arc.
    const Schedule schedule = plan_fan_timing(timeline.intervals, { spatial / 30. / 2., false, 0. }, std::nullopt);
    apply_changes(timeline, schedule);

    const std::vector<Piece> parts = scene.pieces(x);
    check(parts.size() == 2 && parts[0].percent == 40.f && parts[1].percent == 70.f, "the arc is cut in two");
    check(near(parts[0].length_mm, arc_length / 2., 5e-5), "the cut is at half of the arc length");
    check(near(parts[0].length_mm + parts[1].length_mm, arc_length, 5e-5), "the pieces cover the arc");
    const api::ExtrusionEntity first = scene.node(x).child(0);
    const c_extrusion_segment last = first.segment(first.segment_count() - 1);
    check(last.radius != 0.f && near(unscaled(last.z_offset_b), 0.4, 1e-4), "the arc and the interpolated Z are kept");
}

// Checks the conversion of a time into a planar distance on a leaf with a vertical segment: the
// distance stays constant inside that segment. The adapter then refuses every cut of such a leaf
// (see has_vertical_move), which the tests of the vertical moves below check.
void test_vertical_segment_cut()
{
    LeafLengths lengths;
    lengths.planar = { 10., 0., 10. };
    lengths.spatial = { 10., 5., 10. };
    lengths.total = 25.;
    check(near(planar_distance_at(lengths, 5., 1.), 5., 1e-9), "inside the first segment");
    check(near(planar_distance_at(lengths, 5., 2.5), 10., 1e-9), "inside the vertical segment: its ends");
    check(near(planar_distance_at(lengths, 5., 4.), 15., 1e-9), "inside the last segment");
    check(near(planar_distance_at(lengths, 5., 6.), 20., 1e-9), "beyond the end");
}

// A change closer than the placement tolerance (0.1 s) to a boundary of its leaf does not
// cut it: it goes to the nearest boundary in time.
void test_placement_rounding(storage_handle *storage)
{
    // X (40%) and Y (70%) last 1 s each. The rise to Y at 1 s is anticipated by the delay,
    // so it falls `delay` seconds before the end of X.
    {
        // Far from both ends: a cut, at 0.5 s.
        Scene scene(storage);
        const size_t x = scene.line(0, 50, Request{ 40.f, k_auto });
        const size_t y = scene.line(50, 100, Request{ 70.f, k_override });
        run(scene, { 0.5, false, 0. }, std::nullopt);
        const std::vector<Piece> parts = scene.pieces(x);
        check(parts.size() == 2 && parts[0].percent == 40.f && parts[1].percent == 70.f && parts[1].mode == k_override &&
                  near(parts[0].length_mm, 25., 1e-4),
              "a change in the middle of a leaf cuts it");
        expect_leaf(scene, y, 70.f, k_override, "Y keeps its pair");
    }
    {
        // 0.12 s from the end of X: still far enough.
        Scene scene(storage);
        const size_t x = scene.line(0, 50, Request{ 40.f, k_auto });
        scene.line(50, 100, Request{ 70.f, k_override });
        run(scene, { 0.12, false, 0. }, std::nullopt);
        const std::vector<Piece> parts = scene.pieces(x);
        check(parts.size() == 2 && near(parts[0].length_mm, 44., 1e-4), "0.12 s before the end of a leaf is a cut");
    }
    {
        // 0.04 s after the start of X: no cut, the change goes to the start of X (the later change wins).
        Scene scene(storage);
        const size_t x = scene.line(0, 50, Request{ 40.f, k_auto });
        const size_t y = scene.line(50, 100, Request{ 70.f, k_override });
        run(scene, { 0.96, false, 0. }, std::nullopt);
        expect_leaf(scene, x, 70.f, k_override, "a change 0.04 s after the start of a leaf goes to its start");
        expect_leaf(scene, y, 70.f, k_override, "Y keeps its pair");
    }
    {
        // 0.04 s before the end of X: no cut, the change goes to the start of Y.
        Scene scene(storage);
        const size_t x = scene.line(0, 50, Request{ 40.f, k_auto });
        const size_t y = scene.line(50, 100, Request{ 70.f, k_override });
        run(scene, { 0.04, false, 0. }, std::nullopt);
        expect_leaf(scene, x, 40.f, k_auto, "a change 0.04 s before the end of a leaf goes to the start of the next one");
        expect_leaf(scene, y, 70.f, k_override, "the next leaf opens the period");
    }
}

// Two ordinary changes that end at the same position: the later one wins, and the leaves
// in between follow the period of the winner.
void test_ordinary_collision(storage_handle *storage)
{
    Scene scene(storage);
    const size_t a = scene.line(0, 50, Request{ 40.f, k_auto });           // 0.00 - 1.00 s
    const size_t b = scene.line(50, 52.5, Request{ 60.f, k_auto });        // 1.00 - 1.05 s
    const size_t c = scene.line(52.5, 102.5, Request{ 80.f, k_override }); // 1.05 - 2.05 s
    // The rises to B and C land at 0.88 s and 0.93 s: the second is 0.05 s after the first.
    const Schedule schedule = run(scene, { 0.12, false, 0. }, std::nullopt);
    check(schedule.changes.size() == 3 && schedule.changes[1].interval == 0 && schedule.changes[2].interval == 0,
          "the kernel plans both rises in A");
    const std::vector<Piece> parts = scene.pieces(a);
    check(parts.size() == 2 && parts[0].percent == 40.f && parts[1].percent == 80.f && parts[1].mode == k_override &&
              near(parts[0].length_mm, 44., 1e-4),
          "the second change is too close to the first to cut: it takes its position and wins");
    expect_leaf(scene, b, 80.f, k_override, "B is in the period of the winner");
    expect_leaf(scene, c, 80.f, k_override, "C keeps its pair");
}

// The start of a kick and its end (the return) at the same position: the end moves to
// the next position.
void test_kick_end_is_moved(storage_handle *storage)
{
    // A kick of 0.03 s, shorter than the tolerance, in a leaf of 1 s: the return is cut 0.1 s after the start.
    {
        Scene scene(storage);
        const size_t a = scene.line(0, 50, Request{ 40.f, k_auto });
        const size_t b = scene.line(50, 100, Request{ 70.f, k_override });
        const Schedule schedule = run(scene, { 0., false, 0.1 }, 40.f);
        check(!schedule.pulse_shortened, "the kernel keeps the kick");
        expect_leaf(scene, a, 40.f, k_auto, "A is unchanged");
        const std::vector<Piece> parts = scene.pieces(b);
        check(parts.size() == 2 && parts[0].percent == 100.f && parts[0].mode == k_override && parts[1].percent == 70.f &&
                  near(parts[0].length_mm, 5., 1e-4),
              "the kick is stretched to one tolerance: the return is cut at 0.1 s");
    }
    // A leaf that lasts less than twice the tolerance cannot be cut: the return moves to the next leaf.
    {
        Scene scene(storage);
        scene.line(0, 50, Request{ 40.f, k_auto });
        const size_t b = scene.line(50, 55, Request{ 70.f, k_override });  // 0.1 s
        const size_t c = scene.line(55, 105, Request{ 70.f, k_override });
        run(scene, { 0., false, 0.1 }, 40.f);
        expect_leaf(scene, b, 100.f, k_override, "the short leaf is all at 100%");
        expect_leaf(scene, c, 70.f, k_override, "the return is at the start of the next leaf");
    }
    // The gap before a leaf shares its position: a kick that ends where the leaf starts is moved
    // into the leaf, 0.1 s after the start of the gap position.
    {
        Scene scene(storage);
        const size_t a = scene.line(0, 50, Request{ 40.f, k_auto });        // 0 - 1 s
        const size_t b = scene.line(150, 200, Request{ 65.f, k_auto });     // gap 1 - 2 s (100 mm at 100 mm/s), leaf 2 - 3 s
        const Schedule schedule = run(scene, { 0., false, 4. }, 40.f);       // kick of 4 s * 25 / 100 = 1 s
        check(schedule.changes.size() == 2 && schedule.changes[0].interval == 1 && schedule.changes[1].interval == 2 &&
                  schedule.changes[1].offset == 0.,
              "the kernel ends the kick exactly where the leaf starts, after the gap");
        expect_leaf(scene, a, 40.f, k_auto, "A is unchanged");
        const std::vector<Piece> parts = scene.pieces(b);
        check(parts.size() == 2 && parts[0].percent == 100.f && parts[1].percent == 65.f && near(parts[0].length_mm, 5., 1e-4),
              "the kick is not lost: its end is cut 0.1 s into the leaf");
    }
    // The kick starts at a cut (the rise is anticipated by 0.5 s) and is shorter than the tolerance: its end
    // is brought back to the cut, then moved to 0.1 s after it.
    {
        Scene scene(storage);
        const size_t a = scene.line(0, 50, Request{ 40.f, k_auto });
        scene.line(50, 100, Request{ 70.f, k_override });
        run(scene, { 0.5, false, 0.1 }, 40.f); // kick of 0.1 s * 30 / 100 = 0.03 s from 0.5 s
        const std::vector<Piece> parts = scene.pieces(a);
        check(parts.size() == 3 && parts[0].percent == 40.f && parts[1].percent == 100.f && parts[1].mode == k_override &&
                  parts[2].percent == 70.f && near(parts[0].length_mm, 25., 1e-4) && near(parts[1].length_mm, 5., 1e-4),
              "a kick that starts at a cut lasts one tolerance at least");
    }
}

// The end of a kick moved into the next leaf, then a change that falls in the previous leaf:
// changes keep their order, and the final target is written. Every cut leaf keeps pieces of
// at least the tolerance.
void test_changes_keep_their_order(storage_handle *storage)
{
    Scene scene(storage);
    const size_t a = scene.line(0, 50, Request{ 40.f, k_auto });         // 0.00 - 1.00 s
    const size_t b = scene.line(50, 52.5, Request{ 70.f, k_auto });      // 1.00 - 1.05 s
    const size_t c = scene.line(52.5, 102.5, Request{ 90.f, k_auto });   // 1.05 - 2.05 s
    // The rise to B (0.85 s, kick of 0.03 s) has its return moved into B; the rise to C
    // (0.9 s, kick of 0.02 s) then falls in A again.
    const Schedule schedule = run(scene, { 0.15, false, 0.1 }, 40.f);
    check(schedule.changes.size() == 4, "the kernel plans two kicks in A");
    const std::vector<Piece> parts = scene.pieces(a);
    check(parts.size() == 2 && parts[0].percent == 40.f && parts[1].percent == 100.f, "A is cut once, where the first kick starts");
    for (const Piece &part : parts)
        check(part.length_mm / 50. >= 0.1 - 1e-9, "a piece of A lasts at least the tolerance");
    expect_leaf(scene, b, 100.f, k_auto, "B stays in the kick");
    expect_leaf(scene, c, 90.f, k_auto, "the final target is written");
    // The path of the last leaf is not shortened either: the pieces of the three leaves add up.
    double total = 0.;
    for (const size_t leaf : { a, b, c })
        for (const Piece &part : scene.pieces(leaf))
            total += part.length_mm;
    check(near(total, 102.5, 1e-3), "the cuts keep the length of the path");
}

// Cutting a perimeter loop of shell 0 (the one that carries the seam): the writer emits one seam marker
// per node that carries the perimeter property, and the cut copies it onto the collection and every piece.
// The collection keeps it (its start is the start of the loop); the pieces do not.
void test_cut_perimeter_keeps_its_property(storage_handle *storage)
{
    Scene scene(storage);
    const size_t a = scene.line(0, 50, Request{ 20.f, k_auto }, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const size_t b = scene.line(50, 100, Request{ 80.f, k_auto }, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    scene.node(a).get_or_add(api::EPropertyPerimeter::key).shell_count(0);
    const uint16_t flags = scene.node(a).readonly().get(api::EPropertyPerimeter::key)->perimeter_flags();
    run(scene, { 0.5, false, 0. }, std::nullopt);
    check(scene.pieces(a).size() == 2 && scene.node(a).child_count() == 2, "the shell 0 leaf is cut");
    const api::EPropertyPerimeter *perimeter = scene.node(a).readonly().get(api::EPropertyPerimeter::key);
    check(perimeter != nullptr && perimeter->shell_count() == 0 && perimeter->perimeter_flags() == flags,
          "the collection of the cut carries the perimeter property of shell 0");
    for (uint32_t piece = 0; piece < scene.node(a).child_count(); ++piece)
        check(!scene.node(a).readonly().child(piece).has(api::EPropertyPerimeter::key), "no piece carries the perimeter property");
    check(scene.pieces(b).size() == 1, "the next leaf is not cut");
}
// A path that contains a vertical move (horizontal, lift, horizontal) is never cut, wherever the cut is
// requested: before the lift, inside it or after it (the host refuses any split of such a path). The
// change goes to the nearest boundary in time.
void test_path_with_a_vertical_move_is_not_cut(storage_handle *storage)
{
    struct Case
    {
        double delay;        // the rise at 3.2 s lands (3.2 - delay - 1.0) s into the path
        bool at_path_start;  // the nearest boundary is the start of the path (else the start of the next leaf)
        const char *where;
    };
    for (const Case &c : { Case{ 1.7, true, "before the lift (0.5 s into the path)" },
                           Case{ 1.05, false, "inside the lift (1.15 s into the path)" },
                           Case{ 0.5, false, "after the lift (1.7 s into the path)" } }) {
        Scene scene(storage); // 50 mm/s
        const size_t p = scene.line(-50, 0, Request{ 40.f, k_auto }); // 0.0 - 1.0 s
        // 50 mm along X (1.0 s), a lift of 10 mm (0.2 s), 50 mm along X (1.0 s).
        c_extrusion_segment path[3] = {};
        path[0].point_a = cp(0, 0);
        path[0].point_b = cp(50, 0);
        path[1].point_a = cp(50, 0);
        path[1].point_b = cp(50, 0);
        path[1].z_offset_b = scale_i(10.);
        path[2].point_a = cp(50, 0);
        path[2].point_b = cp(100, 0);
        path[2].z_offset_a = scale_i(10.);
        path[2].z_offset_b = scale_i(10.);
        api::StoredExtrusionEntity leaf(storage);
        check(leaf.set_segments(path, 3) && leaf.segment_count() == 3, "cannot build the path with a lift");
        const size_t v = scene.add(std::move(leaf), Request{ 40.f, k_auto }, RAW_EXTRUSION_ROLE_TRAVEL); // 1.0 - 3.2 s
        api::StoredExtrusionEntity next(storage, std::vector<c_point>{ cp(100, 0), cp(150, 0) });
        next.get_or_add(api::EPropertyZOffset::key).set(scale_i(10.));
        const size_t y = scene.add(std::move(next), Request{ 70.f, k_override }, k_infill);              // 3.2 - 4.2 s

        run(scene, { c.delay, false, 0. }, std::nullopt);
        expect_leaf(scene, p, 40.f, k_auto, c.where);
        expect_leaf(scene, v, c.at_path_start ? 70.f : 40.f, c.at_path_start ? k_override : k_auto, c.where);
        expect_leaf(scene, y, 70.f, k_override, c.where);
    }
}

// An ordinary change is placed by the same rule as the end of a kick: a barrier is not a
// position, so a change less than the tolerance before it stays one tolerance before it.
// (The planner never anticipates a rise up to a barrier; the schedule is written by hand.)
void test_ordinary_change_near_a_barrier(storage_handle *storage)
{
    Scene scene(storage);
    scene.line(0, 10, Request{ 40.f, k_auto });                  // 0.0 - 0.2 s
    const size_t x = scene.line(10, 110, Request{ 40.f, k_auto }); // 0.2 - 2.2 s
    const size_t e = scene.event();
    scene.node(e).custom_gcode("M117 barrier");
    const size_t y = scene.line(110, 120, Request{ 70.f, k_override });
    const Timeline timeline = timeline_of(scene);
    Schedule schedule;
    schedule.changes = { Change{ 1, 1.95, 70.f, 3 } }; // 0.05 s before the barrier
    check(!apply_changes(timeline, schedule), "nothing is dropped");
    const std::vector<Piece> parts = scene.pieces(x);
    check(parts.size() == 2 && parts[0].percent == 40.f && parts[1].percent == 70.f && parts[1].mode == k_override &&
              near(parts[0].length_mm, 95., 1e-4) && near(parts[1].length_mm, 5., 1e-4),
          "the change is cut one tolerance before the barrier");
    check(scene.node(e).get(api::EPropertyFan::key) == nullptr, "the barrier is not edited");
    expect_leaf(scene, y, 70.f, k_override, "the leaf after the barrier is unchanged");
}

// If the next position is behind a barrier or the end of the group, a kick shorter than the
// tolerance is dropped; the change keeps its place at the target and the caller is told.
// A return too close to a barrier stays just short of it instead.
void test_kick_dropped_or_clamped(storage_handle *storage)
{
    {
        // B lasts 0.1 s and a barrier follows.
        Scene scene(storage);
        scene.line(0, 50, Request{ 40.f, k_auto });
        const size_t b = scene.line(50, 55, Request{ 70.f, k_override });
        const size_t e = scene.event();
        scene.node(e).custom_gcode("M117 barrier");
        scene.line(55, 105, Request{ 70.f, k_override });
        const Timeline timeline = timeline_of(scene);
        const Schedule schedule = plan_fan_timing(timeline.intervals, { 0., false, 0.1 }, 40.f);
        check(!schedule.pulse_shortened, "the kernel keeps the kick");
        check(apply_changes(timeline, schedule), "the dropped kick is reported");
        expect_leaf(scene, b, 70.f, k_override, "the transition keeps its place at the target");
        check(scene.node(e).get(api::EPropertyFan::key) == nullptr, "the barrier is not edited");
    }
    {
        // B lasts 0.1 s and is the last leaf: the end of the group comes next.
        Scene scene(storage);
        scene.line(0, 50, Request{ 40.f, k_auto });
        const size_t b = scene.line(50, 55, Request{ 70.f, k_override });
        const Timeline timeline = timeline_of(scene);
        const Schedule schedule = plan_fan_timing(timeline.intervals, { 0., false, 0.1 }, 40.f);
        check(apply_changes(timeline, schedule) && !schedule.restore_after, "the kick is dropped at the end of the group");
        expect_leaf(scene, b, 70.f, k_override, "the transition keeps its place at the target");
    }
    {
        // The kick of 1.95 s ends 0.05 s before the barrier: no cut fits, the return is cut 0.1 s before the barrier.
        Scene scene(storage);
        scene.line(0, 10, Request{ 40.f, k_auto });
        const size_t b = scene.line(10, 110, Request{ 70.f, k_override }); // 2 s
        const size_t e = scene.event();
        scene.node(e).custom_gcode("M117 barrier");
        scene.line(110, 120, Request{ 70.f, k_override });
        const Timeline timeline = timeline_of(scene);
        const Schedule schedule = plan_fan_timing(timeline.intervals, { 0., false, 6.5 }, 40.f);
        check(!schedule.pulse_shortened, "the kernel places the return before the barrier");
        check(!apply_changes(timeline, schedule), "nothing is dropped");
        const std::vector<Piece> parts = scene.pieces(b);
        check(parts.size() == 2 && parts[0].percent == 100.f && parts[1].percent == 70.f && near(parts[0].length_mm, 95., 1e-4) &&
                  near(parts[1].length_mm, 5., 1e-4),
              "the return stays one tolerance before the barrier");
    }
}

// A leaf that starts with a vertical move is never cut (the host refuses pieces without XY length):
// the change goes to the nearest boundary instead.
void test_vertical_move_is_not_cut(storage_handle *storage)
{
    Scene scene(storage); // 50 mm/s
    const size_t p = scene.line(-50, 0, Request{ 40.f, k_auto }); // 0.0 - 1.0 s
    // A lift of 10 mm (0.2 s) at the origin, then 50 mm along X (1.0 s), rising to 10 mm.
    c_extrusion_segment lift[2] = {};
    lift[0].point_a = cp(0, 0);
    lift[0].point_b = cp(0, 0);
    lift[0].z_offset_a = 0;
    lift[0].z_offset_b = scale_i(10.);
    lift[1].point_a = cp(0, 0);
    lift[1].point_b = cp(50, 0);
    lift[1].z_offset_a = scale_i(10.);
    lift[1].z_offset_b = scale_i(10.);
    api::StoredExtrusionEntity lift_leaf(storage);
    check(lift_leaf.set_segments(lift, 2) && lift_leaf.segment_count() == 2, "cannot build the vertical move");
    const size_t v = scene.add(std::move(lift_leaf), Request{ 40.f, k_auto }, RAW_EXTRUSION_ROLE_TRAVEL); // 1.0 - 2.2 s
    api::StoredExtrusionEntity next(storage, std::vector<c_point>{ cp(50, 0), cp(100, 0) });
    next.get_or_add(api::EPropertyZOffset::key).set(scale_i(10.));
    const size_t y = scene.add(std::move(next), Request{ 70.f, k_override }, k_infill);                   // 2.2 - 3.2 s

    // The rise lands 0.1 s into the lift, where the XY path has no length.
    run(scene, { 1.1, false, 0. }, std::nullopt);
    expect_leaf(scene, p, 40.f, k_auto, "the leaf before is unchanged");
    expect_leaf(scene, v, 70.f, k_override, "the change goes to the start of the leaf");
    expect_leaf(scene, y, 70.f, k_override, "Y keeps its pair");
}

// A rise reaching into the middle of a leaf that carries a request: the leaf is cut, and
// the following leaves of the period are rewritten, a lower request included.
void test_rise_in_a_carrying_leaf(storage_handle *storage)
{
    Scene scene(storage, 20.f);
    const size_t a = scene.line(0, 10, Request{ 60.f, k_auto });      // 0.0 - 0.5 s
    const size_t b = scene.line(10, 20, Request{ 20.f, k_auto });     // 0.5 - 1.0 s
    const size_t c = scene.line(20, 30, Request{ 70.f, k_override }); // 1.0 - 1.5 s
    const size_t d = scene.line(30, 40, Request{ 30.f, k_auto });     // 1.5 - 2.0 s
    const Schedule schedule = run(scene, { 0.8, false, 0. }, std::nullopt);
    check(schedule.changes.size() == 3, "A, the anticipated rise, the fall of D");
    const std::vector<Piece> parts = scene.pieces(a);
    check(parts.size() == 2 && parts[0].percent == 60.f && parts[1].percent == 70.f && parts[1].mode == k_override &&
              near(parts[0].length_mm, 4., 1e-4),
          "A is cut 0.2 s after its start and its second part takes the rise");
    expect_leaf(scene, b, 70.f, k_override, "the lower request of B is erased by the period");
    expect_leaf(scene, c, 70.f, k_override, "C keeps its pair");
    expect_leaf(scene, d, 30.f, k_auto, "the fall is never moved");
}

// A leaf with a one-shot property is never cut, so the event is not repeated; an instant
// inside a non-cuttable leaf moves to its start, and the planner stops at a heater.
void test_one_shot_events_are_not_duplicated(storage_handle *storage)
{
    // The heater is an event at the start of H: the anticipation stops there.
    {
        Scene scene(storage);
        scene.line(0, 10, Request{ 40.f, k_auto });
        api::StoredExtrusionEntity heated(storage, std::vector<c_point>{ cp(10, 0), cp(20, 0) });
        heated.get_or_add(api::EPropertyHeater::key).set_tool(200);
        const size_t h = scene.add(std::move(heated), Request{ 40.f, k_auto }, k_infill);
        const size_t y = scene.line(20, 30, Request{ 70.f, k_auto });
        run(scene, { 0.5, false, 0. }, 40.f);
        expect_leaf(scene, h, 70.f, k_auto, "the leaf with the heater takes the pair without being cut");
        check(scene.node(h).has(api::EPropertyHeater::key), "the heater event is kept once");
        expect_leaf(scene, y, 70.f, k_auto, "the transition leaf keeps its request");
    }
    // A wipe carries an extrusion-axis event: it lasts 0.4 s and cannot be cut.
    {
        Scene scene(storage);
        const size_t x = scene.line(0, 10, Request{ 40.f, k_auto }); // 0.0 - 0.2 s
        api::StoredExtrusionEntity wipe(storage, std::vector<c_point>{ cp(10, 0), cp(30, 0) });
        wipe.get_or_add(api::EPropertyExtrusionAxis::key).retract_to(0.5);
        const size_t w = scene.add(std::move(wipe), std::nullopt, RAW_EXTRUSION_ROLE_WIPE | RAW_EXTRUSION_ROLE_TRAVEL); // 0.2 - 0.6 s
        const size_t y = scene.line(30, 40, Request{ 70.f, k_override });                                               // 0.6 - 0.8 s
        const Timeline timeline = timeline_of(scene);
        check(timeline.intervals.size() == 3 && timeline.intervals[1].kind == IntervalKind::NonCuttable &&
                  near(timeline.intervals[1].duration, 0.4, 1e-9) && !timeline.intervals[1].request,
              "a wipe is a transparent non-cuttable interval with its real duration");
        const Schedule schedule = plan_fan_timing(timeline.intervals, { 0.2, false, 0. }, std::nullopt);
        apply_changes(timeline, schedule);
        expect_leaf(scene, x, 40.f, k_auto, "the leaf before the wipe is unchanged");
        expect_leaf(scene, w, 70.f, k_override, "the instant inside the wipe moves to its start; it receives the pair");
        expect_leaf(scene, y, 70.f, k_override, "the transition leaf keeps its request");
    }
}

// The barriers: what they are, that they are never edited, and that they limit the anticipation.
void test_barriers(storage_handle *storage)
{
    // Custom G-code before a leaf: a rise is anticipated up to the end of the barrier, no further.
    {
        Scene scene(storage);
        const size_t a = scene.line(0, 10, Request{ 40.f, k_auto });
        const size_t e = scene.event();
        scene.node(e).custom_gcode("M117 barrier");
        const size_t b = scene.line(10, 20, Request{ 40.f, k_auto });
        const size_t c = scene.line(20, 30, Request{ 70.f, k_auto });
        const Timeline timeline = timeline_of(scene);
        check(timeline.intervals.size() == 4 && timeline.intervals[1].kind == IntervalKind::Barrier &&
                  timeline.intervals[1].resets_state && timeline.intervals[1].duration == 0.,
              "a custom G-code is an opaque barrier of zero duration");
        const Schedule schedule = plan_fan_timing(timeline.intervals, { 5., false, 0. }, std::nullopt);
        check(schedule.changes.size() == 2 && schedule.changes[1].interval == 2 && schedule.changes[1].offset == 0. &&
                  schedule.changes[1].percent == 70.f,
              "the rise starts at the end of the barrier");
        apply_changes(timeline, schedule);
        expect_leaf(scene, a, 40.f, k_auto, "A is before the barrier");
        check(scene.node(e).get(api::EPropertyFan::key) == nullptr, "the barrier is not edited");
        expect_leaf(scene, b, 70.f, k_auto, "B, after the barrier, is in the anticipated period");
        expect_leaf(scene, c, 70.f, k_auto, "C keeps its request");
    }
    // A special command is opaque: the first request after it is neither anticipated nor kicked.
    {
        Scene scene(storage);
        const size_t a = scene.line(0, 10, Request{ 40.f, k_auto });
        const size_t s = scene.event();
        scene.node(s).get_or_add(api::EPropertySpecialCommand::key).set(C_EXTRUSION_SPECIAL_COMMAND_FLUSH_PLANNER_QUEUE);
        const size_t b = scene.line(10, 20, Request{ 70.f, k_auto });
        const Schedule schedule = run(scene, { 5., false, 10. }, 40.f);
        check(schedule.changes.size() == 1 && schedule.changes[0].interval == 2 && schedule.changes[0].percent == 70.f &&
                  !schedule.restore_after,
              "the target is unknown after the command: the rise is a plain transition");
        expect_leaf(scene, a, 40.f, k_auto, "A is unchanged");
        check(scene.node(s).get(api::EPropertyFan::key) == nullptr, "the command is not edited");
        expect_leaf(scene, b, 70.f, k_auto, "B keeps its request");
    }
    // A raw EXTRUSION leaf is an opaque barrier that keeps its real duration.
    {
        Scene scene(storage);
        scene.line(0, 10, Request{ 40.f, k_auto });
        api::StoredExtrusionEntity raw(storage, std::vector<c_point>{ cp(10, 0), cp(20, 0) });
        raw.get_or_add(api::EPropertySpecialCommand::key).set(C_EXTRUSION_SPECIAL_COMMAND_EXTRUSION, 1.5);
        scene.add(std::move(raw), std::nullopt, k_infill);
        scene.line(20, 30, Request{ 70.f, k_auto });
        const Timeline timeline = timeline_of(scene);
        check(timeline.intervals.size() == 3 && timeline.intervals[1].kind == IntervalKind::Barrier &&
                  timeline.intervals[1].resets_state && near(timeline.intervals[1].duration, 0.2, 1e-9),
              "a raw EXTRUSION leaf is an opaque barrier");
        check(target_after(timeline.intervals, std::nullopt) == 70.f, "the next request is the target again");
    }
    // A service fan event is a barrier that makes the target known; a heater keeps the target.
    {
        Scene scene(storage);
        const size_t a = scene.line(0, 10, Request{ 40.f, k_auto });
        const size_t f = scene.event(Request{ 60.f, k_override });
        const size_t b = scene.line(10, 20, Request{ 60.f, k_auto });
        const size_t h = scene.event();
        scene.node(h).get_or_add(api::EPropertyHeater::key).set_bed(60);
        const size_t b2 = scene.line(20, 30, Request{ 60.f, k_auto });
        const size_t c = scene.line(30, 40, Request{ 80.f, k_auto });
        const Timeline timeline = timeline_of(scene);
        check(timeline.intervals[1].kind == IntervalKind::Barrier && timeline.intervals[1].request == 60.f &&
                  !timeline.intervals[1].resets_state,
              "a fan request on an empty leaf is a barrier with a request");
        check(timeline.intervals[3].kind == IntervalKind::Barrier && !timeline.intervals[3].request &&
                  !timeline.intervals[3].resets_state,
              "a heater is a barrier without effect on the target");
        // The entry is unknown but the service event sets 60: the rise 60 -> 80 is anticipated,
        // up to the heater.
        const Schedule schedule = plan_fan_timing(timeline.intervals, { 5., false, 0. }, std::nullopt);
        apply_changes(timeline, schedule);
        expect_leaf(scene, a, 40.f, k_auto, "A is before the service event");
        check(scene.pieces(f).size() == 1 && scene.pieces(f)[0].percent == 60.f && scene.pieces(f)[0].mode == k_override,
              "the service event is not edited");
        expect_leaf(scene, b, 60.f, k_auto, "B is before the heater");
        check(scene.node(h).get(api::EPropertyFan::key) == nullptr, "the heater is not edited");
        expect_leaf(scene, b2, 80.f, k_auto, "B2, after the heater, is in the anticipated period");
        expect_leaf(scene, c, 80.f, k_auto, "C keeps its request");
    }
}

// The gap between two leaves belongs to the leaf that follows: the transition is at the
// start of the gap, and the gap itself is never cut. (The kick that ends where the leaf
// starts is in test_kick_end_is_moved.)
void test_gap_belongs_to_the_next_leaf(storage_handle *storage)
{
    Scene scene(storage);
    const size_t a = scene.line(0, 30, Request{ 40.f, k_auto });       // 0.00 - 0.60 s
    const size_t b = scene.line(32, 42, Request{ 70.f, k_override });  // gap 0.60 - 0.62 s (2 mm at 100 mm/s), leaf 0.62 - 0.82 s
    const Timeline timeline = timeline_of(scene);
    check(timeline.intervals.size() == 3 && timeline.intervals[1].request == 70.f, "the gap carries the request of B");
    // The rise is requested before the gap: 0.2 s before the end of A.
    const Schedule schedule = plan_fan_timing(timeline.intervals, { 0.2, false, 0. }, 40.f);
    check(schedule.changes.size() == 1 && schedule.changes[0].interval == 0 && near(schedule.changes[0].offset, 0.4, 1e-9) &&
              schedule.changes[0].source == 1,
          "the rise is requested before the gap");
    apply_changes(timeline, schedule);
    const std::vector<Piece> parts = scene.pieces(a);
    check(parts.size() == 2 && parts[0].percent == 40.f && parts[1].percent == 70.f && parts[1].mode == k_override &&
              near(parts[0].length_mm, 20., 1e-4),
          "A is cut 0.4 s after its start");
    expect_leaf(scene, b, 70.f, k_override, "B keeps its pair");
}

// A kick still running after the last leaf is closed by an empty leaf with a direct
// request, in the mode of the transition that started it.
void test_restore_after(storage_handle *storage)
{
    Scene scene(storage);
    const size_t a = scene.line(0, 10, Request{ 40.f, k_auto });
    const size_t b = scene.line(10, 20, Request{ 70.f, k_override });
    const Timeline timeline = timeline_of(scene);
    const Schedule schedule = plan_fan_timing(timeline.intervals, { 0., false, 10. }, 40.f);
    check(schedule.restore_after && schedule.restore_after->interval == 2 && !schedule.pulse_shortened,
          "the 3 s pulse ends after the group");
    apply_changes(timeline, schedule);
    expect_leaf(scene, a, 40.f, k_auto, "A is unchanged");
    expect_leaf(scene, b, 100.f, k_override, "the pulse is written in the mode of its transition");

    api::StoredExtrusionEntity event = restore_event(storage, timeline, *schedule.restore_after);
    const api::EPropertyFan *fan = event.get(api::EPropertyFan::key);
    check(event.child_count() == 0 && event.point_count() == 0, "the restore event is an empty leaf");
    check(fan != nullptr && fan->percent == 70.f && fan->mode == k_override, "it restores the target with its mode");
}

// The timeline: durations, gaps, classification and roles.
void test_timeline_building(storage_handle *storage)
{
    Scene scene(storage);
    scene.event(); // nothing to do: no interval
    const size_t custom = scene.event();
    scene.node(custom).custom_gcode("M117 x");
    const size_t heater = scene.event();
    scene.node(heater).get_or_add(api::EPropertyHeater::key).set_tool(210);
    scene.event(Request{ 55.f, k_override });
    const size_t axis = scene.event();
    scene.node(axis).get_or_add(api::EPropertyExtrusionAxis::key).unretract();
    scene.line(0, 10, Request{ 40.f, k_auto });                        // first leaf: no gap
    scene.line(12, 22, Request{ 70.f, k_auto }, k_overhang);           // 2 mm gap at 100 mm/s
    api::StoredExtrusionEntity raw(storage, std::vector<c_point>{ cp(22, 0), cp(32, 0) });
    raw.get_or_add(api::EPropertySpecialCommand::key).set(C_EXTRUSION_SPECIAL_COMMAND_EXTRUSION, 1.);
    scene.add(std::move(raw), std::nullopt, k_infill);
    api::StoredExtrusionEntity heated(storage, std::vector<c_point>{ cp(32, 0), cp(42, 0) });
    heated.get_or_add(api::EPropertyHeater::key).set_tool(200);
    scene.add(std::move(heated), Request{ 30.f, k_auto }, k_infill);

    const Timeline timeline = timeline_of(scene);
    const std::vector<Interval> &iv = timeline.intervals;
    check(iv.size() == 10 && timeline.origins.size() == iv.size(), "one interval per event and per leaf, plus the gap");
    check(iv[0].kind == IntervalKind::Barrier && iv[0].resets_state && !iv[0].request, "custom G-code");
    check(iv[1].kind == IntervalKind::Barrier && !iv[1].resets_state && !iv[1].request, "heater");
    check(iv[2].kind == IntervalKind::Barrier && iv[2].request == 55.f && !iv[2].resets_state, "fan event");
    check(iv[3].kind == IntervalKind::NonCuttable && iv[3].duration == 0. && !iv[3].request, "extrusion-axis event");
    check(iv[4].kind == IntervalKind::Cuttable && near(iv[4].duration, 0.2, 1e-9) && iv[4].request == 40.f && !iv[4].overhang,
          "deposition leaf");
    check(iv[5].kind == IntervalKind::NonCuttable && near(iv[5].duration, 0.02, 1e-9) && iv[5].request == 70.f && iv[5].overhang,
          "the gap is non-cuttable, at travel speed, with the request and the role of the leaf that follows");
    check(iv[6].kind == IntervalKind::Cuttable && near(iv[6].duration, 0.2, 1e-9) && iv[6].request == 70.f && iv[6].overhang,
          "overhang leaf");
    check(iv[7].kind == IntervalKind::Barrier && iv[7].resets_state && near(iv[7].duration, 0.2, 1e-9), "raw EXTRUSION leaf");
    check(iv[8].kind == IntervalKind::Barrier && !iv[8].resets_state && iv[8].duration == 0. && !iv[8].request,
          "the heater of a leaf with segments comes first");
    check(iv[9].kind == IntervalKind::NonCuttable && near(iv[9].duration, 0.2, 1e-9) && iv[9].request == 30.f,
          "a leaf with a one-shot property cannot be cut");

    check(target_after(std::vector<Interval>(iv.begin(), iv.begin() + 4), std::nullopt) == 55.f, "the fan event sets the target");
    check(target_after(std::vector<Interval>(iv.begin(), iv.begin() + 7), std::nullopt) == 70.f, "the last request is the target");
    check(target_after(std::vector<Interval>(iv.begin(), iv.begin() + 8), std::nullopt) == std::nullopt,
          "a raw EXTRUSION leaf makes the target unknown");
    check(target_after(iv, std::nullopt) == 30.f, "until the next request");
}

// Never propagate an exception through the C callback.
void test(storage_handle *storage)
{
    test_vertical_segment_cut();
    test_travel_cut_and_pair_copy(storage);
    test_arc_cut_with_z(storage);
    test_placement_rounding(storage);
    test_ordinary_collision(storage);
    test_kick_end_is_moved(storage);
    test_kick_dropped_or_clamped(storage);
    test_vertical_move_is_not_cut(storage);
    test_path_with_a_vertical_move_is_not_cut(storage);
    test_changes_keep_their_order(storage);
    test_cut_perimeter_keeps_its_property(storage);
    test_ordinary_change_near_a_barrier(storage);
    test_rise_in_a_carrying_leaf(storage);
    test_one_shot_events_are_not_duplicated(storage);
    test_barriers(storage);
    test_gap_belongs_to_the_next_leaf(storage);
    test_restore_after(storage);
    test_timeline_building(storage);
}

void callback(uint32_t, storage_handle *storage, void *user)
{
    try {
        test(storage);
    } catch (...) {
        *static_cast<std::exception_ptr *>(user) = std::current_exception();
    }
}
} // namespace

int main()
{
    try {
        std::exception_ptr error;
        slic3r_parallel_for_storage(0, 1, &error, callback);
        if (error)
            std::rethrow_exception(error);
        std::cout << "fan speed-up SDK tests passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
