///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <memory>
#include <vector>

#include <boost/multiprecision/cpp_int.hpp>

#include "libslic3r/Api/plugin/c/slic3r_clipper.h"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"

#include "ClipperShapes.hpp"
#include "Orchestrator.hpp"

/*
Provide polygon clipping and offsets to plugins without exposing native C++ types.
The shape route borrows geometry through ClipperShapes, computes intermediate
path lists or contour/hole trees, then materializes public geometry collections.
The native-result route calls ClipperUtils directly when callers need its exact
processing and order rather than the shape route's processing.
Closed-ring offsets clean spikes without changing unrelated rings. The configurable
variable inward offset validates per-vertex distances and coordinate margins
before calling the native geometry routine; inputs are never changed.

Native-result calls validate arguments, compute into local owning temporaries,
then transfer a successful result to PluginStorage. They catch all exceptions
and return NULL without storing a result. The shape route does not provide that
exception guarantee. Storage owns returned handles until free, clear or destruction;
adapters borrow their source geometry for their entire use.

PluginStorage locks its ownership registry, not the geometry. Callers keep borrowed
inputs alive and unchanged during use and exclude concurrent edits or destruction.

Plugin calls
|-- clipper_shapes_from_*: retain a borrowed source adapter in storage
|-- clipper_execute / clipper_offset: compute a stored shape result
|   `-- offset_closed_rings: clean affected rings and unite the result
|-- clipper_shapes_to_*: materialize a storage-owned geometry collection
|-- clipper_union_ex / clipper_offset_ex / clipper_offset2_ex: compute native results
|   `-- store_expolygons: transfer the completed collection to storage
|-- clipper_variable_offset_inner: validate distances and coordinate margins
|   `-- variable_offset_inner_ex: compute with the requested miter limit
|-- clipper_diff_ex / clipper_intersection_ex: native cleanup and topology rebuild
|   `-- store_expolygons: transfer the completed collection to storage
|-- clipper_variable_offset_inner_ex: validate and regroup vertex distances
|   |-- variable_offset_inner_ex: compute inset areas using per-ring distances
|   `-- store_expolygons: transfer the completed collection to storage
|-- clipper_detour_polyline_expolygons: replace polyline stretches crossing rings by ring arcs
`-- clipper_expand: expand individual rings, unite them and store Polygons
*/

// Borrow both collections and store the native difference unchanged; NULL handles
// or any exception return NULL without adding an object to storage.
expolygon_collection_handle *clipper_diff_ex(storage_handle *storage,
    const expolygon_collection_handle *subject, const expolygon_collection_handle *clip);
// Borrow both collections and store the native intersection unchanged; NULL
// handles or any exception return NULL without adding an object to storage.
expolygon_collection_handle *clipper_intersection_ex(storage_handle *storage,
    const expolygon_collection_handle *subject, const expolygon_collection_handle *clip);
// Validate borrowed rings and flattened non-positive deltas before rebuilding
// native ring vectors; store only a successful result, returning NULL on failure.
expolygon_collection_handle *clipper_variable_offset_inner_ex(storage_handle *storage,
    const expolygon_handle *expolygon, const float *deltas, uint32_t delta_count);
// Expand borrowed rings with native orientation handling and union; finite
// positive delta is required, and failures store nothing and return NULL.
polygon_collection_handle *clipper_expand(storage_handle *storage,
    const polygon_collection_handle *polygons, double delta);
// Compute the detour from borrowed inputs into a local polyline stored only once
// complete; a NULL argument, an unknown direction or an exception returns NULL.
polyline_handle *clipper_detour_polyline_expolygons(storage_handle *storage,
    const polyline_handle *polyline, const clipper_shapes_handle *rings,
    clipper_detour_direction_t direction);

namespace Slic3r {

// Borrowed subject collections remain read-only while core clipping owns its temporary paths.
static const Polylines *to_polylines(const polyline_collection_handle *handle);

// Offset normalized closed rings (contours counter-clockwise, holes clockwise):
// each ring whose offset emits a spike is cleaned alone, the others are offset
// raw, then one positive-fill union.
static ClipperLib::PolyTree offset_closed_rings(const ClipperLib::Paths &rings, double delta,
                                                ClipperLib::JoinType join_type, double miter_limit);

// Apply the join limit, the ClipperUtils shortest-edge rule and the cheaper
// positive-fill clean-up of negative offsets.
static void configure_offsetter(ClipperLib::ClipperOffset &offsetter, double delta,
                                ClipperLib::JoinType join_type, double miter_limit);

// Whether ClipperOffset emits at least one spike loop when offsetting the ring.
static bool has_offset_spike(const ClipperLib::Path &ring, double delta);

// Unite the rings of an offset result with a non-zero fill.
static ClipperLib::PolyTree united_polytree(ClipperLib::PolyTree tree);


/* ---- ABI handle casts ------------------------------------------------
The public headers expose opaque C handles. Internally they are pointers to the
native Slic3r objects or to the ApiClipper::ClipperShapes interface.
*/

static PluginStorage *to_storage(storage_handle *storage) {
    return reinterpret_cast<PluginStorage *>(storage);
}

static const Polygon *to_polygon(const polygon_handle *handle) {
    return reinterpret_cast<const Polygon *>(handle);
}

static const Polyline *to_polyline(const polyline_handle *handle) {
    return reinterpret_cast<const Polyline *>(handle);
}

static Polylines *to_polylines(polyline_collection_handle *handle) {
    return reinterpret_cast<Polylines *>(handle);
}

static const Polylines *to_polylines(const polyline_collection_handle *handle) {
    return reinterpret_cast<const Polylines *>(handle);
}

static const Polygons *to_polygons(const polygon_collection_handle *handle) {
    return reinterpret_cast<const Polygons *>(handle);
}

static Polygons *to_polygons(polygon_collection_handle *handle) {
    return reinterpret_cast<Polygons *>(handle);
}

static const ExPolygon *to_expolygon(const expolygon_handle *handle) {
    return reinterpret_cast<const ExPolygon *>(handle);
}

static ExPolygons *to_expolygons(expolygon_collection_handle *handle) {
    return reinterpret_cast<ExPolygons *>(handle);
}

static const ExPolygons *to_expolygons(const expolygon_collection_handle *handle) {
    return reinterpret_cast<const ExPolygons *>(handle);
}

static BoundingBox to_bounding_box(c_bounding_box bbox) {
    return BoundingBox(Point(bbox.min.x, bbox.min.y), Point(bbox.max.x, bbox.max.y));
}

static c_point to_c_point(const Point &point) {
    c_point out = {};
    out.x = point.x();
    out.y = point.y();
    return out;
}

static c_bounding_box to_c_bounding_box(const BoundingBox &bbox) {
    c_bounding_box out = {};
    out.min = to_c_point(bbox.min);
    out.max = to_c_point(bbox.max);
    return out;
}

static ApiClipper::ClipperShapes *to_shapes(clipper_shapes_handle *handle) {
    return reinterpret_cast<ApiClipper::ClipperShapes *>(handle);
}

static const ApiClipper::ClipperShapes *to_shapes(const clipper_shapes_handle *handle) {
    return reinterpret_cast<const ApiClipper::ClipperShapes *>(handle);
}

// Retain the shape through the locked storage registry; null shapes return null and insertion errors leave no tracked handle.
static ApiClipper::ClipperShapes *store_shape(PluginStorage &plugin_storage, std::unique_ptr<ApiClipper::ClipperShapes> shape) {
    return plugin_storage.store(std::move(shape));
}

/*
Store a newly created ClipperShapes implementation in plugin temporary storage.

The returned raw pointer is the ABI handle. The storage registry entry owns the
shape, so the pointer remains valid until storage_free() or storage_clear()
removes it.
*/
static clipper_shapes_handle *store_shape(storage_handle *storage, std::unique_ptr<ApiClipper::ClipperShapes> shape) {
    if (storage == nullptr || !shape)
        return nullptr;
    PluginStorage *plugin_storage = to_storage(storage);
    return reinterpret_cast<clipper_shapes_handle *>(store_shape(*plugin_storage, std::move(shape)));
}

/* Translate small ABI enums to ClipperLib enums without leaking Clipper headers
through the C ABI.
*/
static ClipperLib::ClipType to_clip_type(clipper_operation_t operation) {
    switch (operation) {
    case CLIPPER_OPERATION_DIFFERENCE:   return ClipperLib::ctDifference;
    case CLIPPER_OPERATION_INTERSECTION: return ClipperLib::ctIntersection;
    case CLIPPER_OPERATION_UNION:        return ClipperLib::ctUnion;
    case CLIPPER_OPERATION_XOR:          return ClipperLib::ctXor;
    default:                             return ClipperLib::ctDifference;
    }
}

// Strict conversions: return false for an unknown ABI value instead of defaulting.
static bool to_known_join_type(clipper_join_type_t join_type, ClipperLib::JoinType &out) {
    switch (join_type) {
    case CLIPPER_JOIN_SQUARE: out = ClipperLib::jtSquare; return true;
    case CLIPPER_JOIN_ROUND:  out = ClipperLib::jtRound;  return true;
    case CLIPPER_JOIN_MITER:  out = ClipperLib::jtMiter;  return true;
    default:                  return false;
    }
}

// Permissive variant for the older functions: an unknown value falls back to a miter join.
static ClipperLib::JoinType to_join_type(clipper_join_type_t join_type) {
    ClipperLib::JoinType native_join = ClipperLib::jtMiter;
    to_known_join_type(join_type, native_join);
    return native_join;
}

static bool to_fill_type(clipper_fill_rule_t fill_rule, ClipperLib::PolyFillType &out) {
    switch (fill_rule) {
    case CLIPPER_FILL_NONZERO:  out = ClipperLib::pftNonZero;  return true;
    case CLIPPER_FILL_EVENODD:  out = ClipperLib::pftEvenOdd;  return true;
    case CLIPPER_FILL_POSITIVE: out = ClipperLib::pftPositive; return true;
    default:                    return false;
    }
}

static ClipperLib::EndType to_end_type(clipper_end_type_t end_type) {
    switch (end_type) {
    case CLIPPER_END_CLOSED_POLYGON: return ClipperLib::etClosedPolygon;
    case CLIPPER_END_CLOSED_LINE:    return ClipperLib::etClosedLine;
    case CLIPPER_END_OPEN_BUTT:      return ClipperLib::etOpenButt;
    case CLIPPER_END_OPEN_SQUARE:    return ClipperLib::etOpenSquare;
    case CLIPPER_END_OPEN_ROUND:     return ClipperLib::etOpenRound;
    default:                         return ClipperLib::etClosedPolygon;
    }
}

static ClipperLib::Paths safety_offset_paths(const ApiClipper::ClipperShapes &clip_shapes) {
    ClipperLib::ClipperOffset offsetter;
    offsetter.MiterLimit = DefaultMiterLimit;
    offsetter.ShortestEdgeLength = ClipperSafetyOffset * Slic3r::ClipperOffsetShortestEdgeFactor;
    offsetter.AddPaths(clip_shapes.to_paths(), DefaultJoinType, ClipperLib::etClosedPolygon);

    ClipperLib::Paths out;
    offsetter.Execute(out, ClipperSafetyOffset);
    return out;
}

/* ---- Offset strategy -------------------------------------------------
At every vertex where the offset turns against the ring (a convex vertex of a
shrinking ring, a concave vertex of a growing one), ClipperOffset emits a small
self-intersecting loop, a "spike", and its final union removes the loops.

Offsetting all rings with one ClipperOffset resolves every spike in a single
union whose sweep line crosses the whole shape, so each spike costs as much as
the shape is wide.

Several rings are therefore offset one by one, like the native ClipperUtils
offsets. A ring with a spike is cleaned by its own small union. A ring without
spike is convex in the offset direction, so its raw offset is already a simple
polygon and skips that union. One positive-fill union then merges every ring:
contours count +1 and holes -1, so islands that meet merge, while a hole that
grows past its contour stays outside.

This needs normalized rings, which ExPolygon and PolyTree sources guarantee. A
raw polygon list may hold slivers of any orientation, such as projected facets:
a clockwise sliver offset outwards turns inside out, and in a grouped union its
raw loop still counts +1, so the whole set grows. Cleaned alone it would be read
as a vanishing hole, so raw polygon and path lists keep the grouped
ClipperOffset. A lone ring keeps it too: cleaning it alone then uniting it
again would run two unions for nothing.

Both paths can leave a zero-width bridge: an edge walked twice in opposite
directions, or a hole sharing an edge with its contour. Clipper joins output
rings that touch along an edge, so it happens when overlapping raw spike loops
or an exact collapse (a feature exactly twice the offset wide) leave a
zero-width remnant. A bridge always shows as two collinear edges overlapping
along a positive length: the edges are sorted along their support lines, and a
second non-zero union, which splits the rings there, only runs when such an
overlap exists (ApiClipper::has_overlapping_edges).
*/

static ClipperLib::PolyTree offset_closed_rings(const ClipperLib::Paths &rings, double delta,
                                                ClipperLib::JoinType join_type, double miter_limit)
{
    ClipperLib::ClipperOffset offsetter;
    configure_offsetter(offsetter, delta, join_type, miter_limit);
    ClipperLib::Paths offset_paths;
    ClipperLib::Paths ring_paths;
    for (const ClipperLib::Path &ring : rings) {
        // ClipperOffset turns a lone clockwise ring counter-clockwise, so a
        // hole is offset with the opposite sign and its result reversed to
        // keep the -1 winding of a hole.
        const bool contour = ClipperLib::Orientation(ring);
        const double ring_delta = contour ? delta : -delta;
        offsetter.Clear();
        offsetter.AddPath(ring, join_type, ClipperLib::etClosedPolygon);
        if (has_offset_spike(ring, delta))
            offsetter.Execute(ring_paths, ring_delta);
        else
            offsetter.ExecuteRaw(ring_paths, ring_delta);
        for (ClipperLib::Path &path : ring_paths) {
            if (!contour)
                std::reverse(path.begin(), path.end());
            offset_paths.push_back(std::move(path));
        }
    }

    ClipperLib::Clipper clipper;
    clipper.AddPaths(offset_paths, ClipperLib::ptSubject, true);
    ClipperLib::PolyTree result;
    clipper.Execute(ClipperLib::ctUnion, result, ClipperLib::pftPositive, ClipperLib::pftPositive);
    return result;
}

static void configure_offsetter(ClipperLib::ClipperOffset &offsetter, double delta,
                                ClipperLib::JoinType join_type, double miter_limit)
{
    if (join_type == ClipperLib::jtRound)
        offsetter.ArcTolerance = miter_limit;
    else
        offsetter.MiterLimit = miter_limit;
    // Keep the same shortest-edge heuristic as ClipperUtils offsets.
    offsetter.ShortestEdgeLength = std::abs(delta * ClipperOffsetShortestEdgeFactor);
    // Same area as the bounding-rectangle clean-up of negative offsets, cheaper.
    offsetter.ShrinkWithPositiveFill = true;
}

static bool has_offset_spike(const ClipperLib::Path &ring, double delta)
{
    // Same test as ClipperOffset::OffsetPoint, on the ring as given: a nearly
    // collinear vertex emits one point, any other vertex emits a spike when its
    // turn (sign of the cross product) opposes the offset sign.
    const size_t count = ring.size();
    for (size_t index = 0; index < count; ++index) {
        const ClipperLib::IntPoint &previous = ring[(index + count - 1) % count];
        const ClipperLib::IntPoint &point = ring[index];
        const ClipperLib::IntPoint &next = ring[(index + 1) % count];
        const double ax = double(point.x() - previous.x());
        const double ay = double(point.y() - previous.y());
        const double bx = double(next.x() - point.x());
        const double by = double(next.y() - point.y());
        const double lengths = std::hypot(ax, ay) * std::hypot(bx, by);
        if (lengths == 0.)
            continue;
        const double sin_a = (ax * by - ay * bx) / lengths;
        const double cos_a = (ax * bx + ay * by) / lengths;
        if (std::abs(sin_a * delta) < 1. && cos_a > 0.)
            continue;
        if (sin_a * delta < 0.)
            return true;
    }
    return false;
}

static ClipperLib::PolyTree united_polytree(ClipperLib::PolyTree tree)
{
    ClipperLib::Paths rings;
    ClipperLib::PolyTreeToPaths(std::move(tree), rings);
    ClipperLib::Clipper clipper;
    clipper.AddPaths(rings, ClipperLib::ptSubject, true);
    ClipperLib::PolyTree united;
    clipper.Execute(ClipperLib::ctUnion, united, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    return united;
}

/*
Polyline detour around closed rings, used by clipper_detour_polyline_expolygons().

A ring is one closed path of the supplied shapes: an outer contour or a hole, both
handled the same way. The polyline crosses a ring where it goes from one side of
that ring to the other. Touching a ring and coming back on the same side is not a
crossing, and neither is a contact at the first or last point of the polyline.

The detour runs in two steps.
1. Find the crossings (detour_crossings). For each ring, every polyline segment is
   intersected with every ring edge (detour_contacts). A contact is a single point,
   or an interval when the polyline runs along an edge. Contacts that touch along
   the polyline are merged. The polyline is then sampled in the middle of the free
   stretch before and after each merged contact (detour_inside): when the two
   samples are on different sides, the contact is a crossing, placed at its end.
2. Replace polyline stretches (detour_polyline). The crossings are visited in
   polyline order. When the ring of a crossing is crossed again later, detour_exit
   chooses the walking direction on that ring and the exit: the first of those
   later crossings met while walking. The polyline between the entry and the exit
   is replaced by the ring vertices met on the way (detour_append_arc). The visit
   continues after the exit, skipping the crossings of the replaced stretch. A
   crossing whose ring is not crossed again changes nothing.

A position on the polyline is a segment index plus the fraction along that segment;
a position on a ring is an edge index plus the fraction along that edge, and a ring
vertex is stored as fraction 0 of the edge starting there.

Whether two segments intersect or overlap, and which way a ring turns, are decided
exactly with 128-bit integer products of coordinate differences. Fractions, side
samples, arc lengths and the rounded crossing points use floating point: crossings
closer than its precision can merge, and a published point can be one unit away
from the exact one. Each crossing keeps a single ring position, so a ring that
touches itself may be followed the long way round.
*/

// Return a copy of subject in which the stretch between each paired entry and exit
// crossing is replaced by the ring arc between them, as explained above.
static Polyline detour_polyline(const Polyline &subject, const ClipperLib::Paths &paths,
    clipper_detour_direction_t direction);

// A position on a ring: the edge index and the fraction along that edge, in [0, 1).
struct DetourRingPosition { size_t edge; long double fraction; };
// A contact between the polyline and one ring: the polyline positions where it begins
// and ends (equal for a single point), and the ring position and rounded point at its
// end. The point is computed from its segment's own fraction, so a polyline with many
// segments does not reduce its precision.
struct DetourContact { long double begin, end; DetourRingPosition ring_end; Point end_point; };
// A crossing: its polyline position, its ring position, the ring index and the rounded point.
struct DetourCrossing { long double position; DetourRingPosition ring_position; size_t ring; Point point; };
// A ring ready for the detour: its points without consecutive repeats, the length of
// each edge (edge i goes from point i to the next one, the last edge closes the ring)
// and whether the ring runs counter-clockwise.
struct DetourRing {
    ClipperLib::Path points;
    std::vector<long double> lengths;
    bool ccw = false;
};
// A product of two 64-bit coordinate differences can need up to 128 bits.
using DetourWide = boost::multiprecision::int128_t;

// For the crossing events[entry], whose ring is crossed next at events[next], set
// follows to the walking direction (true along the ring's point order) and return the
// index of the later crossing of the same ring met first in that direction.
static size_t detour_exit(const std::vector<DetourCrossing> &events, size_t entry, size_t next,
    const DetourRing &ring, clipper_detour_direction_t direction, bool &follows);
// Append the ring vertices met when walking from one ring position to another, without both ends.
static void detour_append_arc(Points &points, const DetourRing &ring, DetourRingPosition from,
    DetourRingPosition to, bool follows);
// Append point unless it equals the last point, so a junction is not emitted twice.
static void detour_append(Points &points, const Point &point);
// Return the crossings of every ring with at least three points, sorted along the
// polyline, then by ring index for crossings at the same position.
static std::vector<DetourCrossing> detour_crossings(const Polyline &subject,
    const std::vector<DetourRing> &rings);
// Return the contacts between the polyline and one ring without consecutive repeated
// points, in no particular order. Zero-length polyline segments are skipped.
static std::vector<DetourContact> detour_contacts(const Polyline &subject, const ClipperLib::Path &ring);
// Return the unrounded XY point at a polyline position.
static std::pair<long double, long double> detour_position(const Polyline &subject, long double position);
// Return whether a point lies inside a ring, by counting the ring edges crossed by a
// horizontal ray from the point (even-odd rule). The point must not lie on the ring.
static bool detour_inside(const ClipperLib::Path &ring, const std::pair<long double, long double> &point);
// Return the ring position at a fraction along an edge; the end of an edge (fraction 1)
// becomes the start of the next edge, so each vertex has a single position.
static DetourRingPosition detour_ring_position(size_t edge, long double fraction, size_t count);
// Return the length of the ring arc from start to end, walking along the ring's point
// order when follows is true and against it otherwise.
static long double detour_arc_length(const DetourRing &ring, DetourRingPosition start,
    DetourRingPosition end, bool follows);

static Polyline detour_polyline(const Polyline &subject, const ClipperLib::Paths &paths,
    clipper_detour_direction_t direction)
{
    if (subject.points.size() < 2)
        return subject;
    std::vector<DetourRing> rings(paths.size());
    for (size_t index = 0; index < paths.size(); ++index) {
        DetourRing &ring = rings[index];
        for (const ClipperLib::IntPoint &point : paths[index])
            if (ring.points.empty() || point != ring.points.back())
                ring.points.push_back(point);
        while (ring.points.size() > 1 && ring.points.front() == ring.points.back())
            ring.points.pop_back();
        const size_t count = ring.points.size();
        size_t lowest = 0;
        for (size_t i = 0; i < count; ++i) {
            const ClipperLib::IntPoint &a = ring.points[i], &b = ring.points[(i + 1) % count];
            ring.lengths.push_back(std::hypot(static_cast<long double>(b.x()) - a.x(),
                static_cast<long double>(b.y()) - a.y()));
            if (a.y() < ring.points[lowest].y() || (a.y() == ring.points[lowest].y() && a.x() < ring.points[lowest].x()))
                lowest = i;
        }
        // The lowest (then leftmost) vertex is always convex, so the turn made there
        // gives the orientation of the whole ring.
        if (count >= 3) {
            const ClipperLib::IntPoint &p = ring.points[(lowest + count - 1) % count], &v = ring.points[lowest],
                &q = ring.points[(lowest + 1) % count];
            ring.ccw = DetourWide(v.x() - p.x()) * (q.y() - v.y()) - DetourWide(v.y() - p.y()) * (q.x() - v.x()) > 0;
        }
    }
    const std::vector<DetourCrossing> events = detour_crossings(subject, rings);
    Polyline result;
    size_t next_original = 0;
    for (size_t entry = 0; entry < events.size();) {
        // Look for the next crossing of the same ring further along the polyline.
        const DetourCrossing &e = events[entry];
        size_t next = entry + 1;
        while (next < events.size() && (events[next].ring != e.ring || events[next].position <= e.position))
            ++next;
        if (next == events.size()) {
            ++entry;
            continue;
        }
        bool follows = true;
        const size_t exit = detour_exit(events, entry, next, rings[e.ring], direction, follows);
        // Copy the original points up to the entry, then the entry, the arc and the exit.
        while (next_original <= e.position)
            detour_append(result.points, subject.points[next_original++]);
        detour_append(result.points, e.point);
        detour_append_arc(result.points, rings[e.ring], e.ring_position, events[exit].ring_position, follows);
        detour_append(result.points, events[exit].point);
        // Resume after the exit, skipping the original points and crossings it passed over.
        next_original = size_t(events[exit].position) + 1;
        for (entry = exit + 1; entry < events.size() && events[entry].position <= events[exit].position; ++entry) {}
    }
    if (result.points.empty())
        return subject;
    while (next_original < subject.points.size())
        detour_append(result.points, subject.points[next_original++]);
    return result;
}

static size_t detour_exit(const std::vector<DetourCrossing> &events, size_t entry, size_t next,
    const DetourRing &ring, clipper_detour_direction_t direction, bool &follows)
{
    const DetourRingPosition from = events[entry].ring_position;
    const long double forward = detour_arc_length(ring, from, events[next].ring_position, true);
    const long double backward = detour_arc_length(ring, from, events[next].ring_position, false);
    switch (direction) {
    case CLIPPER_DETOUR_CW: follows = !ring.ccw; break;
    case CLIPPER_DETOUR_CCW: follows = ring.ccw; break;
    case CLIPPER_DETOUR_REVERSE: follows = false; break;
    case CLIPPER_DETOUR_SHORTEST: follows = forward <= backward; break;
    case CLIPPER_DETOUR_LONGEST: follows = forward >= backward; break;
    default: follows = true; break;
    }
    // Crossings after next are also later on the polyline. On equal distances along
    // the ring, the crossing that comes first on the polyline is kept.
    size_t exit = next;
    long double best = follows ? forward : backward;
    for (size_t candidate = next + 1; candidate < events.size(); ++candidate) {
        if (events[candidate].ring != events[entry].ring)
            continue;
        const long double length = detour_arc_length(ring, from, events[candidate].ring_position, follows);
        if (length < best) {
            best = length;
            exit = candidate;
        }
    }
    return exit;
}

static void detour_append_arc(Points &points, const DetourRing &ring, DetourRingPosition from,
    DetourRingPosition to, bool follows)
{
    // A backward walk meets the vertices of the forward walk from to to from, in reverse order.
    if (!follows)
        std::swap(from, to);
    // Walking forward, the vertices met are from.edge + 1 up to to.edge. Vertex to.edge
    // is the end itself when to.fraction is 0, so it is not appended. When the end lies
    // behind the start on the same edge, the walk goes once around the whole ring.
    const size_t count = ring.points.size();
    size_t vertices = (to.edge + count - from.edge) % count;
    if (vertices == 0 && to.fraction < from.fraction)
        vertices = count;
    if (vertices > 0 && to.fraction == 0)
        --vertices;
    for (size_t i = 0; i < vertices; ++i) {
        const ClipperLib::IntPoint &vertex =
            ring.points[(from.edge + (follows ? 1 + i : vertices - i)) % count];
        detour_append(points, Point(vertex.x(), vertex.y()));
    }
}

static void detour_append(Points &points, const Point &point)
{
    if (points.empty() || points.back() != point)
        points.push_back(point);
}

static std::vector<DetourCrossing> detour_crossings(const Polyline &subject,
    const std::vector<DetourRing> &rings)
{
    std::vector<DetourCrossing> events;
    const long double finish = subject.points.size() - 1;
    for (size_t index = 0; index < rings.size(); ++index) {
        const ClipperLib::Path &ring = rings[index].points;
        if (ring.size() < 3)
            continue;
        std::vector<DetourContact> contacts = detour_contacts(subject, ring);
        std::stable_sort(contacts.begin(), contacts.end(), [](const DetourContact &a, const DetourContact &b) {
            return a.begin < b.begin || (a.begin == b.begin && a.end < b.end);
        });
        // Merge the contacts that touch or overlap along the polyline.
        std::vector<DetourContact> groups;
        for (const DetourContact &contact : contacts) {
            if (groups.empty() || contact.begin > groups.back().end)
                groups.push_back(contact);
            else if (contact.end > groups.back().end) {
                groups.back().end = contact.end;
                groups.back().ring_end = contact.ring_end;
                groups.back().end_point = contact.end_point;
            }
        }
        // Compare the sides in the middle of the free stretches before and after each
        // contact. A contact at either polyline end has no stretch on one side.
        for (size_t i = 0; i < groups.size(); ++i) {
            const DetourContact &contact = groups[i];
            if (contact.begin == 0 || contact.end == finish)
                continue;
            const long double before = (contact.begin + (i == 0 ? 0 : groups[i - 1].end)) / 2;
            const long double after = (contact.end + (i + 1 == groups.size() ? finish : groups[i + 1].begin)) / 2;
            if (detour_inside(ring, detour_position(subject, before)) ==
                detour_inside(ring, detour_position(subject, after)))
                continue;
            events.push_back({contact.end, contact.ring_end, index, contact.end_point});
        }
    }
    std::stable_sort(events.begin(), events.end(), [](const DetourCrossing &a, const DetourCrossing &b) {
        return a.position < b.position || (a.position == b.position && a.ring < b.ring);
    });
    return events;
}

static std::vector<DetourContact> detour_contacts(const Polyline &subject, const ClipperLib::Path &ring)
{
    // value / total lies in [0, 1] exactly when value lies between 0 and total.
    const auto unit = [](const DetourWide &value, const DetourWide &total) {
        return total > 0 ? 0 <= value && value <= total : total <= value && value <= 0;
    };
    std::vector<DetourContact> contacts;
    for (size_t segment = 0; segment + 1 < subject.points.size(); ++segment) {
        const Point &p = subject.points[segment], &q = subject.points[segment + 1];
        const int64_t dx = q.x() - p.x(), dy = q.y() - p.y();
        if (dx == 0 && dy == 0)
            continue;
        for (size_t edge = 0; edge < ring.size(); ++edge) {
            const ClipperLib::IntPoint &a = ring[edge], &b = ring[(edge + 1) % ring.size()];
            const int64_t ex = b.x() - a.x(), ey = b.y() - a.y();
            const int64_t ax = a.x() - p.x(), ay = a.y() - p.y();
            // cross is zero when the segment and the edge are parallel. Otherwise
            // along / cross and across / cross are the fractions of their intersection
            // along the segment and along the edge.
            const DetourWide cross = DetourWide(dx) * ey - DetourWide(dy) * ex;
            const DetourWide along = DetourWide(ax) * ey - DetourWide(ay) * ex;
            if (cross != 0) {
                const DetourWide across = DetourWide(ax) * dy - DetourWide(ay) * dx;
                if (unit(along, cross) && unit(across, cross)) {
                    const long double t = along.convert_to<long double>() / cross.convert_to<long double>();
                    const long double u = across.convert_to<long double>() / cross.convert_to<long double>();
                    contacts.push_back({segment + t, segment + t, detour_ring_position(edge, u, ring.size()),
                        Point(coord_t(std::round(p.x() + t * dx)), coord_t(std::round(p.y() + t * dy)))});
                }
            } else if (along == 0) {
                // The segment and the edge lie on the same line: the contact is their
                // overlap, from begin to end along the segment, if they overlap at all.
                const long double t0 = dx != 0 ? static_cast<long double>(ax) / dx : static_cast<long double>(ay) / dy;
                const long double t1 = dx != 0 ? static_cast<long double>(ax + ex) / dx :
                    static_cast<long double>(ay + ey) / dy;
                const long double begin = std::max(0.L, std::min(t0, t1));
                const long double end = std::min(1.L, std::max(t0, t1));
                if (begin <= end) {
                    // The overlap ends at an edge vertex, kept exact, or at the segment end q,
                    // whose fraction along the edge is then computed.
                    const long double u = end == t0 ? 0 : end == t1 ? 1 :
                        std::clamp(ex != 0 ? (end * dx - ax) / ex : (end * dy - ay) / ey, 0.L, 1.L);
                    const Point end_point = end == t0 ? Point(a.x(), a.y()) : end == t1 ? Point(b.x(), b.y()) : q;
                    contacts.push_back({segment + begin, segment + end, detour_ring_position(edge, u, ring.size()), end_point});
                }
            }
        }
    }
    return contacts;
}

static std::pair<long double, long double> detour_position(const Polyline &subject, long double position)
{
    const size_t segment = std::min(size_t(position), subject.points.size() - 2);
    const long double t = position - segment;
    const Point &a = subject.points[segment], &b = subject.points[segment + 1];
    return {a.x() + t * (static_cast<long double>(b.x()) - a.x()),
            a.y() + t * (static_cast<long double>(b.y()) - a.y())};
}

static bool detour_inside(const ClipperLib::Path &ring, const std::pair<long double, long double> &point)
{
    bool inside = false;
    for (size_t i = 0; i < ring.size(); ++i) {
        const ClipperLib::IntPoint &a = ring[i], &b = ring[(i + 1) % ring.size()];
        if ((a.y() > point.second) != (b.y() > point.second) &&
            point.first < a.x() + (point.second - a.y()) *
                (static_cast<long double>(b.x()) - a.x()) / (static_cast<long double>(b.y()) - a.y()))
            inside = !inside;
    }
    return inside;
}

static DetourRingPosition detour_ring_position(size_t edge, long double fraction, size_t count)
{
    if (fraction == 1)
        return {(edge + 1) % count, 0};
    return {edge, fraction};
}

static long double detour_arc_length(const DetourRing &ring, DetourRingPosition start,
    DetourRingPosition end, bool follows)
{
    if (!follows)
        std::swap(start, end);
    if (start.edge == end.edge && end.fraction >= start.fraction)
        return (end.fraction - start.fraction) * ring.lengths[start.edge];
    long double length = (1 - start.fraction) * ring.lengths[start.edge] + end.fraction * ring.lengths[end.edge];
    for (size_t edge = (start.edge + 1) % ring.points.size(); edge != end.edge; edge = (edge + 1) % ring.points.size())
        length += ring.lengths[edge];
    return length;
}

} // namespace Slic3r

extern "C" {

/* ---- Adapter creation ------------------------------------------------
These functions allocate only a small ClipperShapes adapter in storage. Source
geometry is intentionally not copied, keeping setup cheap for plugin code that
creates short-lived boolean operations.
*/

clipper_shapes_handle *clipper_shapes_create_empty(storage_handle *storage) {
    return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_path_list_shapes(Slic3r::ClipperLib::Paths{}));
}

clipper_shapes_handle *clipper_shapes_from_polygon(storage_handle *storage, const polygon_handle *polygon) {
    return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_polygon_shapes(Slic3r::to_polygon(polygon)));
}

clipper_shapes_handle *clipper_shapes_from_polyline(storage_handle *storage, const polyline_handle *polyline) {
    return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_polyline_shapes(Slic3r::to_polyline(polyline)));
}

clipper_shapes_handle *clipper_shapes_from_polygons(storage_handle *storage, const polygon_collection_handle *polygons) {
    return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_polygons_shapes(Slic3r::to_polygons(polygons)));
}

clipper_shapes_handle *clipper_shapes_from_expolygon(storage_handle *storage, const expolygon_handle *expolygon) {
    return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_expolygon_shapes(Slic3r::to_expolygon(expolygon)));
}

clipper_shapes_handle *clipper_shapes_from_expolygons(storage_handle *storage, const expolygon_collection_handle *expolygons) {
    return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_expolygons_shapes(Slic3r::to_expolygons(expolygons)));
}

int32_t clipper_shapes_empty(const clipper_shapes_handle *shapes) {
    const Slic3r::ApiClipper::ClipperShapes *source = Slic3r::to_shapes(shapes);
    return source == nullptr || source->empty() ? 1 : 0;
}

uint32_t clipper_shapes_path_count(const clipper_shapes_handle *shapes) {
    const Slic3r::ApiClipper::ClipperShapes *source = Slic3r::to_shapes(shapes);
    return source == nullptr ? 0 : source->path_count();
}

c_bounding_box clipper_shapes_bounding_box(const clipper_shapes_handle *shapes) {
    const Slic3r::ApiClipper::ClipperShapes *source = Slic3r::to_shapes(shapes);
    return source == nullptr ? c_bounding_box{} : Slic3r::to_c_bounding_box(source->bounding_box());
}

/* ---- Boolean operations ----------------------------------------------
Boolean operations create a PolyTree-backed ClipperShapes result. Returning a
ClipperShapes handle instead of immediately materializing Polygons/ExPolygons
lets callers chain operations without repeatedly converting between formats.
*/

static clipper_shapes_handle *clipper_execute_impl(storage_handle *storage,
                                                   clipper_operation_t operation,
                                                   const clipper_shapes_handle *subject,
                                                   const clipper_shapes_handle *clip,
                                                   bool apply_safety_offset,
                                                   Slic3r::ClipperLib::PolyFillType fill_type)
{
    const Slic3r::ApiClipper::ClipperShapes *subject_shapes = Slic3r::to_shapes(subject);
    const Slic3r::ApiClipper::ClipperShapes *clip_shapes = Slic3r::to_shapes(clip);
    if (storage == nullptr || subject_shapes == nullptr)
        return nullptr;

    Slic3r::ClipperLib::Clipper clipper;
    subject_shapes->add_to_clipper(clipper, Slic3r::ClipperLib::ptSubject, true);
    if (clip_shapes != nullptr) {
        const bool can_apply_safety_offset =
            operation == CLIPPER_OPERATION_DIFFERENCE || operation == CLIPPER_OPERATION_INTERSECTION;
        if (can_apply_safety_offset && apply_safety_offset) {
            const Slic3r::ClipperLib::Paths expanded_clip = Slic3r::safety_offset_paths(*clip_shapes);
            clipper.AddPaths(expanded_clip, Slic3r::ClipperLib::ptClip, true);
        } else {
            clip_shapes->add_to_clipper(clipper, Slic3r::ClipperLib::ptClip, true);
        }
    }

    // PolyTree keeps the contour/hole hierarchy needed by to_expolygons().
    Slic3r::ClipperLib::PolyTree result;
    clipper.Execute(Slic3r::to_clip_type(operation), result, fill_type, fill_type);
    return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_polytree_shapes(std::move(result)));
}

clipper_shapes_handle *clipper_execute(storage_handle *storage,
                                       clipper_operation_t operation,
                                       const clipper_shapes_handle *subject,
                                       const clipper_shapes_handle *clip)
{
    return clipper_execute_impl(storage, operation, subject, clip, false, Slic3r::ClipperLib::pftNonZero);
}

clipper_shapes_handle *clipper_diff(storage_handle *storage,
                                    const clipper_shapes_handle *subject,
                                    const clipper_shapes_handle *clip)
{
    return clipper_execute(storage, CLIPPER_OPERATION_DIFFERENCE, subject, clip);
}

clipper_shapes_handle *clipper_intersection(storage_handle *storage,
                                           const clipper_shapes_handle *subject,
                                           const clipper_shapes_handle *clip)
{
    return clipper_execute(storage, CLIPPER_OPERATION_INTERSECTION, subject, clip);
}

int32_t clipper_split(storage_handle *storage,
                      const clipper_shapes_handle *subject,
                      const clipper_shapes_handle *clip,
                      clipper_shapes_handle **inside,
                      clipper_shapes_handle **outside)
{
    const Slic3r::ApiClipper::ClipperShapes *subject_shapes = Slic3r::to_shapes(subject);
    const Slic3r::ApiClipper::ClipperShapes *clip_shapes = Slic3r::to_shapes(clip);
    if (storage == nullptr || subject_shapes == nullptr || inside == nullptr || outside == nullptr)
        return 0;

    // Each operation runs on its own Clipper fed with the same inputs, so the two results are cut along
    // the same clip edges. add_to_clipper() keeps the kind of each adapter: a polyline operand stays
    // an open path, as in clipper_intersection() and clipper_diff().
    auto run = [&](Slic3r::ClipperLib::ClipType clip_type) {
        Slic3r::ClipperLib::Clipper clipper;
        subject_shapes->add_to_clipper(clipper, Slic3r::ClipperLib::ptSubject, true);
        if (clip_shapes != nullptr)
            clip_shapes->add_to_clipper(clipper, Slic3r::ClipperLib::ptClip, true);
        Slic3r::ClipperLib::PolyTree result;
        clipper.Execute(clip_type, result, Slic3r::ClipperLib::pftNonZero, Slic3r::ClipperLib::pftNonZero);
        return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_polytree_shapes(std::move(result)));
    };
    *inside = run(Slic3r::ClipperLib::ctIntersection);
    *outside = run(Slic3r::ClipperLib::ctDifference);
    return 1;
}

clipper_shapes_handle *clipper_diff_with_safety_offset(storage_handle *storage,
                                                       const clipper_shapes_handle *subject,
                                                       const clipper_shapes_handle *clip)
{
    return clipper_execute_impl(storage, CLIPPER_OPERATION_DIFFERENCE, subject, clip, true, Slic3r::ClipperLib::pftNonZero);
}

clipper_shapes_handle *clipper_intersection_with_safety_offset(storage_handle *storage,
                                                              const clipper_shapes_handle *subject,
                                                              const clipper_shapes_handle *clip)
{
    return clipper_execute_impl(storage, CLIPPER_OPERATION_INTERSECTION, subject, clip, true, Slic3r::ClipperLib::pftNonZero);
}

// Union of the subject paths with the given fill rule; an unknown rule returns NULL.
clipper_shapes_handle *clipper_union(storage_handle *storage,
                                     const clipper_shapes_handle *subject,
                                     clipper_fill_rule_t fill_rule)
{
    // Validate the rule, then run the same path as every other boolean operation with that fill type
    // for the subject and the (absent) clip.
    Slic3r::ClipperLib::PolyFillType fill_type;
    if (!Slic3r::to_fill_type(fill_rule, fill_type))
        return nullptr;
    return clipper_execute_impl(storage, CLIPPER_OPERATION_UNION, subject, nullptr, false, fill_type);
}

// Stores the result in a new storage-owned collection through the C++ PluginStorage (not the extern "C"
// storage_new_expolygons(), which MSVC assumes never throws), so an allocation failure reaches the caller try/catch.
namespace Slic3r {
static expolygon_collection_handle *store_expolygons(storage_handle *storage, ExPolygons &&result) {
    return reinterpret_cast<expolygon_collection_handle *>(
        to_storage(storage)->store(std::make_unique<ExPolygons>(std::move(result))));
}
} // namespace Slic3r

// Native union_ex(Polygons, fill_type), stored as is in a new storage-owned collection.
expolygon_collection_handle *clipper_union_ex(storage_handle *storage,
                                              const polygon_collection_handle *polygons,
                                              clipper_fill_rule_t fill_rule)
{
    Slic3r::ClipperLib::PolyFillType fill_type;
    if (storage == nullptr || polygons == nullptr || !Slic3r::to_fill_type(fill_rule, fill_type))
        return nullptr;
    // The native result is computed first: an exception (bad_alloc, ClipperLib range error) must not cross the
    // C boundary nor leave an empty collection in the storage.
    try {
        Slic3r::ExPolygons result = Slic3r::union_ex(*Slic3r::to_polygons(polygons), fill_type);
        return Slic3r::store_expolygons(storage, std::move(result));
    } catch (...) {
        return nullptr;
    }
}

// Native offset_ex(ExPolygons, ...), stored as is. A zero delta is accepted: native does not assert on it here.
// Non-finite values are rejected because ClipperLib throws on them, and no exception may cross the C boundary.
expolygon_collection_handle *clipper_offset_ex(storage_handle *storage,
                                               const expolygon_collection_handle *src,
                                               double delta,
                                               clipper_join_type_t join_type,
                                               double miter_limit)
{
    Slic3r::ClipperLib::JoinType native_join;
    if (storage == nullptr || src == nullptr || !std::isfinite(delta) || !std::isfinite(miter_limit) ||
        !Slic3r::to_known_join_type(join_type, native_join))
        return nullptr;
    // Compute first, then store; an internal error returns NULL with nothing added to the storage.
    try {
        Slic3r::ExPolygons result = Slic3r::offset_ex(*Slic3r::to_expolygons(src), delta, native_join, miter_limit);
        return Slic3r::store_expolygons(storage, std::move(result));
    } catch (...) {
        return nullptr;
    }
}

// Native offset2_ex(ExPolygons, ...), stored as is. delta2 == 0 is rejected: native offset_paths asserts
// offset != 0 for the second pass. Non-finite values are rejected as in clipper_offset_ex().
expolygon_collection_handle *clipper_offset2_ex(storage_handle *storage,
                                                const expolygon_collection_handle *src,
                                                double delta1,
                                                double delta2,
                                                clipper_join_type_t join_type,
                                                double miter_limit)
{
    Slic3r::ClipperLib::JoinType native_join;
    if (storage == nullptr || src == nullptr || !std::isfinite(delta1) || !std::isfinite(delta2) ||
        !std::isfinite(miter_limit) || delta2 == 0. || !Slic3r::to_known_join_type(join_type, native_join))
        return nullptr;
    // Compute first, then store; an internal error returns NULL with nothing added to the storage.
    try {
        Slic3r::ExPolygons result =
            Slic3r::offset2_ex(*Slic3r::to_expolygons(src), delta1, delta2, native_join, miter_limit);
        return Slic3r::store_expolygons(storage, std::move(result));
    } catch (...) {
        return nullptr;
    }
}

expolygon_collection_handle *clipper_diff_ex(storage_handle *storage,
    const expolygon_collection_handle *subject, const expolygon_collection_handle *clip)
{
    if (storage == nullptr || subject == nullptr || clip == nullptr)
        return nullptr;
    try {
        Slic3r::ExPolygons result = Slic3r::diff_ex(*Slic3r::to_expolygons(subject), *Slic3r::to_expolygons(clip));
        return Slic3r::store_expolygons(storage, std::move(result));
    } catch (...) {
        return nullptr;
    }
}

expolygon_collection_handle *clipper_intersection_ex(storage_handle *storage,
    const expolygon_collection_handle *subject, const expolygon_collection_handle *clip)
{
    if (storage == nullptr || subject == nullptr || clip == nullptr)
        return nullptr;
    try {
        Slic3r::ExPolygons result = Slic3r::intersection_ex(*Slic3r::to_expolygons(subject), *Slic3r::to_expolygons(clip));
        return Slic3r::store_expolygons(storage, std::move(result));
    } catch (...) {
        return nullptr;
    }
}

expolygon_collection_handle *clipper_variable_offset_inner_ex(storage_handle *storage,
    const expolygon_handle *expolygon, const float *deltas, uint32_t delta_count)
{
    if (storage == nullptr || expolygon == nullptr)
        return nullptr;
    try {
        const Slic3r::ExPolygon &input = *Slic3r::to_expolygon(expolygon);
        // Reject orientations and delta layouts the native inset routine cannot use.
        size_t expected = input.contour.size();
        if (!(Slic3r::ClipperLib::Area(input.contour.points) > 0.))
            return nullptr;
        for (const Slic3r::Polygon &hole : input.holes) {
            if (!(Slic3r::ClipperLib::Area(hole.points) < 0.))
                return nullptr;
            expected += hole.size();
        }
        if (expected != delta_count || (expected > 0 && deltas == nullptr))
            return nullptr;
        for (size_t i = 0; i < expected; ++i)
            if (!std::isfinite(deltas[i]) || deltas[i] > 0.f)
                return nullptr;
        // The native routine needs one delta vector per ring, not the ABI flat array.
        std::vector<std::vector<float>> ring_deltas;
        ring_deltas.emplace_back(deltas, deltas + input.contour.size());
        size_t position = input.contour.size();
        for (const Slic3r::Polygon &hole : input.holes) {
            ring_deltas.emplace_back(deltas + position, deltas + position + hole.size());
            position += hole.size();
        }
        Slic3r::ExPolygons result = Slic3r::variable_offset_inner_ex(input, ring_deltas, 2.0);
        return Slic3r::store_expolygons(storage, std::move(result));
    } catch (...) {
        return nullptr;
    }
}

polygon_collection_handle *clipper_expand(storage_handle *storage,
    const polygon_collection_handle *polygons, double delta)
{
    if (storage == nullptr || polygons == nullptr || !std::isfinite(delta) || delta <= 0.)
        return nullptr;
    try {
        Slic3r::Polygons result = Slic3r::expand(*Slic3r::to_polygons(polygons), delta);
        return reinterpret_cast<polygon_collection_handle *>(
            Slic3r::to_storage(storage)->store(std::make_unique<Slic3r::Polygons>(std::move(result))));
    } catch (...) {
        return nullptr;
    }
}

clipper_shapes_handle *clipper_union_with_safety_offset(storage_handle *storage,
                                                        const clipper_shapes_handle *subject) {
    const Slic3r::ApiClipper::ClipperShapes *subject_shapes = Slic3r::to_shapes(subject);
    if (storage == nullptr || subject_shapes == nullptr)
        return nullptr;

    Slic3r::ClipperLib::Paths expanded = Slic3r::safety_offset_paths(*subject_shapes);

    Slic3r::ClipperLib::Clipper clipper;
    clipper.AddPaths(expanded, Slic3r::ClipperLib::ptSubject, true);

    Slic3r::ClipperLib::PolyTree result;
    clipper.Execute(Slic3r::ClipperLib::ctUnion,
                    result,
                    Slic3r::ClipperLib::pftNonZero,
                    Slic3r::ClipperLib::pftNonZero);

    return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_polytree_shapes(std::move(result)));
}

clipper_shapes_handle *clipper_union2(storage_handle *storage,
                                      const clipper_shapes_handle *subject1,
                                      const clipper_shapes_handle *subject2)
{
    const Slic3r::ApiClipper::ClipperShapes *first = Slic3r::to_shapes(subject1);
    const Slic3r::ApiClipper::ClipperShapes *second = Slic3r::to_shapes(subject2);
    if (storage == nullptr || first == nullptr || second == nullptr)
        return nullptr;

    Slic3r::ClipperLib::Clipper clipper;
    first->add_to_clipper(clipper, Slic3r::ClipperLib::ptSubject, true);
    second->add_to_clipper(clipper, Slic3r::ClipperLib::ptSubject, true);

    Slic3r::ClipperLib::PolyTree result;
    clipper.Execute(Slic3r::ClipperLib::ctUnion, result, Slic3r::ClipperLib::pftNonZero, Slic3r::ClipperLib::pftNonZero);
    return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_polytree_shapes(std::move(result)));
}

// Concatenation intentionally does not run a boolean operation. It creates a
// fresh PathListShapes accumulator containing the raw paths of both inputs.
clipper_shapes_handle *clipper_concat(storage_handle *storage,
                                      const clipper_shapes_handle *first,
                                      const clipper_shapes_handle *second)
{
    const Slic3r::ApiClipper::ClipperShapes *first_shapes = Slic3r::to_shapes(first);
    const Slic3r::ApiClipper::ClipperShapes *second_shapes = Slic3r::to_shapes(second);
    if (storage == nullptr || (first_shapes == nullptr && second_shapes == nullptr))
        return nullptr;

    Slic3r::ClipperLib::Paths paths = first_shapes != nullptr ? first_shapes->to_paths() : Slic3r::ClipperLib::Paths{};
    if (second_shapes != nullptr) {
        Slic3r::ClipperLib::Paths other = second_shapes->to_paths();
        paths.insert(paths.end(),
                     std::make_move_iterator(other.begin()),
                     std::make_move_iterator(other.end()));
    }
    return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_path_list_shapes(std::move(paths)));
}

clipper_shapes_handle *clipper_concat_replace(storage_handle *storage,
                                              clipper_shapes_handle *first,
                                              const clipper_shapes_handle *second)
{
    if (storage == nullptr)
        return nullptr;
    // A null handle is an empty operand: keep first when there is nothing to
    // append, and copy second when there is nothing to append to.
    if (second == nullptr)
        return first;
    if (first == nullptr)
        return clipper_concat(storage, nullptr, second);

    Slic3r::PluginStorage *plugin_storage = Slic3r::to_storage(storage);
    Slic3r::ApiClipper::ClipperShapes *first_shapes = Slic3r::to_shapes(first);
    const Slic3r::ApiClipper::ClipperShapes *second_shapes = Slic3r::to_shapes(second);
    if (first_shapes == nullptr || second_shapes == nullptr)
        return nullptr;

    clipper_shapes_handle *result =
        reinterpret_cast<clipper_shapes_handle *>(first_shapes->concat(*second_shapes, *plugin_storage));
    if (result != nullptr && result != first)
        storage_free(storage, first);
    return result;
}

/* ---- Offset operations ------------------------------------------------
ClipperOffset works from raw Paths, so every ClipperShapes implementation first
materializes itself as Paths. Several closed rings from an ExPolygon or PolyTree
source are offset ring by ring (see "Offset strategy"); a lone ring, raw polygon
or path lists and open or closed-line paths use one grouped ClipperOffset. The result is again kept as a PolyTree-backed
ClipperShapes so it can be chained or converted later.
*/

clipper_shapes_handle *clipper_offset(storage_handle *storage,
                                      const clipper_shapes_handle *subject,
                                      double delta,
                                      clipper_join_type_t join_type,
                                      double miter_limit,
                                      clipper_end_type_t end_type)
{
    const Slic3r::ApiClipper::ClipperShapes *subject_shapes = Slic3r::to_shapes(subject);
    if (storage == nullptr || subject_shapes == nullptr)
        return nullptr;

    const Slic3r::ClipperLib::JoinType native_join = Slic3r::to_join_type(join_type);
    const Slic3r::ClipperLib::EndType native_end = Slic3r::to_end_type(end_type);
    const Slic3r::ClipperLib::Paths rings = subject_shapes->to_paths();
    // An empty path (for example an empty ExPolygon) is not a ring.
    const std::ptrdiff_t ring_count = std::count_if(rings.begin(), rings.end(),
        [](const Slic3r::ClipperLib::Path &ring) { return !ring.empty(); });
    Slic3r::ClipperLib::PolyTree result;
    if (native_end == Slic3r::ClipperLib::etClosedPolygon && subject_shapes->has_normalized_rings() &&
        ring_count > 1) {
        result = Slic3r::offset_closed_rings(rings, delta, native_join, miter_limit);
    } else {
        Slic3r::ClipperLib::ClipperOffset offsetter;
        Slic3r::configure_offsetter(offsetter, delta, native_join, miter_limit);
        offsetter.AddPaths(rings, native_join, native_end);
        offsetter.Execute(result, delta);
    }
    if (Slic3r::ApiClipper::has_overlapping_edges(result))
        result = Slic3r::united_polytree(std::move(result));
    return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_polytree_shapes(std::move(result)));
}

clipper_shapes_handle *clipper_offset2(storage_handle *storage,
                                       const clipper_shapes_handle *subject,
                                       double delta1,
                                       double delta2,
                                       clipper_join_type_t join_type,
                                       double miter_limit,
                                       clipper_end_type_t end_type)
{
    // Store only the final result for the caller. The intermediate offset is
    // explicitly freed from the same storage before returning.
    clipper_shapes_handle *first = clipper_offset(storage, subject, delta1, join_type, miter_limit, end_type);
    clipper_shapes_handle *second = clipper_offset(storage, first, delta2, join_type, miter_limit, end_type);
    storage_free(storage, first);
    return second;
}

// Delegate to the native per-polyline offset and union so results match ClipperUtils point for point; the united polygons are copied into a path list.
clipper_shapes_handle *clipper_offset_polylines(storage_handle *storage,
                                                const polyline_collection_handle *polylines,
                                                double delta,
                                                clipper_join_type_t join_type,
                                                double miter_limit,
                                                clipper_end_type_t end_type)
{
    if (storage == nullptr || polylines == nullptr || delta <= 0.)
        return nullptr;
    const Slic3r::Polygons offset = Slic3r::offset(*Slic3r::to_polylines(polylines), delta,
                                                   Slic3r::to_join_type(join_type), miter_limit,
                                                   Slic3r::to_end_type(end_type));
    Slic3r::ClipperLib::Paths paths;
    paths.reserve(offset.size());
    for (const Slic3r::Polygon &polygon : offset)
        paths.emplace_back(polygon.points);
    return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_path_list_shapes(std::move(paths)));
}

expolygon_collection_handle *clipper_variable_offset_inner(
    storage_handle *storage, const expolygon_handle *source,
    const float *deltas, uint32_t delta_count, double miter_limit)
{
    if (storage == nullptr || source == nullptr || (delta_count != 0 && deltas == nullptr) ||
        !std::isfinite(miter_limit) || miter_limit < 1.)
        return nullptr;
    const Slic3r::ExPolygon &input = *Slic3r::to_expolygon(source);
    size_t count = input.contour.size();
    for (const auto &hole : input.holes)
        count += hole.size();
    if (count != delta_count)
        return nullptr;
    if (input.contour.empty()) {
        return input.holes.empty() ? storage_new_expolygons(storage) : nullptr;
    }
    if (input.contour.size() < 3 || !input.contour.is_counter_clockwise())
        return nullptr;
    for (const auto &hole : input.holes)
        if (hole.size() < 3 || !hole.is_clockwise())
            return nullptr;

    double maximum_delta = 0.;
    for (uint32_t index = 0; index < delta_count; ++index) {
        if (!std::isfinite(deltas[index]) || deltas[index] > 0.f)
            return nullptr;
        maximum_delta = std::max(maximum_delta, -double(deltas[index]));
    }
    // Reject unsafe margins before any variable-offset coordinate arithmetic.
    // A double rounds hiRange up, so use the next lower representable value.
    const double limit = std::nextafter(double(Slic3r::ClipperLib::hiRange), 0.);
    const double margin = std::ceil(maximum_delta * std::max(2., miter_limit)) + 2. * Slic3r::SCALED_EPSILON;
    if (!std::isfinite(margin) || margin > limit)
        return nullptr;
    const auto bounds = Slic3r::get_extents(input);
    const double safe = limit - margin;
    if (double(bounds.min.x()) < -safe || double(bounds.min.y()) < -safe ||
        double(bounds.max.x()) > safe || double(bounds.max.y()) > safe)
        return nullptr;

    // Split the flat ABI array into one delta vector per ring: contour first, then holes.
    // delta_count was checked above against the total vertex count, so every slice stays in bounds.
    std::vector<std::vector<float>> ring_deltas;
    ring_deltas.reserve(input.holes.size() + 1);

    const float *const contour_ring_begin = deltas;
    const float *const contour_ring_end = contour_ring_begin + input.contour.size();
    ring_deltas.emplace_back(contour_ring_begin, contour_ring_end);

    const float *hole_ring_begin = contour_ring_end;
    for (const Slic3r::Polygon &hole_ring : input.holes) {
        const float *const hole_ring_end = hole_ring_begin + hole_ring.size();
        ring_deltas.emplace_back(hole_ring_begin, hole_ring_end);
        hole_ring_begin = hole_ring_end;
    }
    assert(hole_ring_begin == deltas + delta_count);
    Slic3r::ExPolygons output = Slic3r::variable_offset_inner_ex(input, ring_deltas, miter_limit);
    auto *result = storage_new_expolygons(storage);
    *Slic3r::to_expolygons(result) = std::move(output);
    return result;
}

/* ---- Conversion back to public geometry handles ----------------------
These functions materialize the abstract ClipperShapes result into normal ABI
collections. The output collections are also storage-owned and can be returned
from plugin code or freed with storage_free().
*/

polygon_collection_handle *clipper_shapes_to_polygons(storage_handle *storage, const clipper_shapes_handle *shapes) {
    const Slic3r::ApiClipper::ClipperShapes *source = Slic3r::to_shapes(shapes);
    if (storage == nullptr || source == nullptr)
        return nullptr;

    polygon_collection_handle *out_handle = storage_new_polygons(storage);
    Slic3r::Polygons *out = Slic3r::to_polygons(out_handle);
    out->clear();
    for (const Slic3r::Polygon &polygon : source->to_polygons())
        out->emplace_back(polygon);
    return out_handle;
}

expolygon_collection_handle *clipper_shapes_to_expolygons(storage_handle *storage, const clipper_shapes_handle *shapes) {
    const Slic3r::ApiClipper::ClipperShapes *source = Slic3r::to_shapes(shapes);
    if (storage == nullptr || source == nullptr)
        return nullptr;

    expolygon_collection_handle *out_handle = storage_new_expolygons(storage);
    *Slic3r::to_expolygons(out_handle) = source->to_expolygons();
    return out_handle;
}

void clipper_shapes_replace_polygons(polygon_collection_handle *dst, const clipper_shapes_handle *shapes) {
    const Slic3r::ApiClipper::ClipperShapes *source = Slic3r::to_shapes(shapes);
    if (dst == nullptr || source == nullptr)
        return;

    Slic3r::Polygons &out = *Slic3r::to_polygons(dst);
    // TODO: optimiser, en réutilisant les objets MultiPoint déjà alloués dans la collection de destination, plutôt
    // que de tout clear() et ré-emplace_back() à chaque fois. Mais cela demande de faire de nouvelles méthodes dans source
    // pour remplir directement un object déjà crée.
    out.clear();
    for (const Slic3r::Polygon &polygon : source->to_polygons())
        out.emplace_back(polygon);
}

void clipper_shapes_replace_expolygons(expolygon_collection_handle *dst, const clipper_shapes_handle *shapes) {
    const Slic3r::ApiClipper::ClipperShapes *source = Slic3r::to_shapes(shapes);
    if (dst == nullptr || source == nullptr)
        return;

    *Slic3r::to_expolygons(dst) = source->to_expolygons();
}

expolygon_collection_handle *clipper_clip_expolygons_with_subject_bbox(storage_handle *storage,
                                                                       const expolygon_collection_handle *src,
                                                                       c_bounding_box bbox)
{
    if (storage == nullptr || src == nullptr)
        return nullptr;

    expolygon_collection_handle *out_handle = storage_new_expolygons(storage);
    *Slic3r::to_expolygons(out_handle) =
        Slic3r::ClipperUtils::clip_clipper_expolygons_with_subject_bbox(*Slic3r::to_expolygons(src),
                                                                        Slic3r::to_bounding_box(bbox));
    return out_handle;
}

clipper_shapes_handle *clipper_clip_shapes_with_subject_bbox(storage_handle *storage,
                                                             const clipper_shapes_handle *src,
                                                             c_bounding_box bbox)
{
    const Slic3r::ApiClipper::ClipperShapes *source = Slic3r::to_shapes(src);
    if (storage == nullptr || source == nullptr)
        return nullptr;

    // This is a pre-filter for later Clipper operations. It deliberately keeps a
    // flat path list instead of rebuilding ExPolygon topology, because the next
    // boolean operation will rebuild the final contour/hole tree if needed.
    Slic3r::Polygons polygons;
    for (const Slic3r::ClipperLib::Path &path : source->to_paths()) {
        Slic3r::Polygon polygon;
        polygon.points.reserve(path.size());
        for (const Slic3r::ClipperLib::IntPoint &point : path)
            polygon.points.emplace_back(point.x(), point.y());
        polygons.emplace_back(std::move(polygon));
    }

    Slic3r::Polygons clipped =
        Slic3r::ClipperUtils::clip_clipper_polygons_with_subject_bbox(polygons,
                                                                      Slic3r::to_bounding_box(bbox));

    Slic3r::ClipperLib::Paths paths;
    paths.reserve(clipped.size());
    for (const Slic3r::Polygon &polygon : clipped) {
        Slic3r::ClipperLib::Path path;
        path.reserve(polygon.points.size());
        for (const Slic3r::Point &point : polygon.points)
            path.emplace_back(point.x(), point.y());
        paths.emplace_back(std::move(path));
    }

    return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_path_list_shapes(std::move(paths)));
}

polyline_collection_handle *clipper_diff_polyline_expolygons(storage_handle *storage,
                                                             const polyline_handle *subject,
                                                             const expolygon_collection_handle *clip)
{
    if (storage == nullptr || subject == nullptr)
        return nullptr;

    polyline_collection_handle *out_handle = storage_new_polylines(storage);
    Slic3r::Polylines &out = *Slic3r::to_polylines(out_handle);
    if (clip == nullptr || Slic3r::to_expolygons(clip)->empty())
        out.push_back(*Slic3r::to_polyline(subject));
    else
        out = Slic3r::diff_pl(*Slic3r::to_polyline(subject), *Slic3r::to_expolygons(clip));
    return out_handle;
}

polyline_collection_handle *clipper_diff_polylines_expolygons(storage_handle *storage,
                                                             const polyline_collection_handle *subject,
                                                             const expolygon_collection_handle *clip)
{
    if (storage == nullptr || subject == nullptr)
        return nullptr;

    polyline_collection_handle *out_handle = storage_new_polylines(storage);
    Slic3r::Polylines &out = *Slic3r::to_polylines(out_handle);
    if (clip == nullptr || Slic3r::to_expolygons(clip)->empty())
        out = *Slic3r::to_polylines(subject);
    else if (!Slic3r::to_polylines(subject)->empty())
        out = Slic3r::diff_pl(*Slic3r::to_polylines(subject), *Slic3r::to_expolygons(clip));
    return out_handle;
}

polyline_collection_handle *clipper_intersection_polyline_expolygons(storage_handle *storage,
                                                                     const polyline_handle *subject,
                                                                     const expolygon_collection_handle *clip)
{
    if (storage == nullptr || subject == nullptr)
        return nullptr;

    polyline_collection_handle *out_handle = storage_new_polylines(storage);
    Slic3r::Polylines &out = *Slic3r::to_polylines(out_handle);
    if (clip != nullptr && !Slic3r::to_expolygons(clip)->empty())
        out = Slic3r::intersection_pl(*Slic3r::to_polyline(subject), *Slic3r::to_expolygons(clip));
    return out_handle;
}

polyline_collection_handle *clipper_intersection_polylines_expolygons(storage_handle *storage,
                                                                     const polyline_collection_handle *subject,
                                                                     const expolygon_collection_handle *clip)
{
    if (storage == nullptr || subject == nullptr)
        return nullptr;

    polyline_collection_handle *out_handle = storage_new_polylines(storage);
    Slic3r::Polylines &out = *Slic3r::to_polylines(out_handle);
    if (!Slic3r::to_polylines(subject)->empty() && clip != nullptr && !Slic3r::to_expolygons(clip)->empty())
        out = Slic3r::intersection_pl(*Slic3r::to_polylines(subject), *Slic3r::to_expolygons(clip));
    return out_handle;
}

int32_t clipper_split_polyline_expolygons(storage_handle *storage,
                                          const polyline_handle *subject,
                                          const expolygon_collection_handle *clip,
                                          polyline_collection_handle **inside,
                                          polyline_collection_handle **outside)
{
    if (storage == nullptr || subject == nullptr || inside == nullptr || outside == nullptr)
        return 0;

    polyline_collection_handle *inside_handle = storage_new_polylines(storage);
    polyline_collection_handle *outside_handle = storage_new_polylines(storage);
    if (clip == nullptr || Slic3r::to_expolygons(clip)->empty()) {
        Slic3r::to_polylines(outside_handle)->push_back(*Slic3r::to_polyline(subject));
    } else {
        std::pair<Slic3r::Polylines, Slic3r::Polylines> split =
            Slic3r::split_pl(*Slic3r::to_polyline(subject), *Slic3r::to_expolygons(clip));
        *Slic3r::to_polylines(inside_handle) = std::move(split.first);
        *Slic3r::to_polylines(outside_handle) = std::move(split.second);
    }
    *inside = inside_handle;
    *outside = outside_handle;
    return 1;
}

polyline_handle *clipper_detour_polyline_expolygons(storage_handle *storage,
    const polyline_handle *polyline, const clipper_shapes_handle *rings,
    clipper_detour_direction_t direction)
{
    if (storage == nullptr || polyline == nullptr || rings == nullptr ||
        direction < CLIPPER_DETOUR_CW || direction > CLIPPER_DETOUR_LONGEST)
        return nullptr;
    try {
        std::unique_ptr<Slic3r::Polyline> result = std::make_unique<Slic3r::Polyline>(
            Slic3r::detour_polyline(*Slic3r::to_polyline(polyline), Slic3r::to_shapes(rings)->to_paths(), direction));
        return reinterpret_cast<polyline_handle *>(Slic3r::to_storage(storage)->store(std::move(result)));
    } catch (...) {
        return nullptr;
    }
}

} // extern "C"
