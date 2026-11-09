#include "PerimeterCrossing.hpp"
#include "libslic3r/Plugins/Common/ClipperHelpers.hpp"

/*
PerimeterCrossing answers path-crossing queries for travel and retraction
decisions. It tests prepared area boundaries, not extrusion centerlines. It neither
routes a path nor proves containment: a path wholly outside all contours or wholly
inside a hole may return false. All paths use the target's local coordinates.

prepare builds a base version and an offset version of the layer slices. For
objects, when any region assigns a perimeter or infill role to another tool,
preparation clips the slices to the selected tool's area so that tool does not
ooze its material over another tool's area. It then insets the offset version by
1.5 nozzle radii. Tops are removed from both versions. For support the base is the support
layer's own slices; the offset version expands them by 1.5 radii, without tool
clipping or top removal. Already printed object layers are not included in that
support test. Both versions are simplified using the scaled nozzle radius.

Queries first reject distant islands with contour boxes, compare inclusive
endpoint containment, and test every path segment against the outer contour.
For paths whose endpoints are inside it, hole boxes are built on demand and hole
edges are tested. Large holes use boxes to avoid unnecessary edge scans.

    prepare: reuse or rebuild the two versions for a layer, tool and radius
    |-- object: union slices, clip if any region assigns a role to another tool,
    |           inset, then subtract regional tops
    `-- support: union own slices and union their expansion
        Both branches simplify rings and retain unsimplified contour boxes.
    can_cross_perimeter: choose base or offset version
    |-- reject by contour box, then compare endpoint containment
    |-- first_intersection: test path segments against the outer contour
    `-- build hole boxes lazily, then test path segments against hole edges

The cache key is the LayerInput address, zero-based tool and scaled radius.
The caller keeps that input alive and unchanged until preparation with another
input; owned cached rings do not borrow temporary storage. One worker owns each
instance because queries populate hole boxes. Cancellation may throw: prepare
calls it per object region before top subtraction, queries once per examined hole.
*/
namespace slic3r_api { namespace LayerExtrusionEdit { namespace AvoidCrossing {
namespace pw = PluginCommon::polygon_work;
namespace clipper = PluginCommon::clipper;

void PerimeterCrossing::prepare(storage_handle *storage, const LayerInput &layer,
                                size_t extruder, coord_t radius, std::function<void()> cancel)
{
    m_cancel = cancel ? std::move(cancel) : []() {};
    if (m_cache.layer == &layer && m_cache.extruder == extruder && m_cache.radius == radius)
        return;
    m_cache.layer = &layer;
    m_cache.extruder = extruder;
    m_cache.radius = radius;
    pw::ExPolygons slices = clipper::union_ex(storage, layer.lslices);
    pw::ExPolygons offset_slices;
    if (layer.support) {
        offset_slices = clipper::union_ex(storage, clipper::offset_ex(storage, layer.lslices, radius * 1.5f));
    } else {
        bool multiple_extruders = false;
        for (const RegionInput &region : layer.regions) {
            multiple_extruders = multiple_extruders || region.perimeter_extruder != extruder + 1 ||
                region.infill_extruder != extruder + 1 || region.solid_infill_extruder != extruder + 1;
            if (multiple_extruders)
                break;
        }
        // Restrict this tool to its own material area before testing paths that could ooze over another tool's area.
        if (multiple_extruders) {
            pw::ExPolygons clip;
            for (const RegionInput &region : layer.regions) {
                const bool same = region.perimeter_extruder == region.infill_extruder &&
                    region.infill_extruder == region.solid_infill_extruder;
                if (same) {
                    if (region.perimeter_extruder == extruder + 1)
                        clip = clipper::union_ex(storage, clip, region.raw_slices);
                } else if (region.perimeter_extruder == extruder + 1) {
                    assert(region.infill_extruder != extruder + 1);
                    clip = clipper::union_ex(storage, clip, clipper::diff_ex(storage, region.raw_slices, region.fill_expolygons));
                } else {
                    assert(region.infill_extruder == extruder + 1);
                    clip = clipper::union_ex(storage, clip, region.fill_expolygons);
                }
            }
            slices = clipper::intersection_ex(storage, slices, clipper::offset_ex(storage, clip, SCALED_EPSILON * 10));
        }
        offset_slices = clipper::offset_ex(storage, slices, -radius * 1.5f);
        // Tops are excluded even when no outer contour would be crossed.
        for (const RegionInput &region : layer.regions) {
            m_cancel();
            offset_slices = clipper::diff_ex(storage, offset_slices, region.top_surfaces);
            slices = clipper::diff_ex(storage, slices, region.top_surfaces);
        }
    }
    // Preserve the unsimplified contour box, as used by the crossing rejection.
    m_cache.slices.clear();
    for (const pw::ExPolygon &polygon : slices) {
        const BoundingBox box(polygon.contour.points);
        for (pw::ExPolygon &simplified : clipper::simplify(storage, polygon, radius))
            m_cache.slices.push_back({std::move(simplified), box, {}});
    }
    m_cache.offset_slices.clear();
    for (const pw::ExPolygon &polygon : offset_slices) {
        const BoundingBox box(polygon.contour.points);
        for (pw::ExPolygon &simplified : clipper::simplify(storage, polygon, radius))
            m_cache.offset_slices.push_back({std::move(simplified), box, {}});
    }
}

bool PerimeterCrossing::can_cross_perimeter(const PluginCommon::work::Polyline &travel, bool offset)
{
    assert(m_cache.layer != nullptr);
    std::vector<SliceIsland> &islands = offset ? m_cache.offset_slices : m_cache.slices;
    for (SliceIsland &island : islands) {
        // Reject paths whose sampled points and segments miss the contour box before scanning its edges.
        if (travel.size() <= 1 || !(island.boundingbox.contains(travel.front()) ||
            island.boundingbox.contains(travel.back()) ||
            island.boundingbox.contains(travel.points[travel.size() / 2]) || pw::cross(island.boundingbox, travel)))
            continue;
        const bool front = pw::contains(island.expolygon.contour, travel.front(), true);
        const bool back = pw::contains(island.expolygon.contour, travel.back(), true);
        // Different inclusive containment results already imply a contour crossing.
        if (front != back)
            return true;
        pw::Point intersection;
        // A multi-point path can leave and re-enter even when its endpoints have equal containment.
        for (size_t i = travel.size() - 1; i > 0; --i) {
            const pw::Line line(travel.points[i], travel.points[i - 1]);
            const pw::Points &ring = island.expolygon.contour.points;
            if (pw::first_intersection(ring, line, &intersection) ||
                pw::Line(ring.front(), ring.back()).intersection(line, &intersection))
                return true;
        }
        if (!front)
            continue;
        // Only paths inside the contour need hole intersection tests.
        if (front && back) {
            // Cache hole boxes only when a path inside the outer contour actually needs hole tests.
            if (island.hole_boundingboxes.size() != island.expolygon.holes.size()) {
                island.hole_boundingboxes.clear();
                for (const pw::Polygon &hole : island.expolygon.holes)
                    island.hole_boundingboxes.emplace_back(hole.points);
            }
            for (size_t h = 0; h < island.expolygon.holes.size(); ++h) {
                const pw::Points &hole = island.expolygon.holes[h].points;
                const BoundingBox &box = island.hole_boundingboxes[h];
                m_cancel();
                for (size_t i = travel.size() - 1; i > 0; --i) {
                    const pw::Line line(travel.points[i], travel.points[i - 1]);
                    // A box costs up to four edge tests, so use it only for holes with more than ten vertices.
                    if (hole.size() > 10 && !pw::cross(box, line) && !box.contains(line.a))
                        continue;
                    if (pw::first_intersection(hole, line, &intersection) ||
                        pw::Line(hole.front(), hole.back()).intersection(line, &intersection))
                        return true;
                }
            }
        }
    }
    return false;
}
}}}
