///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "BranchSheathPaths.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "BranchIslandClass.hpp"
#include "libslic3r/Api/plugin/c/slic3r_extrusion_entity.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_toolpath.h"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/ExtrusionProperties.hpp"
#include "libslic3r/Plugins/Support/Common/ClipperResults.hpp"
#include "libslic3r/Plugins/Support/Common/SupportParams.hpp"
#include "libslic3r/Plugins/Support/Common/SupportPaths.hpp"

/* Tree branch sheaths: the organic base paths of the legacy support, applied to the branch surfaces of each support
   layer. A branch surface is the cross-section of one tree branch, marked RAW_SURFACE_TYPE_MOD_BRANCH.

   A sheath walls one contour or hole of a surface with a loop of extrusion, centred half a width inside the outline.
   Each loop starts with an anchor: a short run on the second perimeter, one width further inside, that joins the loop
   at its seam, so the nozzle starts away from the outline. The loop is clipped by 0.15 spacing at the seam so it does
   not overlap its own start. Sheaths are neither sortable nor reversible.

   A surface larger than the circle of support_tree_branch_diameter_double_wall gets two walls: the perimeter of its
   outline and the sheath of the surface shrunk by one width. The inner sheath prints first, because the legacy
   reverses its group, and the group keeps that order.

   The anchor candidates are the surface shrunk by one width. The legacy tags each candidate point with the contour
   it came from through a Clipper that stores a Z value. The plugin API has no such Clipper, so each contour is
   offset on its own, as the legacy does before subtracting the holes (the outline shrinks, each hole grows), and a
   candidate point belongs to the contour whose own offset ring it lies on. A point on two such rings lies where two
   offsets cross, and is tagged -1: the anchor search never follows it.

   A branch gets thinner towards its tip, so a sheath is only one of the methods. The surfaces of a region island are
   fused (closed by one epsilon) into islands, and each island gets one method chosen by classify_island() from the
   share of its area narrower than 1.5 extrusion widths and from its compactness (BranchIslandClass.hpp):
   - Sheath: the sheaths above, from the pieces of the island that survive the closing. Their thin leftovers stay unfilled.
   - Centreline: no sheath. The medial axis of the island prints as one variable-width line, as wide as the island where it
     is at most 1.5 widths wide. A part narrower than the nozzle is widened to the nozzle when the widening is small and
     within the support XY distance (centreline_widths()); a narrower part is trimmed. An island with no line left (a thin
     elongated island narrower than the trim width, such as the crescent a tip leaves on a slope) falls back to the Blob
     below, so that the tip does not end without material at its contact.
   - Blob: a thin island that is not elongated, or a Centreline island with no line. One stationary deposit at its contour
     centroid (the closest boundary point when the centroid lies outside the island) prints the island's net area times layer
     height, before the extrusion multiplier, with the sheath's support flow attributes.
   A Centreline or Blob entity takes the place of the first sheath piece that its island would have printed, so the order of the
   sheaths of the other islands does not change; an island whose pieces all vanished appends its entity after them.

   On the first support layer the bottom of the layer is the bed, and the surfaces become the base flange of
   LegacySupportPaths: first-layer flow, rectilinear lines inside a perimeter, and the base angle turned by a quarter
   turn. The paths of each region island go to the support bucket of its base-tool region island. */

namespace slic3r_api::Support::Toolpath {
namespace {
using Points = std::vector<c_point>;
using PointLists = std::vector<Points>;

// A point or a vector in scaled units, for the anchor walk.
struct Vec {
    double x = 0., y = 0.;
};
Vec operator+(Vec a, Vec b) { return Vec{a.x + b.x, a.y + b.y}; }
Vec operator-(Vec a, Vec b) { return Vec{a.x - b.x, a.y - b.y}; }
Vec operator*(Vec v, double factor) { return Vec{v.x * factor, v.y * factor}; }
double norm(Vec v) { return std::sqrt(v.x * v.x + v.y * v.y); }
Vec as_vec(c_point point) { return Vec{double(point.x), double(point.y)}; }
// Rounds toward zero, as the legacy cast of an anchor point to coord_t does.
c_point to_point(Vec v) { return c_point{coord_t(v.x), coord_t(v.y)}; }

using BranchParams = Common::ToolpathParams;

// An anchor candidate ring: its points, and for each point the contour it came from, or -1 at a crossing of contours.
struct CandidateRing {
    Points points;
    std::vector<int32_t> sources;
};

// The candidate point nearest to a seam: its ring and segment, the position t along the segment, and the squared
// distance to the seam. The default distance is the largest, so a missing foot never qualifies for an anchor.
struct SeamFoot {
    size_t ring = 0, segment = 0;
    double t = 0., x = 0., y = 0., distance2 = std::numeric_limits<double>::max();
};

// Generate the paths of each object island of one support layer and publish them into the island's base-tool region.
void process_layer(const plugin_run_context *context, const run_ctx_support_toolpath &payload, storage_handle *storage, const Object &object, const BranchParams &params, layer_handle *layer);
// Append the paths of one region island's branch surfaces to out: the first-layer flange, or the paths of its fused islands.
void append_region_paths(const plugin_run_context *context, const run_ctx_support_toolpath &payload, storage_handle *storage, const Object &object, const BranchParams &params,
                         const layer_handle *layer, const layer_region_island_handle *region, MutableExtrusionEntity out);
// The branch surfaces of one region island, with the support and branch bits set, as one collection.
StoredExPolygonCollection branch_surfaces(storage_handle *storage, const layer_region_island_handle *region);
// The first-layer flange of the surfaces: the base flange recipe of LegacySupportPaths over their closing, appended to out.
void append_flange(const plugin_run_context *context, const run_ctx_support_toolpath &payload, storage_handle *storage, const layer_handle *layer,
                   const StoredExPolygonCollection &surfaces, const BranchParams &params, MutableExtrusionEntity out);
// Append island methods in closed-piece order, replacing the first thin-island piece or appending when no piece survives.
void append_island_paths(storage_handle *storage, const StoredExPolygonCollection &fused, const StoredExPolygonCollection &closed,
                         const BranchParams &params, const c_flow &flow, MutableExtrusionEntity out);
// The medial axis of a thin elongated island as one entity of variable-width lines, appended to out; returns whether it appended one.
// A thin elongated island narrower than the trim width (the crescent a tip leaves on a slope) yields no line. xy_distance is the
// support XY distance that bounds the widening of the lines narrower than the nozzle.
bool append_centreline(storage_handle *storage, const ExPolygon &island, const c_flow &flow, coord_t xy_distance, MutableExtrusionEntity out);
// The paths of one closed surface: its sheaths, or the double-wall group when the surface is larger than the threshold.
void append_surface_paths(storage_handle *storage, const ExPolygon &surface, const BranchParams &params, const c_flow &flow,
                          MutableExtrusionEntity out);
// The outer perimeter of a surface: one loop per contour and hole, each clipped at its end like the sheaths.
PointLists perimeter_lines(storage_handle *storage, const ExPolygon &surface, double clip_length, coord_t resolution);
// The sheaths of a surface: one per contour, then one per hole, each extended by its anchor when one is found.
PointLists sheath_lines(storage_handle *storage, const ExPolygon &surface, double width, double spacing, coord_t resolution);
// The surface shrunk by one width, each ring clockwise like the sheaths, with the source contour of each point.
std::vector<CandidateRing> anchor_candidates(storage_handle *storage, const ExPolygon &surface, double width);
// The offset ring of each source contour on its own, with the parameters of the combined shrink: the contour shrinks, each hole grows.
std::vector<std::vector<Points>> source_offset_rings(storage_handle *storage, const ExPolygon &surface, double width);
// The points of each ring of a Clipper result.
std::vector<Points> ring_points(const StoredPolygonCollection &rings);
// The contour whose own offset ring the point lies on, or -1 when no contour or two contours hold it.
int32_t source_contour(c_point point, const std::vector<std::vector<Points>> &offsets);
// The distance from a point to a closed ring, in scaled units.
double ring_distance(c_point point, const Points &ring);
// The distance from a point to the segment from a to b, in scaled units.
double segment_distance(c_point point, c_point a, c_point b);
// The point of the candidate rings nearest to the seam, among the segments that touch the sheath of the given contour.
SeamFoot closest_candidate(const std::vector<CandidateRing> &candidates, c_point seam, int32_t loop);
// Extend the path, which ends at its seam, with the anchor found at foot: the path turns to the closest candidate point and
// then walks the candidate ring back for anchor_length, so the printed path starts on the anchor.
void extend_with_anchor(const CandidateRing &ring, const SeamFoot &foot, double anchor_length, int32_t loop, Points &path);
// The points of a polyline cut by distance at its end, through the host polyline clipping.
Points clip_end(storage_handle *storage, const Points &points, double distance);
// Publish the paths of one object island into the support bucket of its base-tool region island.
void publish_paths(const plugin_run_context *context, layer_handle *layer, uint32_t island_index, MutableExtrusionEntity paths, int32_t tool);

// Borrow inputs and context for synchronous generation, moving temporary paths into layer-owned regions. Stopping
// propagates silent PluginCancelled (the cause has its message, the step names it, or the runner rethrows the print
// cancellation); ordinary refusals throw std::runtime_error.
void process_layer(const plugin_run_context *context, const run_ctx_support_toolpath &payload, storage_handle *storage, const Object &object, const BranchParams &params, layer_handle *layer)
{
    for (uint32_t island_index = 0; island_index < layer_count_island(layer); ++island_index) {
        const layer_island_handle *island = layer_get_island(layer, island_index);
        StoredExtrusionEntity paths(storage);
        for (uint32_t region_index = 0; region_index < layer_island_count_region_island(island); ++region_index)
            append_region_paths(context, payload, storage, object, params, layer, layer_island_get_region_island(island, region_index), paths.mutable_view());
        publish_paths(context, layer, island_index, paths.mutable_view(), params.geometry.base_tool);
    }
}

// Borrow the region and run inputs, appending storage-owned branch paths to out; empty surfaces add nothing.
// Stopping propagates silent PluginCancelled (the cause has its message, the step names it, or the runner rethrows the
// print cancellation); ordinary pattern refusals throw std::runtime_error.
void append_region_paths(const plugin_run_context *context, const run_ctx_support_toolpath &payload, storage_handle *storage, const Object &object, const BranchParams &params,
                         const layer_handle *layer, const layer_region_island_handle *region, MutableExtrusionEntity out)
{
    const StoredExPolygonCollection surfaces = branch_surfaces(storage, region);
    if (surfaces.empty()) return;
    const coord_t z = layer_get_print_z(layer), height = layer_get_height(layer);
    // A layer whose bottom is at the bed prints the base flange instead of the sheaths, as the legacy first layer does.
    if (z - height <= 0) {
        append_flange(context, payload, storage, layer, surfaces, params, out);
        return;
    }
    const c_flow flow = object.support_flow(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL, params.geometry.layer_height, unscaled(height));
    // The legacy closing of the branch surfaces: a dilation by one epsilon, then a shrink by the epsilon and half a width.
    const ClipperContext clipper(storage);
    StoredExPolygonCollection closed = clipper_offset2(clipper(surfaces), double(SCALED_EPSILON),
                                                       -(double(SCALED_EPSILON) + 0.5 * double(flow.width))).to_expolygon_collection();
    closed.ensure_valid(params.geometry.resolution);
    // The fused islands are the surfaces closed by the epsilon only, as the flange fuses them: each gets one method.
    StoredExPolygonCollection fused = clipper_offset2(clipper(surfaces), double(SCALED_EPSILON), -double(SCALED_EPSILON)).to_expolygon_collection();
    fused.ensure_valid(params.geometry.resolution);
    append_island_paths(storage, fused, closed, params, flow, out);
}

StoredExPolygonCollection branch_surfaces(storage_handle *storage, const layer_region_island_handle *region)
{
    StoredExPolygonCollection result(storage);
    const raw_surface_type mask = RAW_SURFACE_TYPE_MOD_SUPPORT | RAW_SURFACE_TYPE_MOD_BRANCH | RAW_SURFACE_TYPE_POS_INTERNAL | RAW_SURFACE_TYPE_DENS_SPARSE;
    for (uint32_t index = 0; index < layer_region_island_count_fill_surface(region); ++index) {
        const surface_handle *surface = layer_region_island_get_fill_surface(region, index);
        if ((surface_get_type(surface) & mask) == mask)
            result.push_back(ExPolygon(surface_get_expolygon(surface)));
    }
    return result;
}

// Close borrowed surfaces and append their perimeter-wrapped fill to out; storage owns temporary geometry and the
// borrowed context outlives the fill. Stopping propagates silent PluginCancelled (the cause has its message, the step
// names it, or the runner rethrows the print cancellation); ordinary pattern refusals throw std::runtime_error.
void append_flange(const plugin_run_context *context, const run_ctx_support_toolpath &payload, storage_handle *storage, const layer_handle *layer,
                   const StoredExPolygonCollection &surfaces, const BranchParams &params, MutableExtrusionEntity out)
{
    const ClipperContext clipper(storage);
    StoredExPolygonCollection fused = clipper_offset2(clipper(surfaces), double(SCALED_EPSILON), -double(SCALED_EPSILON)).to_expolygon_collection();
    fused.ensure_valid(params.geometry.resolution);
    Common::append_base_flange(context, payload, storage, out, fused, layer, params.geometry);
}

// Classify valid disjoint fused islands using flow width, then move owned entities into out in closed-piece order. A piece
// outside every island throws; thin islands print once at their first piece, or after the loop in fused-island order.
void append_island_paths(storage_handle *storage, const StoredExPolygonCollection &fused, const StoredExPolygonCollection &closed,
                         const BranchParams &params, const c_flow &flow, MutableExtrusionEntity out)
{
    std::vector<BranchIslandClass> kinds;
    for (const ExPolygon island : fused)
        kinds.push_back(classify_island(storage, island, flow.width).kind);
    // A closed piece belongs to the fused island that holds the first point of its contour. The piece lies at least half a
    // width inside that island and the fused islands are disjoint, so exactly one island holds the point: finding none is a
    // broken invariant.
    std::vector<size_t> owners;
    for (uint32_t piece = 0; piece < closed.size(); ++piece) {
        const c_point first = closed.at(piece).contour().front();
        size_t owner = 0;
        while (owner < fused.size() && !fused.at(uint32_t(owner)).contains(first))
            ++owner;
        if (owner == fused.size())
            throw std::logic_error("A closed branch piece lies in no fused island");
        owners.push_back(owner);
    }

    // The pieces of a Sheath island print as they are, with no check that the island has one, because it always does. Its thin
    // part is under half of its area, so the opening of the island by r = 0.75 w is not empty, and nor is the erosion of the
    // island by r that the opening grows back. The island is D eroded by one epsilon, D being the dilation of the branch
    // surfaces by one epsilon, and a closed piece is D eroded by epsilon + w / 2. Erosions of the same D nest: the larger the
    // distance, the smaller the result. The erosion of the island by r is D eroded by epsilon + r, which lies inside D eroded by
    // epsilon + w / 2 because r is larger than w / 2. The equality is approximate: two composed offsets differ from one offset
    // by the rounding of the Clipper grid and by the corners, which the miter join (limit 3) does not follow exactly. The gap
    // between the two distances, w / 4 (about 0.1 mm), absorbs that difference.
    // Both thin methods replace the first piece; the other pieces of that island are skipped.
    std::vector<bool> drawn(fused.size(), false);
    for (uint32_t piece = 0; piece < closed.size(); ++piece) {
        const size_t island = owners[piece];
        if (kinds[island] == BranchIslandClass::Sheath) {
            append_surface_paths(storage, closed.at(piece), params, flow, out);
        } else if (!drawn[island]) {
            drawn[island] = true;
            if (kinds[island] == BranchIslandClass::Blob || !append_centreline(storage, fused.at(uint32_t(island)), flow, params.geometry.gap_xy, out))
                Common::append_deposit(storage, out, fused.at(uint32_t(island)), flow);
        }
    }
    for (uint32_t island = 0; island < fused.size(); ++island)
        if (kinds[island] != BranchIslandClass::Sheath && !drawn[island]) {
            if (kinds[island] == BranchIslandClass::Blob || !append_centreline(storage, fused.at(island), flow, params.geometry.gap_xy, out))
                Common::append_deposit(storage, out, fused.at(island), flow);
        }
}

bool append_centreline(storage_handle *storage, const ExPolygon &island, const c_flow &flow, coord_t xy_distance, MutableExtrusionEntity out)
{
    // The helper trims the parts of the island narrower than the nozzle by more than the widening limit and widens the others
    // to the nozzle (centreline_widths()). It is told no extrusion maximum and no endpoint extension. The line is extended to
    // the contour before the trim, so the trim shortens it where the island narrows; at a square end the extension copies the
    // last width to the contour (MedialAxis::extends_line()), and the bead overshoots the island by about w / 2.
    const CentrelineWidths widths = centreline_widths(flow, xy_distance);
    std::optional<StoredExtrusionEntity> centreline = medial_axis_extrusion(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL, flow)
        .medial_widths(widths.min_medial_width, widths.max_medial_width).extrusion_widths(widths.min_extrusion_width, 0).can_reverse()
        .try_build(storage, island);
    if (!centreline.has_value())
        return false;
    out.append_child_move(centreline->mutable_view());
    return true;
}
// Builds sheaths from a borrowed closed surface and flow, moving storage-owned paths or a fixed inner-first double-wall group into out; area and threshold use squared scaled units, and geometry/allocation failures propagate.
void append_surface_paths(storage_handle *storage, const ExPolygon &surface, const BranchParams &params, const c_flow &flow,
                          MutableExtrusionEntity out)
{
    const double width = double(flow.width), spacing = double(flow.spacing);
    const coord_t resolution = params.geometry.resolution;
    const EPropertyAttributes attributes = Common::path_attributes(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL, flow);
    if (params.double_wall_area <= 0. || surface.area() <= params.double_wall_area) {
        Common::append_paths(storage, out, sheath_lines(storage, surface, width, spacing, resolution), attributes, false);
        return;
    }
    // The inner area is the surface shrunk by one width; its sheath is printed first, as the legacy group is reversed.
    const ClipperContext clipper(storage);
    StoredExPolygonCollection inner = clipper_offset2(clipper(surface), -1.5 * width, 0.5 * width).to_expolygon_collection();
    inner.ensure_valid(resolution);
    const bool two_walls = inner.size() == 1;
    PointLists walls = two_walls ? perimeter_lines(storage, surface, spacing * 0.15, resolution) : PointLists{};
    const PointLists sheaths = sheath_lines(storage, two_walls ? inner.at(0) : surface, width, spacing, resolution);
    walls.insert(walls.end(), sheaths.begin(), sheaths.end());
    std::reverse(walls.begin(), walls.end());
    StoredExtrusionEntity group(storage);
    Common::append_paths(storage, group.mutable_view(), walls, attributes, false);
    // The flags come after the children, since adding the first child resets them to the sortable default.
    group.set_flags(0u);
    out.append_child_move(group.mutable_view());
}

// Copies ordered rings from a borrowed surface, reverses holes to counter-clockwise, closes and clips their ends, returning owned validated lines; clip_length and resolution are scaled and geometry/allocation failures propagate.
PointLists perimeter_lines(storage_handle *storage, const ExPolygon &surface, double clip_length, coord_t resolution)
{
    PointLists lines;
    std::vector<Points> rings;
    for (const Polygon ring : Common::polygon_rings(surface))
        rings.push_back(ring.points());
    for (size_t index = 0; index < rings.size(); ++index) {
        Points points = rings[index];
        points.push_back(points.front());
        // Holes are stored clockwise: they are reversed so that every perimeter runs counter-clockwise, as in the legacy.
        if (index > 0) std::reverse(points.begin(), points.end());
        lines.push_back(clip_end(storage, points, clip_length));
    }
    return Common::valid_polylines(storage, lines, resolution);
}

// Copies ordered rings from a borrowed surface, clips clockwise seams and extends matching anchors before reversal, returning owned validated lines; dimensions are scaled and geometry/allocation failures propagate.
PointLists sheath_lines(storage_handle *storage, const ExPolygon &surface, double width, double spacing, coord_t resolution)
{
    std::vector<Points> rings;
    for (const Polygon ring : Common::polygon_rings(surface))
        rings.push_back(ring.points());
    const std::vector<CandidateRing> candidates = anchor_candidates(storage, surface, width);
    const double clip_length = spacing * 0.15, anchor_length = spacing * 6.;
    PointLists lines;
    for (int32_t loop = 0; loop < int32_t(rings.size()); ++loop) {
        Points points = rings[size_t(loop)];
        // The contour is walked clockwise, as the holes already are, so the anchor continues the walk at the seam.
        if (loop == 0) std::reverse(points.begin(), points.end());
        points.push_back(points.front());
        Points path = clip_end(storage, points, clip_length);
        if (path.size() < 2) continue;
        const SeamFoot foot = closest_candidate(candidates, path.back(), loop);
        if (foot.distance2 < (3. * width) * (3. * width))
            extend_with_anchor(candidates[foot.ring], foot, anchor_length, loop, path);
        std::reverse(path.begin(), path.end());
        lines.push_back(path);
    }
    return Common::valid_polylines(storage, lines, resolution);
}

std::vector<CandidateRing> anchor_candidates(storage_handle *storage, const ExPolygon &surface, double width)
{
    const ClipperContext clipper(storage);
    // The legacy second perimeter is a miter offset with a miter limit of 1.2.
    const StoredPolygonCollection shrunk = clipper_offset(clipper(surface), -width, CLIPPER_JOIN_MITER, 1.2).to_polygon_collection();
    const std::vector<std::vector<Points>> offsets = source_offset_rings(storage, surface, width);
    std::vector<CandidateRing> candidates;
    for (const Polygon polygon : shrunk) {
        CandidateRing candidate;
        candidate.points = polygon.points();
        if (!is_clockwise(candidate.points))
            std::reverse(candidate.points.begin(), candidate.points.end());
        for (const c_point point : candidate.points)
            candidate.sources.push_back(source_contour(point, offsets));
        candidates.push_back(std::move(candidate));
    }
    return candidates;
}

// The legacy Clipper_Z offsets each contour before it subtracts the holes from the outline: the outline shrinks and each
// hole grows, with the same miter join and limit as the combined shrink.
std::vector<std::vector<Points>> source_offset_rings(storage_handle *storage, const ExPolygon &surface, double width)
{
    const ClipperContext clipper(storage);
    std::vector<std::vector<Points>> offsets;
    offsets.push_back(ring_points(clipper_offset(clipper(surface.contour()), -width, CLIPPER_JOIN_MITER, 1.2).to_polygon_collection()));
    for (const Polygon hole : surface.holes())
        offsets.push_back(ring_points(clipper_offset(clipper(hole), width, CLIPPER_JOIN_MITER, 1.2).to_polygon_collection()));
    return offsets;
}

std::vector<Points> ring_points(const StoredPolygonCollection &rings)
{
    std::vector<Points> points;
    for (const Polygon ring : rings)
        points.push_back(ring.points());
    return points;
}

// A point belongs to a contour when it lies on that contour's own offset ring, within a few scaled epsilons that absorb
// the rounding of Clipper's integer grid. The nearest contour is not used: at a miter corner of a hole, a point of its
// offset can lie nearer to the outline than to the hole. A point on two rings lies where two offsets cross; a point on
// none is not attributed to a contour. Both are tagged -1, which the anchor search never follows.
int32_t source_contour(c_point point, const std::vector<std::vector<Points>> &offsets)
{
    const double tolerance = 4. * SCALED_EPSILON;
    int32_t source = -1;
    size_t hits = 0;
    for (size_t index = 0; index < offsets.size(); ++index)
        for (const Points &ring : offsets[index])
            if (ring_distance(point, ring) <= tolerance) {
                source = int32_t(index);
                ++hits;
                break;
            }
    return hits == 1 ? source : -1;
}

double ring_distance(c_point point, const Points &ring)
{
    double nearest = std::numeric_limits<double>::max();
    for (size_t index = 0; index < ring.size(); ++index)
        nearest = std::min(nearest, segment_distance(point, ring[index], ring[(index + 1) % ring.size()]));
    return nearest;
}

double segment_distance(c_point point, c_point a, c_point b)
{
    const Vec start = as_vec(a), direction = as_vec(b) - start, offset = as_vec(point) - start;
    const double length2 = direction.x * direction.x + direction.y * direction.y;
    const double t = length2 == 0. ? 0. : std::clamp((direction.x * offset.x + direction.y * offset.y) / length2, 0., 1.);
    const Vec closest = start + direction * t;
    return norm(as_vec(point) - closest);
}

SeamFoot closest_candidate(const std::vector<CandidateRing> &candidates, c_point seam, int32_t loop)
{
    SeamFoot best;
    for (size_t ring_index = 0; ring_index < candidates.size(); ++ring_index) {
        const CandidateRing &ring = candidates[ring_index];
        for (size_t i = 0; i < ring.points.size(); ++i) {
            const size_t j = (i + 1) % ring.points.size();
            const bool i_loop = ring.sources[i] == loop, j_loop = ring.sources[j] == loop;
            if (!i_loop && !j_loop)
                continue;
            const Vec start = as_vec(ring.points[i]), direction = as_vec(ring.points[j]) - start, offset = as_vec(seam) - start;
            const double length2 = direction.x * direction.x + direction.y * direction.y;
            const double t = length2 == 0. ? 0. : std::clamp((direction.x * offset.x + direction.y * offset.y) / length2, 0., 1.);
            if (!(i_loop || t > EPSILON) || !(j_loop || t < 1. - EPSILON))
                continue;
            const Vec foot = start + direction * t;
            const Vec from_seam = foot - as_vec(seam);
            const double distance2 = from_seam.x * from_seam.x + from_seam.y * from_seam.y;
            if (distance2 < best.distance2)
                best = SeamFoot{ring_index, i, t, foot.x, foot.y, distance2};
        }
    }
    return best;
}

void extend_with_anchor(const CandidateRing &ring, const SeamFoot &foot, double anchor_length, int32_t loop, Points &path)
{
    const Points &points = ring.points;
    const Vec closest{foot.x, foot.y};
    double remaining = anchor_length - norm(closest - as_vec(path.back()));
    path.push_back(to_point(closest));
    size_t i = foot.segment, j = (i + 1) % points.size();
    Vec pi = as_vec(points[i]), pj = as_vec(points[j]);
    Vec v = pj - pi;
    double l = norm(v);
    if (remaining < (1. - foot.t) * l) {
        path.push_back(to_point(closest + v * (remaining / l)));
        return;
    }
    // Skip the candidate point when it repeats the closest point within the legacy anchor resolution.
    const c_point last = path.back();
    if (std::abs(last.x - points[j].x) >= SCALED_EPSILON / 2 || std::abs(last.y - points[j].y) >= SCALED_EPSILON / 2)
        path.push_back(points[j]);
    pi = pj;
    for (i = j; ring.sources[i] == loop && remaining > 0.; i = j, pi = pj) {
        j = (i + 1) % points.size();
        pj = as_vec(points[j]);
        v = pj - pi;
        l = norm(v);
        // The walk came back to the segment of the closest point: the candidate ring is shorter than the anchor.
        if (i == foot.segment)
            break;
        if (remaining <= l) {
            path.push_back(to_point(pi + v * (remaining / l)));
            break;
        }
        path.push_back(points[j]);
        remaining -= l;
    }
}

Points clip_end(storage_handle *storage, const Points &points, double distance)
{
    StoredPolyline line(storage);
    line.insert_array(0, points.data(), uint32_t(points.size()));
    line.clip_end(distance);
    return line.points();
}

// Move children from borrowed paths to the layer-owned support bucket; empty paths add nothing. A refused destination
// exits through silent PluginCancelled when stopping (the cause has its message, the step names it, or the runner
// rethrows the print cancellation), otherwise
// std::runtime_error; insertion failures throw std::runtime_error.
void publish_paths(const plugin_run_context *context, layer_handle *layer, uint32_t island_index, MutableExtrusionEntity paths, int32_t tool)
{
    if (paths.readonly().child_count() == 0)
        return;
    layer_island_handle *island = layer_get_island_mutable(layer, island_index);
    layer_region_island_handle *region = layer_island_get_or_create_region_island(island, nullptr, 0, tool > 0 ? tool - 1 : EXTRUDER_ID_UNKNOWN);
    extrusion_entity_handle *destination = layer_region_island_get_mutable_extrusion(region, RAW_EXTRUSION_ROLE_SUPPORT);
    if (!destination) {
        throw_if_cancelled(context);
        throw std::runtime_error("Cannot access support extrusion destination");
    }
    for (uint32_t index = 0; index < paths.readonly().child_count(); ++index) {
        MutableExtrusionEntity entity = paths.child_mutable(index);
        if (extrusion_insert_child_move(destination, extrusion_child_count(destination), entity.mutable_handle()) == EXTRUSION_INDEX_INVALID)
            throw std::runtime_error("Cannot publish support extrusion");
    }
}

} // namespace

// Walk the callback object's support layers, publishing the first-layer flange and then classified island sheaths,
// centrelines or deposits into their base-tool support buckets. Per-call parameters isolate concurrent objects;
// missing payload, cancellation, geometry, pattern-service and publication errors propagate to the caller.
// The borrowed context outlives all synchronous fills; stopping propagates silent PluginCancelled (the cause has its
// message, the step names it, or the runner rethrows the print cancellation), while ordinary refusals propagate std::runtime_error.
void generate_branch_paths(const plugin_run_context *context)
{
    Common::for_support_layers(context, true,
        // Borrows matching layer/run inputs synchronously and publishes branch methods; generation failures propagate.
        [context](const run_ctx_support_toolpath &payload, const Object &object, const Common::ToolpathParams &params, layer_handle *layer) {
            process_layer(context, payload, context->plugin_storage, object, params, layer);
        });
}

} // namespace slic3r_api::Support::Toolpath
