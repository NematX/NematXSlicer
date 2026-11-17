///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "LegacySupportPaths.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <map>
#include <stdexcept>
#include <tuple>
#include <utility>

#include "libslic3r/Api/plugin/c/slic3r_extrusion_entity.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_toolpath.h"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/ExtrusionProperties.hpp"
#include "libslic3r/Plugins/Support/Common/ClipperResults.hpp"
#include "libslic3r/Plugins/Support/Common/ExtrusionLines.hpp"
#include "libslic3r/Plugins/Support/Common/SupportCloseZ.hpp"
#include "libslic3r/Plugins/Support/Common/SupportParams.hpp"
#include "libslic3r/Plugins/Support/Common/SupportCounters.hpp"
#include "libslic3r/Plugins/Support/Common/SupportPaths.hpp"
#include "libslic3r/Plugins/Support/Common/SupportSettings.hpp"

/* Support surfaces describe printable areas, not paths. This file chooses the
   pattern plugin that fills each area and keeps the spacing fixed between layers.
   Contacts are separated from intermediate interfaces by the contact modifier
    that the publication sets on the dense areas of contact layers, even on sides
    without interface layers; unmarked dense surfaces remain interfaces. Contact loops
   leave small circular breaks for removal. Overlapping slabs split paths and
   reduce their flow in proportion to the remaining thickness. Each role prints
   only its own surfaces: with no interface layers a contact that cannot merge
   into the base keeps its own role, while a mergeable contact is published as
   base and printed by the base role instead. The generated paths
   are published into the island that holds them, with the tool of the role.
   Contact providers first return automatically fused components by clearing only
   their contact bit. Interface providers fill those areas when their turn arrives.
   Filling precedes island assignment, so nearby contours can join in the union.
   A synchronous InterfaceFill may replace dense fill above the bed after
   contact handling and recipe selection, but before modulation and publication.
   generate_paths -> process_layer -> hand_back_contacts -> should_fuse_contact
                                  -> fill_layer_pass -> InterfaceFill or ordinary fill
                                                     -> Internal::modulate -> publish_layer_paths

   Every geometric operation goes through the plugin API: Clipper views for the
   booleans and offsets, data-tree views for the layers, the pattern service for
   the fills and extrusion views for the trees. Temporary geometry lives in the
   plugin storage of the run. */

namespace slic3r_api::Support::Toolpath {
namespace {
using Polys = StoredPolygonCollection;
using Points = std::vector<c_point>;
using PointLists = std::vector<Points>;
using PointKey = std::pair<coord_t, coord_t>;
using SplitPoints = std::map<PointKey, int>;

// How far a support layer may lie from the exact position of a contact (under or over an object layer) and still be matched with that object layer.
// The tree's own snapping (align_slab_bounds/merge_close_bounds) and the host pass at the end of STEP_SUPPORT can each move a support z by up to
// the close-Z tolerance, max(z_step / 2, 4 * SCALED_EPSILON) (Common::close_z_tolerance_limit(), the largest value that either pass uses), so the
// margin is the matching epsilon plus both moves, about one Z step. With a layer height of one step the window can hold two object layers (the
// neighbour is one step away from the expected position even when nothing moved), so matched_layer() keeps only the nearest one to the expected
// Z. It applies to every print: a layer that close to the exact contact position is the contact.
coord_t contact_match_tolerance(const Common::SupportParams &geometry)
{
    return SCALED_EPSILON + 2 * Common::close_z_tolerance_limit(geometry.z_step);
}

using PathParams = Common::ToolpathParams;
// One filler pass: a polygon set printed with the same settings.
struct Pass {
    Pass(Polys polygons, bool contact, bool as_base, bool bridge) : polygons(std::move(polygons)), contact(contact), as_base(as_base), bridge(bridge) {}
    Polys polygons;
    bool contact; // prints with the configured contact pattern
    bool as_base; // prints with the base settings: flow, spacing, angle, density and role
    bool bridge;  // prints with the bridging flow
};
// The lines that one slab cut from a path, with the attributes of the path they came from.
struct Fragment {
    EPropertyAttributes attributes;
    PointLists lines;
};

// Convert a Clipper result into a flat polygon collection, like the native polygon booleans.
Polys as_polygons(ClipperOperand &&result);
// Match borrowed type bits to a role; non-support or wrong-side surfaces return false without effects or failures.
bool role_surface(raw_surface_type type, SurfaceRole role);
// Copy matching rings into caller storage, optionally filtering bridge state; inputs stay unchanged and API/allocation failures propagate.
Polys role_polygons(storage_handle *storage, const layer_region_island_handle *region, SurfaceRole role, int bridge = -1);
// Collect one role across the islands of a support layer before any union.
Polys layer_polygons(storage_handle *storage, const layer_handle *layer, SurfaceRole role, int bridge = -1);
// Evaluate fusion on a borrowed component; active loops block pattern-equality fusion, but not narrow-component erosion.
// Positive scaled width defines erosion in storage; layers are unchanged and geometry failures propagate.
bool should_fuse_contact(const Common::SupportParams &geometry, SurfaceRole role, bool bridge, const ExPolygon &area, storage_handle *storage);
// Copy each changed region before clearing contact only; replacements move to the layer and invalidate source handles.
// Rejection checks cancellation then throws runtime_error; allocation and geometry failures propagate without a silent result.
void hand_back_contacts(const plugin_run_context *context, storage_handle *storage, layer_handle *layer, SurfaceRole role, const Common::SupportParams &geometry);
// Whether any of the polylines crosses the area.
bool crosses(storage_handle *storage, const PointLists &lines, const ExPolygonCollection &area);
// Convert point lists to a storage-owned polyline collection, and back.
StoredPolylineCollection polyline_collection(storage_handle *storage, const PointLists &lines);
PointLists point_lists(const PolylineCollection &lines);
// Convert point rings to a storage-owned polygon collection.
Polys polygon_collection(storage_handle *storage, const PointLists &rings);
// Whether a point lies within radius of one of the centers: the native lookup's find() test.
bool near_center(const Points &centers, c_point point, coord_t radius);
// Key of a point in the split map, matching the native Point equality.
PointKey point_key(c_point point);
// The circle breaks that contact loops place around each center, rounded like the native Point constructor.
c_point rounded(double x, double y);
// Squared distance between two points, computed in double like the native squaredNorm().
double squared_distance(c_point a, c_point b);
// Cut removable loops out of a contact part and append their interface paths to out.
void contact_loops(storage_handle *storage, MutableExtrusionEntity out, Polys &contact, const Polys &overhang, const c_flow &flow, coord_t resolution);
// Clips broken contact contours, rejoins split endpoints, publishes loops and removes their footprint from contact.
void publish_contact_loops(storage_handle *storage, const ClipperContext &clipper, MutableExtrusionEntity out, Polys &contact, const Polys &overhang,
                           const c_flow &flow, coord_t resolution, coordf_t radius, const PointLists &contours, const PointLists &circles);
// Split each polyline by the area, returning the pieces inside and the pieces outside.
std::pair<PointLists, PointLists> split_lines(storage_handle *storage, const PointLists &lines, const ExPolygonCollection &area);
// Clip one path from the highest overlap down, scale its flow by the remaining height and rechain the fragments from its start; a path
// that no footprint grown by half its width takes a non-empty piece of (before the length filter) is left untouched.
void modulate_entity(storage_handle *storage, MutableExtrusionEntity entity, coord_t z, const std::vector<Internal::Overlap> &overlapping, coord_t resolution);
// Borrowed marked layers keyed by print Z, with their auxiliary-list position retained for operand order.
using SupportLayerIndex = std::map<coord_t, std::vector<std::pair<uint32_t, const layer_handle *>>>;
// Gather lower footprints strictly between bottom and z into caller storage in ascending Z;
// lower_slabs includes sparse and non-bridge bottom areas, and geometry/allocation failures propagate without changing layers.
std::vector<Internal::Overlap> overlaps(storage_handle *storage, const SupportLayerIndex &support_layers, coord_t z, coord_t bottom, bool lower_slabs);

// Returns the first layer reaching z: bottom=true selects bottom_z, false selects print_z; both sequences must be sorted.
// Callers apply an inclusive tolerance window starting at this index.
uint32_t first_layer_at(const Object &object, coord_t z, bool bottom);
// Return fused contacts then generate this role's paths with borrowed object-local Z indices and storage-owned temporaries.
// Layers own the published paths; cancellation and pattern/assignment failures propagate to PluginBase.
void process_layer(const plugin_run_context *context, const run_ctx_support_toolpath &payload, storage_handle *storage, const Object &object, const PathParams &params,
                   const std::vector<coord_t> &support_z, const SupportLayerIndex &support_layers, SurfaceRole role, layer_handle *layer,
                   const InterfaceFill &interface_fill);
// Fill one role/bridge pass and modulate against lower slabs; pass polygons lose loop footprints, temporaries belong to storage
// until publication moves paths into layers, and pattern/assignment/cancellation failures propagate.
void fill_layer_pass(const plugin_run_context *context, const run_ctx_support_toolpath &payload, storage_handle *storage,
                      const Object &object, const Common::SupportParams &geometry, const SupportLayerIndex &support_layers,
                      SurfaceRole role, layer_handle *layer, coord_t z, coord_t height, coord_t bottom,
                       float base_angle, float angle, const std::string &contact_pattern, Pass &pass, bool lower_slabs,
                       const InterfaceFill &interface_fill);
// Moves generated children to the first intersecting island, using the role's bucket and tool.
void publish_layer_paths(const plugin_run_context *context, storage_handle *storage, layer_handle *layer,
                         StoredExtrusionEntity &out, const Common::SupportParams &geometry, bool as_base);

Polys as_polygons(ClipperOperand &&result)
{
    return result.to_polygon_collection();
}

bool role_surface(raw_surface_type type, SurfaceRole role)
{
    const bool top = role == SurfaceRole::TopInterface || role == SurfaceRole::TopContact;
    const bool contact = role == SurfaceRole::TopContact || role == SurfaceRole::BottomContact;
    const raw_surface_type mask = RAW_SURFACE_TYPE_MOD_SUPPORT | (role == SurfaceRole::Sparse ?
        RAW_SURFACE_TYPE_POS_INTERNAL | RAW_SURFACE_TYPE_DENS_SPARSE : top ?
        RAW_SURFACE_TYPE_POS_TOP | RAW_SURFACE_TYPE_DENS_SOLID : RAW_SURFACE_TYPE_POS_BOTTOM | RAW_SURFACE_TYPE_DENS_SOLID);
    if ((type & mask) != mask) return false;
    return role == SurfaceRole::Sparse || surface_type_is_contact(type) == contact;
}

Polys role_polygons(storage_handle *storage, const layer_region_island_handle *region, SurfaceRole role, int bridge)
{
    Polys result(storage);
    for (uint32_t index = 0; index < layer_region_island_count_fill_surface(region); ++index) {
        const surface_handle *surface = layer_region_island_get_fill_surface(region, index);
        const raw_surface_type type = surface_get_type(surface);
        if (!role_surface(type, role) || (bridge >= 0 && bool(type & RAW_SURFACE_TYPE_MOD_BRIDGE) != bool(bridge))) continue;
        Common::append_polygons(result, ExPolygon(surface_get_expolygon(surface)));
    }
    return result;
}

Polys layer_polygons(storage_handle *storage, const layer_handle *layer, SurfaceRole role, int bridge)
{
    Polys result(storage);
    for (uint32_t island_index = 0; island_index < layer_count_island(layer); ++island_index) {
        const layer_island_handle *island = layer_get_island(layer, island_index);
        for (uint32_t region_index = 0; region_index < layer_island_count_region_island(island); ++region_index)
            result.append_move_from(role_polygons(storage, layer_island_get_region_island(island, region_index), role, bridge));
    }
    return result;
}

bool should_fuse_contact(const Common::SupportParams &geometry, SurfaceRole role, bool bridge, const ExPolygon &area, storage_handle *storage)
{
    const bool top = role == SurfaceRole::TopContact;
    const bool as_base = top ? !geometry.top_published_interfaces_enabled : !geometry.bottom_published_interfaces_enabled;
    const std::string &configured = top ? geometry.top_pattern_id : geometry.bottom_pattern_id;
    const std::string contact_pattern = Internal::surface_pattern(as_base, true, bridge, false, configured, geometry.base_pattern_id);
    const std::string interface_pattern = Internal::surface_pattern(as_base, false, bridge, false, configured, geometry.base_pattern_id);
    const bool loops = top && geometry.top_published_interfaces_enabled && geometry.interface_contact_loops;
    const bool specific_active = contact_pattern != interface_pattern || loops || (!top && bridge);
    if (!specific_active || (contact_pattern == interface_pattern && !loops)) return true;
    const ClipperContext clipper(storage);
    return clipper_offset(clipper(area), -double(geometry.interface_flow.width) / 2.).to_expolygon_collection().empty();
}

void hand_back_contacts(const plugin_run_context *context, storage_handle *storage, layer_handle *layer, SurfaceRole role, const Common::SupportParams &geometry)
{
    for (uint32_t island_index = 0; island_index < layer_count_island(layer); ++island_index) {
        layer_island_handle *island = layer_get_island_mutable(layer, island_index);
        for (uint32_t region_index = 0; region_index < layer_island_count_region_island(island); ++region_index) {
            layer_region_island_handle *region = layer_island_get_region_island_mutable(island, region_index);
            const uint32_t count = layer_region_island_count_fill_surface(region);
            std::vector<bool> handed_back(count, false);
            bool any = false;
            for (uint32_t index = 0; index < count; ++index) {
                const Surface source(layer_region_island_get_fill_surface(region, index));
                if (!role_surface(source.type(), role)) continue;
                const bool bridge = (source.type() & RAW_SURFACE_TYPE_MOD_BRIDGE) != 0;
                handed_back[index] = should_fuse_contact(geometry, role, bridge, source.expolygon(), storage);
                any = any || handed_back[index];
            }
            if (!any) continue;
            StoredSurfaceCollection surfaces(storage);
            for (uint32_t index = 0; index < count; ++index) {
                const Surface source(layer_region_island_get_fill_surface(region, index));
                surfaces.append(source.expolygon(), handed_back[index] ?
                    surface_type_remove_flags(source.type(), RAW_SURFACE_TYPE_MOD_CONTACT) : source.type());
                surfaces.back_mutable().copy_properties_from(source);
            }
            if (!LayerRegionIsland(region).set_fill_surfaces(&surfaces)) {
                throw_if_cancelled(context);
                throw std::runtime_error("Cannot hand support contacts back to the interface role");
            }
        }
    }
}

// Binary-searches borrowed object layers: bottom=true selects bottom_z, false selects print_z, both assumed non-decreasing.
// Returns the first index whose key is >= z, or layer_count if absent; callers apply the inclusive tolerance window.
// Owns no geometry, changes no layers and propagates view-access failures.
uint32_t first_layer_at(const Object &object, coord_t z, bool bottom)
{
    uint32_t first = 0, last = object.layer_count();
    while (first < last) {
        const uint32_t middle = first + (last - first) / 2;
        const Layer layer = object.layer(middle);
        if ((bottom ? layer.bottom_z() : layer.print_z()) < z) first = middle + 1;
        else last = middle;
    }
    return first;
}

bool crosses(storage_handle *storage, const PointLists &lines, const ExPolygonCollection &area)
{
    for (const Points &points : lines) {
        StoredPolyline line(storage);
        line.insert_array(0, points.data(), uint32_t(points.size()));
        if (!clipper_intersection_polyline_expolygons(storage, line, area).empty())
            return true;
    }
    return false;
}

StoredPolylineCollection polyline_collection(storage_handle *storage, const PointLists &lines)
{
    StoredPolylineCollection result(storage);
    for (const Points &points : lines) {
        StoredPolyline line(storage);
        line.insert_array(0, points.data(), uint32_t(points.size()));
        result.push_back(line);
    }
    return result;
}

PointLists point_lists(const PolylineCollection &lines)
{
    PointLists result;
    for (const Polyline line : lines)
        result.push_back(line.points());
    return result;
}

Polys polygon_collection(storage_handle *storage, const PointLists &rings)
{
    Polys result(storage);
    for (const Points &points : rings) {
        StoredPolygon polygon(storage);
        polygon.insert_array(0, points.data(), uint32_t(points.size()));
        result.push_back(polygon);
    }
    return result;
}

bool near_center(const Points &centers, c_point point, coord_t radius)
{
    for (const c_point &center : centers)
        if (squared_distance(point, center) < double(radius) * double(radius))
            return true;
    return false;
}

PointKey point_key(c_point point)
{
    return PointKey{point.x, point.y};
}

c_point rounded(double x, double y)
{
    return c_point{coord_t(std::round(x)), coord_t(std::round(y))};
}

double squared_distance(c_point a, c_point b)
{
    const double dx = double(a.x - b.x), dy = double(a.y - b.y);
    return dx * dx + dy * dy;
}

// Offsets borrowed contact/overhang rings to form removable loops, appends storage-owned interface paths to out and subtracts their footprint from contact; resolution is scaled and geometry/allocation failures propagate.
void contact_loops(storage_handle *storage, MutableExtrusionEntity out, Polys &contact, const Polys &overhang, const c_flow &flow, coord_t resolution)
{
    const ClipperContext clipper(storage);
    const coordf_t radius = 1.5 * double(flow.width);
    const coordf_t distance = radius * 3.;
    const coord_t search_radius = coord_t(distance - SCALED_EPSILON);
    Points circle;
    for (size_t index = 0; index < 6; ++index) {
        const double angle = double(index) * M_PI / 3.;
        circle.push_back(rounded(radius * std::cos(angle), radius * std::sin(angle)));
    }
    const StoredExPolygonCollection contact_union = Common::native_expolygons(clipper_union(clipper(contact)));
    const StoredExPolygonCollection inset = clipper_offset(clipper(contact_union), double(-0.5f * float(flow.width))).to_expolygon_collection();
    const StoredExPolygonCollection overhang_union = Common::native_expolygons(clipper_union(clipper(overhang)));
    const StoredExPolygonCollection margin = clipper_offset(clipper(overhang_union), double(0.5f * float(flow.width))).to_expolygon_collection();
    PointLists contours, circles;
    for (const ExPolygon polygon : inset) {
        std::vector<Points> rings;
        for (const Polygon ring : Common::polygon_rings(polygon))
            rings.push_back(ring.points());
        Points centers;
        c_point last{0, 0};
        for (Points &contour : rings) {
            Points closed = contour;
            closed.push_back(contour.front());
            if (!crosses(storage, PointLists{closed}, margin)) continue;
            // Position on the contour that the walk reached last: -1 when none, otherwise the index of its segment start.
            std::ptrdiff_t current = -1;
            coordf_t current_t = 0.;
            if (centers.empty()) {
                current = 0;
                last = contour.front();
                centers.push_back(last);
            }
            for (size_t i = 1; i < contour.size();) {
                const c_point p1 = contour[i - 1], p2 = contour[i];
                const coordf_t sx = coordf_t(p2.x) - p1.x, sy = coordf_t(p2.y) - p1.y;
                const coordf_t cx = coordf_t(p1.x - last.x), cy = coordf_t(p1.y - last.y);
                const coordf_t a = sx * sx + sy * sy, b = 2. * (sx * cx + sy * cy);
                const coordf_t c = (cx * cx + cy * cy) - distance * distance;
                const coordf_t discriminant = b * b - 4. * a * c;
                if (discriminant > 0.) {
                    const coordf_t t1 = (-b - std::sqrt(discriminant)) / (2. * a);
                    const coordf_t t2 = (-b + std::sqrt(discriminant)) / (2. * a);
                    const coordf_t t0 = current == std::ptrdiff_t(i - 1) ? current_t : 0.;
                    coordf_t t;
                    if (t0 <= t1) t = t0;
                    else if (t2 <= 1.) t = t2;
                    else {
                        current = -1;
                        ++i;
                        continue;
                    }
                    current = std::ptrdiff_t(i - 1);
                    current_t = t;
                    last = c_point{coord_t(p1.x + coord_t(sx * t)), coord_t(p1.y + coord_t(sy * t))};
                    // A center already within the radius replaces this one: the segment is walked again from there.
                    if (near_center(centers, last, search_radius)) continue;
                } else {
                    current = std::ptrdiff_t(i - 1);
                    current_t = 0.;
                    last = p1;
                }
                centers.push_back(last);
                ++i;
            }
            contours.push_back(contour);
            for (const c_point &center : centers) {
                Points moved = circle;
                for (c_point &point : moved) {
                    point.x += center.x;
                    point.y += center.y;
                }
                circles.push_back(moved);
            }
        }
    }
    publish_contact_loops(storage, clipper, out, contact, overhang, flow, resolution, radius, contours, circles);
}

// Subtracts borrowed circle rings from contour rings, clips and rejoins their lines, then moves generated paths into out;
// storage owns geometry, contact loses the loop footprint, resolution is scaled, and geometry/allocation failures propagate.
void publish_contact_loops(storage_handle *storage, const ClipperContext &clipper, MutableExtrusionEntity out, Polys &contact, const Polys &overhang,
                           const c_flow &flow, coord_t resolution, coordf_t radius, const PointLists &contours, const PointLists &circles)
{
    const Polys contour_polygons = polygon_collection(storage, contours);
    const Polys circle_polygons = polygon_collection(storage, circles);
    const Polys loop_polygons = as_polygons(clipper_diff(clipper(contour_polygons), clipper(circle_polygons)));
    PointLists lines;
    SplitPoints split_points;
    for (const Polygon polygon : loop_polygons) {
        const Points points = polygon.points();
        split_points[point_key(points.front())] = -1;
        Points closed = points;
        closed.push_back(points.front());
        lines.push_back(closed);
    }
    const StoredExPolygonCollection expanded = clipper_offset(clipper(overhang), scale_d(1.5)).to_expolygon_collection();
    const StoredPolylineCollection candidates = polyline_collection(storage, lines);
    const StoredPolylineCollection kept = clipper_intersection_polylines_expolygons(storage, candidates, expanded);
    lines = point_lists(kept);
    // A closed contour may come back as two lines meeting at its split point: join them before the loop footprint is removed.
    for (int index = 0; index < int(lines.size()); ++index) {
        Points &line = lines[size_t(index)];
        SplitPoints::iterator split = split_points.find(point_key(line.front()));
        const bool at_start = split != split_points.end();
        if (!at_start) split = split_points.find(point_key(line.back()));
        if (split == split_points.end()) continue;
        if (split->second < 0) {
            split->second = index;
        } else {
            Points &first = lines[size_t(split->second)];
            if (point_key(first.front()) == split->first) std::reverse(first.begin(), first.end());
            if (!at_start) std::reverse(line.begin(), line.end());
            first.insert(first.end(), line.begin() + 1, line.end());
            line.clear();
            split->second = -2;
        }
    }
    StoredPolylineCollection loops = polyline_collection(storage, lines);
    loops.remove_degenerate();
    contact = as_polygons(clipper_diff(clipper(contact), clipper_offset_polylines(storage, loops, radius * 1.1)));
    contact.ensure_valid(resolution);
    loops.ensure_valid(resolution);
    Common::append_paths(storage, out, point_lists(loops), Common::path_attributes(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE, flow), true);
}

std::pair<PointLists, PointLists> split_lines(storage_handle *storage, const PointLists &lines, const ExPolygonCollection &area)
{
    std::pair<PointLists, PointLists> result;
    for (const Points &points : lines) {
        StoredPolyline subject(storage);
        subject.insert_array(0, points.data(), uint32_t(points.size()));
        std::tuple<StoredPolylineCollection, StoredPolylineCollection> split = clipper_split_polyline_expolygons(storage, subject, area);
        for (const Polyline piece : std::get<0>(split))
            result.first.push_back(piece.points());
        for (const Polyline piece : std::get<1>(split))
            result.second.push_back(piece.points());
    }
    return result;
}

// Recursively splits the borrowed entity against ascending lower-Z footprints. A path is rebuilt, with storage-owned height-scaled
// fragments, when some footprint grown by half its width takes a non-empty piece of what the higher footprints left of it, before the
// length filter; changed nonempty fragments are counted. Otherwise the path is left untouched (same object, flags and place), so a path
// that only touches or runs along the edge of a grown footprint stays intact. Missing flow attributes, geometry/allocation failures propagate.
void modulate_entity(storage_handle *storage, MutableExtrusionEntity entity, coord_t z, const std::vector<Internal::Overlap> &overlapping, coord_t resolution)
{
    if (!entity.readonly().has_polyline()) {
        for (uint32_t index = 0; index < entity.readonly().child_count(); ++index)
            modulate_entity(storage, entity.child_mutable(index), z, overlapping, resolution);
        return;
    }
    const ClipperContext clipper(storage);
    const EPropertyAttributes *stored = entity.readonly().get(EPropertyAttributes::key);
    if (!stored) throw std::runtime_error("Missing support path flow attributes");
    const EPropertyAttributes attributes = *stored;
    std::vector<Fragment> fragments(overlapping.size() + 1, Fragment{attributes, {}});
    fragments.back().lines.push_back(entity.readonly().points());
    const c_point start = fragments.back().lines.front().front();
    // Set when some footprint takes a piece of the path: only then the path is rebuilt.
    bool cut = false;
    for (size_t index = overlapping.size(); index-- > 0;) {
        const StoredExPolygonCollection footprint = Common::native_expolygons(clipper_union(clipper(overlapping[index].polygons)));
        StoredExPolygonCollection trimming = clipper_offset(clipper(footprint), scale_d(0.5 * double(attributes.c_extrusion_property_attributes::width))).to_expolygon_collection();
        trimming.ensure_valid(resolution);
        std::pair<PointLists, PointLists> split = split_lines(storage, fragments.back().lines, trimming);
        cut = cut || !split.first.empty();
        fragments[index].lines = Common::valid_polylines(storage, split.first, resolution);
        fragments.back().lines = std::move(split.second);
        const float height = float(unscaled(z - overlapping[index].z));
        if (Common::Counters::enabled && !fragments[index].lines.empty() && height != attributes.c_extrusion_property_attributes::height)
            Common::Counters::add(Common::Counters::totals.height_changes);
        fragments[index].attributes.height(height);
        fragments[index].attributes.mm3_per_mm(fragments[index].attributes.mm3_per_mm() * (height / attributes.c_extrusion_property_attributes::height));
    }
    // No grown footprint took a non-empty piece of the path (one that only touches or runs along its edge takes none): keep the path
    // as it was. The counters only count non-empty fragments, so none was added.
    if (!cut) return;
    StoredExtrusionEntity result(storage);
    c_point current = start;
    size_t remaining = 0;
    for (const Fragment &fragment : fragments)
        remaining += fragment.lines.size();
    while (remaining > 0) {
        Fragment *found_fragment = nullptr;
        Points *found_line = nullptr;
        bool reverse = false;
        double best = 49.;
        for (Fragment &fragment : fragments)
            for (Points &line : fragment.lines) {
                if (line.empty()) continue;
                const double first = squared_distance(line.front(), current);
                const double last = squared_distance(line.back(), current);
                if (first < best || last < best) {
                    reverse = last < first;
                    best = std::min(first, last);
                    found_fragment = &fragment;
                    found_line = &line;
                }
            }
        if (!found_line) {
            // Disconnected rounding fragments remain printable rather than disappearing.
            for (Fragment &fragment : fragments)
                Common::append_paths(storage, result.mutable_view(), fragment.lines, fragment.attributes, true);
            break;
        }
        if (reverse) std::reverse(found_line->begin(), found_line->end());
        found_line->front() = current;
        current = found_line->back();
        StoredExtrusionEntity path(storage, *found_line);
        path.set_flags(0u);
        path.get_or_add(EPropertyAttributes::key) = found_fragment->attributes;
        found_line->clear();
        result.append_child_move(path.mutable_view());
        --remaining;
    }
    // Like the native ExtrusionMultiPath, neither the group nor its fragments may be reversed, so the printing direction
    // is kept. The flags come after the children, since adding the first child resets them to the sortable default.
    result.set_flags(0u);
    extrusion_move_from(entity.mutable_handle(), result.mutable_handle());
}

std::vector<Internal::Overlap> overlaps(storage_handle *storage, const SupportLayerIndex &support_layers, coord_t z, coord_t bottom, bool lower_slabs)
{
    const ClipperContext clipper(storage);
    std::vector<Internal::Overlap> result;
    // Select the open Z interval, then restore auxiliary-list order before the final overlap sort, including ties.
    std::vector<std::pair<uint32_t, const layer_handle *>> selected;
    for (SupportLayerIndex::const_iterator it = support_layers.upper_bound(bottom); it != support_layers.end() && it->first < z; ++it)
        selected.insert(selected.end(), it->second.begin(), it->second.end());
    std::sort(selected.begin(), selected.end(), [](const std::pair<uint32_t, const layer_handle *> &a,
                                                 const std::pair<uint32_t, const layer_handle *> &b) { return a.first < b.first; });
    for (const std::pair<uint32_t, const layer_handle *> &entry : selected) {
        const layer_handle *layer = entry.second;
        const coord_t other_z = layer_get_print_z(layer);
        if (other_z <= bottom || other_z >= z || !plugin_property_has(layer_get_properties(layer), PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT)) continue;
        Polys polygons(storage);
        for (uint32_t island_index = 0; island_index < layer_count_island(layer); ++island_index) {
            const layer_island_handle *island = layer_get_island(layer, island_index);
            for (uint32_t region_index = 0; region_index < layer_island_count_region_island(island); ++region_index) {
                const layer_region_island_handle *region = layer_island_get_region_island(island, region_index);
                polygons.append_move_from(role_polygons(storage, region, SurfaceRole::TopContact));
                polygons.append_move_from(role_polygons(storage, region, SurfaceRole::TopInterface));
                if (lower_slabs) {
                    polygons.append_move_from(role_polygons(storage, region, SurfaceRole::Sparse));
                    polygons.append_move_from(role_polygons(storage, region, SurfaceRole::BottomContact, 0));
                    polygons.append_move_from(role_polygons(storage, region, SurfaceRole::BottomInterface, 0));
                }
            }
        }
        if (!polygons.empty()) {
            Common::Counters::add(Common::Counters::totals.overlap_pairs);
            result.push_back(Internal::Overlap{other_z, as_polygons(clipper_union(clipper(polygons)))});
        }
    }
    std::sort(result.begin(), result.end(), [](const Internal::Overlap &a, const Internal::Overlap &b) { return a.z < b.z; });
    return result;
}

void process_layer(const plugin_run_context *context, const run_ctx_support_toolpath &payload, storage_handle *storage, const Object &object, const PathParams &params,
                   const std::vector<coord_t> &support_z, const SupportLayerIndex &support_layers, SurfaceRole role, layer_handle *layer,
                   const InterfaceFill &interface_fill)
{
    Common::Counters::add(Common::Counters::totals.modulation_layers);
    const Common::SupportParams &geometry = params.geometry;
    const coord_t z = layer_get_print_z(layer), height = layer_get_height(layer), bottom = z - height;
    const bool contact = role == SurfaceRole::TopContact || role == SurfaceRole::BottomContact;
    if (contact) hand_back_contacts(context, storage, layer, role, geometry);
    const LayerSupportProperty &property = *static_cast<const LayerSupportProperty *>(
        plugin_property_data(layer_get_properties(layer), PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT));
    float base_angle = geometry.base_angle;
    const float interface_angle = geometry.interface_angle;
    // The native generator switches a rectilinear-grid base to the interface angle on every second support layer, and counts those layers per print Z.
    const size_t z_index = size_t(std::lower_bound(support_z.begin(), support_z.end(), z) - support_z.begin());
    if (geometry.base_pattern == Common::BasePattern::RectilinearGrid && z_index % 2 == 1)
        base_angle = interface_angle;
    else if (geometry.angle_height > 0 && int(unscaled(z) / geometry.angle_height) % 2 == 1)
        base_angle += float(M_PI / 2.);
    // Every style turns its interfaces by the configured increment from one interface id to the next.
    const float angle = interface_angle + property.interface_id * Common::radians(float(geometry.interface_angle_increment));
    const bool top = role == SurfaceRole::TopContact || role == SurfaceRole::TopInterface;
    const std::string &contact_pattern = top ? geometry.top_pattern_id : geometry.bottom_pattern_id;
    const bool as_base = role == SurfaceRole::Sparse || (top ? !geometry.top_published_interfaces_enabled : !geometry.bottom_published_interfaces_enabled);
    const bool lower_slabs = role == SurfaceRole::BottomContact || (role == SurfaceRole::BottomInterface &&
        layer_polygons(storage, layer, SurfaceRole::BottomContact).empty());
    for (int bridge = 0; bridge != 2; ++bridge) {
        Polys polygons = layer_polygons(storage, layer, role, bridge);
        if (polygons.empty()) continue;
        if (role == SurfaceRole::Sparse && bridge)
            throw std::runtime_error("Sparse support surfaces cannot use bridge flow");
        Pass pass(std::move(polygons), contact, as_base, bridge != 0);
        fill_layer_pass(context, payload, storage, object, geometry, support_layers, role, layer, z, height, bottom,
            base_angle, angle, contact_pattern, pass, lower_slabs, interface_fill);
    }
}

void fill_layer_pass(const plugin_run_context *context, const run_ctx_support_toolpath &payload, storage_handle *storage,
                     const Object &object, const Common::SupportParams &geometry, const SupportLayerIndex &support_layers,
                     SurfaceRole role, layer_handle *layer, coord_t z, coord_t height, coord_t bottom,
                     float base_angle, float angle, const std::string &contact_pattern, Pass &pass, bool lower_slabs,
                     const InterfaceFill &interface_fill)
{
    // Object demand directly above the layer: the removable loops of a top contact follow its outline.
    Polys overhang(storage);
    const coord_t match_tolerance = contact_match_tolerance(geometry);
    if (role == SurfaceRole::TopContact && geometry.top_published_interfaces_enabled && geometry.interface_contact_loops) {
        const uint32_t matched = Internal::matched_layer(object, z + scale_to_layer_coord(geometry.gap_above), match_tolerance, true);
        if (matched < object.layer_count()) {
            const Layer object_layer = object.layer(matched);
            for (uint32_t island = 0; island < object_layer.island_count(); ++island)
                Common::append_polygons(overhang, object_layer.island(island).support_demand());
        }
    }
    StoredExtrusionEntity out(storage);
    const ClipperContext clipper(storage);
    const bool sparse_filler = role == SurfaceRole::Sparse;
    // Only the sparse role uses the base filler; dense roles printed with base settings keep their own filler.
    const bool base_filler = pass.as_base && sparse_filler;
    const raw_extrusion_role interface_role = geometry.top_interfaces_enabled ? RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE : RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL;
    const raw_extrusion_role flow_role = pass.as_base ? RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL : interface_role;
    // Bridge flow follows the surface modifier even after contact is removed.
    c_flow flow = pass.bridge ? object.support_flow(interface_role, geometry.layer_height, unscaled(height), OBJECT_SUPPORT_FLOW_BRIDGE)
                             : object.support_flow(flow_role, geometry.layer_height, unscaled(height));
    double spacing = pass.as_base ? geometry.base_flow_mm.spacing : geometry.interface_flow_mm.spacing;
    float density = float(pass.as_base ? geometry.base_density : geometry.interface_density);
    const std::string pattern = Internal::surface_pattern(pass.as_base, pass.contact, pass.bridge, base_filler, contact_pattern, geometry.base_pattern_id);
    float filler_angle = pass.as_base ? base_angle : angle;
    double filler_z = unscaled(z);
    const bool flange = bottom <= 0;
    if (flange) {
        // The first layer uses the flange flow and raft density; dense fillers use zero angle and print Z as phase.
        flow = geometry.first_flow;
        spacing = geometry.first_flow_mm.spacing;
        density = float(geometry.raft_first_layer_density);
        if (!base_filler) filler_angle = 0;
    }
    // A sheath wraps sparse support; the first layer wraps every pass in a rectilinear perimeter.
    const bool sheath = base_filler && geometry.with_sheath;
    const bool perimeter = flange || sheath;
    const std::string inner = flange ? std::string("rectilinear") : pattern;
    const raw_extrusion_role fill_role = pass.as_base ? RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL : RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE;
    raw_infill_pattern_params recipe = flange && base_filler ? raw_infill_pattern_params{} :
        Common::base_fill_recipe(payload, perimeter ? inner : pattern, fill_role, flow, spacing, density, filler_angle, filler_z);
    // Interfaces have no perimeter to hide a badly placed last line: their lines take the phase that covers the surface best.
    // The strict score keeps the lines off the border, which the XY gap to the object makes mandatory.
    if (!pass.as_base && !flange)
        recipe.infill_flags |= C_INFILL_FLAG_BEST_COVERAGE | C_INFILL_FLAG_BEST_COVERAGE_STRICT;
    if (pass.contact && !overhang.empty()) contact_loops(storage, out.mutable_view(), pass.polygons, overhang, flow, geometry.resolution);
    StoredExPolygonCollection fused = sparse_filler ?
        clipper_offset2(clipper(pass.polygons), double(SCALED_EPSILON), -double(SCALED_EPSILON)).to_expolygon_collection() :
        clipper_union_with_safety_offset(clipper(pass.polygons)).to_expolygon_collection();
    fused.ensure_valid(geometry.resolution);
    StoredExtrusionEntity paths(storage);
    if (flange && base_filler)
        Common::append_base_flange(context, payload, storage, paths.mutable_view(), fused, layer, geometry);
    else if (interface_fill && !pass.as_base && !flange) {
        const c_flow_mm precise_flow = object.support_flow_mm(pass.bridge ? interface_role : flow_role,
            geometry.layer_height, unscaled(height), pass.bridge ? OBJECT_SUPPORT_FLOW_BRIDGE : 0);
        interface_fill({context, payload, storage, object, fused, layer, pattern, precise_flow.spacing, recipe}, paths.mutable_view());
    }
    else
        Common::fill_base_areas(context, payload, storage, paths.mutable_view(), fused, layer, recipe, perimeter);
    // The first bottom role also collects lower sparse and non-bridge bottom slabs.
    Internal::modulate(storage, paths.mutable_view(), z, overlaps(storage, support_layers, z, bottom, lower_slabs), geometry.resolution);
    Common::move_children(out.mutable_view(), paths.mutable_view());
    // A part printed as base joins the support bucket and the support extruder, on this role's own
    // surface; otherwise the role keeps its interface bucket and extruder.
    publish_layer_paths(context, storage, layer, out, geometry, pass.as_base);
}

// Moves borrowed generated children into layer-owned extrusion destinations in child order, choosing the first crossing island;
// as_base selects the bucket/tool, storage owns slice temporaries, and failed assignment/destination access checks cancellation
// before throwing runtime_error; failed insertion throws runtime_error, and geometry/allocation failures propagate.
void publish_layer_paths(const plugin_run_context *context, storage_handle *storage, layer_handle *layer,
                         StoredExtrusionEntity &out, const Common::SupportParams &geometry, bool as_base)
{
    const raw_extrusion_role bucket = as_base ? RAW_EXTRUSION_ROLE_SUPPORT : RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE;
    const int32_t tool = as_base ? geometry.base_tool : geometry.interface_tool;
    for (uint32_t index = 0; index < out.readonly().child_count(); ++index) {
        MutableExtrusionEntity entity = out.child_mutable(index);
        PointLists lines;
        Common::collect_leaf_polylines(entity.readonly(), lines);
        layer_region_island_handle *region = nullptr;
        for (uint32_t island_index = 0; island_index < layer_count_island(layer); ++island_index) {
            layer_island_handle *island = layer_get_island_mutable(layer, island_index);
            const StoredExPolygonCollection slice(storage, ExPolygon(layer_island_get_slice(island)));
            if (!crosses(storage, lines, slice)) continue;
            region = layer_island_get_or_create_region_island(island, nullptr, 0, tool > 0 ? tool - 1 : EXTRUDER_ID_UNKNOWN);
            break;
        }
        if (!region) {
            throw_if_cancelled(context);
            throw std::runtime_error("Cannot assign support path to an island");
        }
        extrusion_entity_handle *destination = layer_region_island_get_mutable_extrusion(region, bucket);
        if (!destination) {
            throw_if_cancelled(context);
            throw std::runtime_error("Cannot access support extrusion destination");
        }
        if (extrusion_insert_child_move(destination, extrusion_child_count(destination), entity.mutable_handle()) == EXTRUSION_INDEX_INVALID)
            throw std::runtime_error("Cannot publish support extrusion");
    }
}

} // namespace

// Scans the window of first_layer_at(expected - tolerance) in layer order and keeps the strictly nearest key, so a tie keeps the first (lower) layer.
// Owns no geometry, changes no layers and propagates view-access failures.
uint32_t Internal::matched_layer(const Object &object, coord_t expected, coord_t tolerance, bool bottom)
{
    uint32_t best = object.layer_count();
    coord_t best_distance = tolerance + 1;
    for (uint32_t index = first_layer_at(object, expected - tolerance, bottom); index < object.layer_count(); ++index) {
        const Layer object_layer = object.layer(index);
        const coord_t key = bottom ? object_layer.bottom_z() : object_layer.print_z();
        if (key > expected + tolerance) break;
        const coord_t distance = std::abs(key - expected);
        if (distance < best_distance) {
            best = index;
            best_distance = distance;
        }
    }
    return best;
}

// Split each leaf path of out that a lower footprint cuts, in place: such a path becomes its fragments in printing order. The
// fragments and their group are neither reversible nor sortable, like the native multipath. A path that no footprint cuts is
// kept as it is, with its flags and order. Preconditions: z and each
// Overlap::z are scaled layer Z values with z above them; overlapping is sorted by ascending z, because the walk trims from
// the highest slab down; storage is borrowed and owns the temporaries. A leaf without EPropertyAttributes throws
// std::runtime_error; an empty overlap list changes nothing.
void Internal::modulate(storage_handle *storage, MutableExtrusionEntity out, coord_t z, const std::vector<Overlap> &overlapping, coord_t resolution)
{
    if (overlapping.empty()) return;
    for (uint32_t index = 0; index < out.readonly().child_count(); ++index)
        modulate_entity(storage, out.child_mutable(index), z, overlapping, resolution);
}

// Choose the pattern plugin of one surface part; a part printed as base uses the base pattern where the base
// filler applies and rectilinear otherwise, and a bridging contact that is not printed as base falls back to rectilinear.
std::string Internal::surface_pattern(bool as_base, bool contact, bool bridge, bool base_filler, const std::string &contact_pattern, const std::string &base_pattern)
{
    const std::string pattern = as_base ? base_filler ? base_pattern : std::string("rectilinear") : !contact ? std::string("rectilinear") : contact_pattern;
    return bridge && pattern == "supportbase" ? std::string("rectilinear") : pattern;
}

// Walk the callback object's marked layers through the C API and append this role's paths; per-call parameters isolate concurrent objects, and invalid payloads fail visibly.
// Borrow context only during this call; layers own published paths. Stopping propagates silent PluginCancelled
// (the cause has its message, the step names it, or the runner rethrows the print cancellation); ordinary refusals
// propagate std::runtime_error.
void generate_paths(const plugin_run_context *context, SurfaceRole role, const InterfaceFill &interface_fill)
{
    std::vector<coord_t> support_z;
    SupportLayerIndex support_layers;
    Common::for_support_layers(context, false,
        // Builds unique support-Z ranks and a print-Z-ordered index of borrowed marked layers with auxiliary-list positions once
        // per object on its first marked layer. The index and handles live through this synchronous for_support_layers call;
        // process_layer publishes paths into layer-owned islands, and allocation/generation failures propagate.
        [context, role, &support_z, &support_layers, &interface_fill](const run_ctx_support_toolpath &payload, const Object &object, const Common::ToolpathParams &params, layer_handle *layer) {
            if (support_z.empty()) {
                for (uint32_t index = 0; index < object_count_auxiliary_layer(payload.object); ++index) {
                    const layer_handle *support_layer = object_get_auxiliary_layer(payload.object, index);
                    if (plugin_property_has(layer_get_properties(support_layer), PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT)) {
                        support_z.push_back(layer_get_print_z(support_layer));
                        support_layers[layer_get_print_z(support_layer)].emplace_back(index, support_layer);
                    }
                }
                std::sort(support_z.begin(), support_z.end());
                support_z.erase(std::unique(support_z.begin(), support_z.end()), support_z.end());
            }
            process_layer(context, payload, context->plugin_storage, object, params, support_z, support_layers, role, layer, interface_fill);
        });
}
} // namespace slic3r_api::Support::Toolpath
