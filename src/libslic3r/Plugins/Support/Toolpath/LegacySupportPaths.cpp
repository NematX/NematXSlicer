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
#include "libslic3r/Plugins/Support/Common/SupportParams.hpp"
#include "libslic3r/Plugins/Support/Common/SupportPaths.hpp"
#include "libslic3r/Plugins/Support/Common/SupportSettings.hpp"

/* Support surfaces describe printable areas, not paths. This file chooses the
   pattern plugin that fills each area and keeps the spacing fixed between layers.
   Contacts are separated from intermediate interfaces by the adjacent object
   slices; bridge surfaces identify non-soluble bottom contacts. Contact loops
   leave small circular breaks for removal. Overlapping slabs split paths and
   reduce their flow in proportion to the remaining thickness. Each role prints
   only its own surfaces: with no interface layers its contact is printed as base
   on that same surface, and no area moves to another role. The generated paths
   are published into the island that holds them, with the tool of the role.
   Filling precedes island assignment, so nearby contours can join in the union.

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
// 4 * SCALED_EPSILON, so the margin is the matching epsilon plus both moves. It stays far below the spacing of two layers of one object, so it cannot
// match another layer. It applies to every print: a layer that close to the exact contact position is the contact.
constexpr coord_t CONTACT_MATCH_TOLERANCE = SCALED_EPSILON + 2 * (4 * SCALED_EPSILON);

// Settings that every layer of one support role reads, derived once per callback from the object.
struct PathParams {
    PathParams(const Print &print, const Object &object, const object_handle *handle);
    Common::SupportParams geometry;
    // Interface layers exist on each side: the top count is nonzero, the bottom one follows it when disabled.
    bool top_enabled = false, bottom_enabled = false;
    // Flow role of the interfaces: the base role when the top interface count is zero.
    raw_extrusion_role interface_role = RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE;
    // One-based tools of the base and of the interfaces, as configured.
    int32_t base_tool = 0, interface_tool = 0;
    // Base angle in degrees, needed for the first-layer flange turned by a quarter turn.
    double base_angle_degrees = 0;
    // Distinct print Z values of the support layers, ascending: a layer's rank replaces the
    // native support layer index, which counts one layer per print Z.
    std::vector<coord_t> support_z;
};
// Parts of one support layer's surfaces that are filled separately.
struct LayerParts {
    explicit LayerParts(storage_handle *storage) : base(storage), top_contact(storage), top_interface(storage), bottom_contact(storage), bottom_interface(storage) {}
    Polys base;             // sparse columns
    Polys top_contact;      // top surfaces touching the object above
    Polys top_interface;    // remaining top surfaces
    Polys bottom_contact;   // bottom bridge surfaces, or bottom surfaces touching the object below
    Polys bottom_interface; // remaining bottom surfaces
};
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
// Copy the contours and holes of one area into dst, like the native polygons_append().
void append_expolygon(Polys &dst, const ExPolygon &expolygon);
// Copy the contours and holes of every area of a collection into dst.
void append_expolygons(Polys &dst, const ExPolygonCollection &expolygons);
// Read the fill surfaces of one role in a region island; bridge 1 or 0 keeps only bridge or non-bridge surfaces.
Polys role_polygons(storage_handle *storage, const layer_region_island_handle *region, SurfaceRole role, int bridge = -1);
// Collect one role across the islands of a support layer before any union.
Polys layer_polygons(storage_handle *storage, const layer_handle *layer, SurfaceRole role, int bridge = -1);
// Split one support layer's typed surfaces into the parts that are filled separately.
LayerParts layer_parts(storage_handle *storage, const Object &object, const layer_handle *layer, const PathParams &params);
// Join the upper contact and its intermediate interfaces, which the top role fills in one pass.
void fuse_parts(LayerParts &parts);
// The attributes of paths with this role and flow: width, height and mm3 per mm.
EPropertyAttributes flow_attributes(raw_extrusion_role role, const c_flow &flow);
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
// Split each polyline by the area, returning the pieces inside and the pieces outside.
std::pair<PointLists, PointLists> split_lines(storage_handle *storage, const PointLists &lines, const ExPolygonCollection &area);
// Clip one path from the highest overlap down, scale its flow by the remaining height and rechain the fragments from its start.
void modulate_entity(storage_handle *storage, MutableExtrusionEntity entity, coord_t z, const std::vector<Internal::Overlap> &overlapping, coord_t resolution);
// Gather the lower support footprints whose top lies between the bottom and the current slab, sorted by Z.
std::vector<Internal::Overlap> overlaps(storage_handle *storage, const object_handle *object, coord_t z, coord_t bottom, bool bottom_contact);
// Generate one role across a support layer and assign its paths to the islands they cross.
void process_layer(const plugin_run_context *context, const run_ctx_support_toolpath &payload, storage_handle *storage, const Object &object, const PathParams &params, SurfaceRole role, layer_handle *layer);

PathParams::PathParams(const Print &print, const Object &object, const object_handle *handle) : geometry(print, object)
{
    const Config config = object.config();
    const int32_t top = config.get("support_material_interface_layers").get_int();
    top_enabled = top != 0;
    const ConfigOption bottom_option = config.get("support_material_bottom_interface_layers");
    bottom_enabled = config_option_is_enabled(bottom_option.handle(), -1) != 0 ? top_enabled && bottom_option.get_int() != 0 : top_enabled;
    interface_role = std::max(0, top) == 0 ? RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL : RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE;
    base_tool = config.get("support_material_extruder").get_int();
    interface_tool = config.get("support_material_interface_extruder").get_int();
    base_angle_degrees = config.get("support_material_angle").get_float();
    for (uint32_t index = 0; index < object_count_auxiliary_layer(handle); ++index) {
        const layer_handle *layer = object_get_auxiliary_layer(handle, index);
        if (plugin_property_has(layer_get_properties(layer), PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT))
            support_z.push_back(layer_get_print_z(layer));
    }
    std::sort(support_z.begin(), support_z.end());
    support_z.erase(std::unique(support_z.begin(), support_z.end()), support_z.end());
}

Polys as_polygons(ClipperOperand &&result)
{
    return result.to_polygon_collection();
}

void append_expolygon(Polys &dst, const ExPolygon &expolygon)
{
    dst.push_back(expolygon.contour());
    for (const Polygon hole : expolygon.holes())
        dst.push_back(hole);
}

void append_expolygons(Polys &dst, const ExPolygonCollection &expolygons)
{
    for (const ExPolygon expolygon : expolygons)
        append_expolygon(dst, expolygon);
}

Polys role_polygons(storage_handle *storage, const layer_region_island_handle *region, SurfaceRole role, int bridge)
{
    Polys result(storage);
    const raw_surface_type mask = RAW_SURFACE_TYPE_MOD_SUPPORT | (role == SurfaceRole::Sparse ?
        RAW_SURFACE_TYPE_POS_INTERNAL | RAW_SURFACE_TYPE_DENS_SPARSE : role == SurfaceRole::Top ?
        RAW_SURFACE_TYPE_POS_TOP | RAW_SURFACE_TYPE_DENS_SOLID : RAW_SURFACE_TYPE_POS_BOTTOM | RAW_SURFACE_TYPE_DENS_SOLID);
    for (uint32_t index = 0; index < layer_region_island_count_fill_surface(region); ++index) {
        const surface_handle *surface = layer_region_island_get_fill_surface(region, index);
        const raw_surface_type type = surface_get_type(surface);
        // Branch bodies belong to the branch role, which no legacy role prints.
        if ((type & mask) != mask || surface_type_is_branch(type) || (bridge >= 0 && bool(type & RAW_SURFACE_TYPE_MOD_BRIDGE) != bool(bridge))) continue;
        append_expolygon(result, ExPolygon(surface_get_expolygon(surface)));
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

LayerParts layer_parts(storage_handle *storage, const Object &object, const layer_handle *layer, const PathParams &params)
{
    const Common::SupportParams &geometry = params.geometry;
    const ClipperContext clipper(storage);
    LayerParts parts(storage);
    parts.base = layer_polygons(storage, layer, SurfaceRole::Sparse);
    const Polys top = layer_polygons(storage, layer, SurfaceRole::Top);
    const Polys bottom = layer_polygons(storage, layer, SurfaceRole::Bottom);
    const coord_t z = layer_get_print_z(layer), layer_bottom = z - layer_get_height(layer);
    if (!params.bottom_enabled) {
        // Without interface layers the published bottom surfaces are the contact alone.
        parts.bottom_contact.copy_from(bottom);
    } else if (!geometry.soluble) {
        // A non-soluble bottom contact is printed as a bridge over the object top surface.
        parts.bottom_contact = layer_polygons(storage, layer, SurfaceRole::Bottom, 1);
        parts.bottom_interface = layer_polygons(storage, layer, SurfaceRole::Bottom, 0);
    } else {
        Polys adjacent(storage);
        for (uint32_t index = 0; index < object.layer_count(); ++index) {
            const Layer object_layer = object.layer(index);
            if (std::abs(object_layer.print_z() - layer_bottom) <= CONTACT_MATCH_TOLERANCE) append_expolygons(adjacent, object_layer.slices());
        }
        parts.bottom_contact = as_polygons(clipper_intersection(clipper(bottom), clipper(adjacent)));
        parts.bottom_interface = as_polygons(clipper_diff(clipper(bottom), clipper(parts.bottom_contact)));
    }
    if (!params.top_enabled) {
        // Without interface layers the published top surfaces are the contact alone.
        parts.top_contact.copy_from(top);
    } else {
        Polys adjacent(storage);
        for (uint32_t index = 0; index < object.layer_count(); ++index) {
            const Layer object_layer = object.layer(index);
            if (std::abs(object_layer.bottom_z() - z - scale_to_layer_coord(geometry.gap_above)) <= CONTACT_MATCH_TOLERANCE) append_expolygons(adjacent, object_layer.slices());
        }
        parts.top_contact = as_polygons(clipper_intersection(clipper(top), clipper(adjacent)));
        parts.top_interface = as_polygons(clipper_diff(clipper(top), clipper(parts.top_contact)));
    }
    return parts;
}

void fuse_parts(LayerParts &parts)
{
    if (!parts.top_contact.empty() && !parts.top_interface.empty()) {
        // The contact and its intermediate interfaces share one filler and one union.
        parts.top_contact.append_move_from(parts.top_interface);
    }
}

EPropertyAttributes flow_attributes(raw_extrusion_role role, const c_flow &flow)
{
    EPropertyAttributes attributes;
    attributes.extrusion_role(int32_t(role)).mm3_per_mm(flow.mm3_per_mm).width(float(unscaled(flow.width))).height(float(unscaled(flow.height)));
    return attributes;
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
        rings.push_back(polygon.contour().points());
        for (const Polygon hole : polygon.holes())
            rings.push_back(hole.points());
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
    Common::append_paths(storage, out, point_lists(loops), flow_attributes(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE, flow), true);
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
    for (size_t index = overlapping.size(); index-- > 0;) {
        const StoredExPolygonCollection footprint = Common::native_expolygons(clipper_union(clipper(overlapping[index].polygons)));
        StoredExPolygonCollection trimming = clipper_offset(clipper(footprint), scale_d(0.5 * double(attributes.c_extrusion_property_attributes::width))).to_expolygon_collection();
        trimming.ensure_valid(resolution);
        std::pair<PointLists, PointLists> split = split_lines(storage, fragments.back().lines, trimming);
        fragments[index].lines = Common::valid_polylines(storage, split.first, resolution);
        fragments.back().lines = std::move(split.second);
        const float height = float(unscaled(z - overlapping[index].z));
        fragments[index].attributes.height(height);
        fragments[index].attributes.mm3_per_mm(fragments[index].attributes.mm3_per_mm() * (height / attributes.c_extrusion_property_attributes::height));
    }
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

std::vector<Internal::Overlap> overlaps(storage_handle *storage, const object_handle *object, coord_t z, coord_t bottom, bool bottom_contact)
{
    const ClipperContext clipper(storage);
    std::vector<Internal::Overlap> result;
    for (uint32_t index = 0; index < object_count_auxiliary_layer(object); ++index) {
        const layer_handle *layer = object_get_auxiliary_layer(object, index);
        const coord_t other_z = layer_get_print_z(layer);
        if (other_z <= bottom || other_z >= z || !plugin_property_has(layer_get_properties(layer), PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT)) continue;
        Polys polygons(storage);
        for (uint32_t island_index = 0; island_index < layer_count_island(layer); ++island_index) {
            const layer_island_handle *island = layer_get_island(layer, island_index);
            for (uint32_t region_index = 0; region_index < layer_island_count_region_island(island); ++region_index) {
                const layer_region_island_handle *region = layer_island_get_region_island(island, region_index);
                polygons.append_move_from(role_polygons(storage, region, SurfaceRole::Top));
                if (bottom_contact) {
                    polygons.append_move_from(role_polygons(storage, region, SurfaceRole::Sparse));
                    polygons.append_move_from(role_polygons(storage, region, SurfaceRole::Bottom, 0));
                }
            }
        }
        if (!polygons.empty()) result.push_back(Internal::Overlap{other_z, as_polygons(clipper_union(clipper(polygons)))});
    }
    std::sort(result.begin(), result.end(), [](const Internal::Overlap &a, const Internal::Overlap &b) { return a.z < b.z; });
    return result;
}

// Generate and move this role's paths into layer-owned islands using borrowed inputs and storage-owned temporaries.
// The context outlives the synchronous fill. Stopping exits through silent PluginCancelled (the cause has its message,
// the step names it, or the runner rethrows the print cancellation); ordinary assignment, destination and pattern
// refusals throw std::runtime_error.
void process_layer(const plugin_run_context *context, const run_ctx_support_toolpath &payload, storage_handle *storage, const Object &object, const PathParams &params, SurfaceRole role, layer_handle *layer)
{
    const Common::SupportParams &geometry = params.geometry;
    const coord_t z = layer_get_print_z(layer), height = layer_get_height(layer), bottom = z - height;
    const bool bridging = !geometry.soluble;
    LayerParts parts = layer_parts(storage, object, layer, params);
    fuse_parts(parts);
    const LayerSupportProperty &property = *static_cast<const LayerSupportProperty *>(
        plugin_property_data(layer_get_properties(layer), PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT));
    float base_angle = geometry.base_angle;
    const float interface_angle = geometry.interface_angle;
    // The native generator switches a rectilinear-grid base to the interface angle on every second support layer, and counts those layers per print Z.
    const size_t z_index = size_t(std::lower_bound(params.support_z.begin(), params.support_z.end(), z) - params.support_z.begin());
    if (geometry.base_pattern == Common::BasePattern::RectilinearGrid && z_index % 2 == 1)
        base_angle = interface_angle;
    else if (geometry.angle_height > 0 && int(unscaled(z) / geometry.angle_height) % 2 == 1)
        base_angle += float(M_PI / 2.);
    float angle = interface_angle;
    if (geometry.interface_angle_increment == 0 && geometry.style == Common::SupportStyle::Snug)
        angle += property.interface_id & 1 ? float(-M_PI / 4.) : float(M_PI / 4.);
    else if (geometry.interface_angle_increment > 0)
        angle += interface_angle + property.interface_id * Common::radians(float(geometry.interface_angle_increment));
    const std::string &contact_pattern = role == SurfaceRole::Top ? geometry.top_pattern_id : geometry.bottom_pattern_id;
    const bool top_as_base = !params.top_enabled, bottom_as_base = !params.bottom_enabled;
    std::vector<Pass> passes;
    if (role == SurfaceRole::Sparse) {
        if (!parts.base.empty()) passes.emplace_back(std::move(parts.base), false, true, false);
    } else if (role == SurfaceRole::Top) {
        if (!parts.top_contact.empty()) passes.emplace_back(std::move(parts.top_contact), true, top_as_base, false);
        if (!parts.top_interface.empty()) passes.emplace_back(std::move(parts.top_interface), false, top_as_base, false);
    } else {
        // A bottom contact keeps the configured contact pattern even when it bridges, like the native filler; only its flow becomes a bridging flow.
        if (!parts.bottom_contact.empty()) passes.emplace_back(std::move(parts.bottom_contact), true, bottom_as_base, bridging);
        if (!parts.bottom_interface.empty()) passes.emplace_back(std::move(parts.bottom_interface), false, bottom_as_base, false);
    }
    if (passes.empty()) return;
    // Object demand directly above the layer: the removable loops of a top contact follow its outline.
    Polys overhang(storage);
    if (role == SurfaceRole::Top && params.top_enabled && geometry.interface_contact_loops)
        for (uint32_t index = 0; index < object.layer_count(); ++index) {
            const Layer object_layer = object.layer(index);
            if (std::abs(object_layer.bottom_z() - z - scale_to_layer_coord(geometry.gap_above)) > CONTACT_MATCH_TOLERANCE) continue;
            for (uint32_t island = 0; island < object_layer.island_count(); ++island)
                append_expolygons(overhang, object_layer.island(island).support_demand());
        }
    StoredExtrusionEntity out(storage);
    const ClipperContext clipper(storage);
    for (Pass &pass : passes) {
        const bool sparse_filler = role == SurfaceRole::Sparse;
        // Only the sparse role uses the base filler; a contact printed with the base settings keeps its own filler.
        const bool base_filler = pass.as_base && sparse_filler;
        const raw_extrusion_role flow_role = pass.as_base ? RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL : params.interface_role;
        // The native filler picks the bridging flow first, even for a contact printed with the base settings.
        c_flow flow = pass.bridge ? object.support_flow(params.interface_role, geometry.layer_height, unscaled(height), OBJECT_SUPPORT_FLOW_BRIDGE)
                                  : object.support_flow(flow_role, geometry.layer_height, unscaled(height));
        double spacing = pass.as_base ? geometry.base_flow_mm.spacing : geometry.interface_flow_mm.spacing;
        float density = float(pass.as_base ? geometry.base_density : geometry.interface_density);
        const std::string pattern = Internal::surface_pattern(pass.as_base, pass.contact, pass.bridge, base_filler, contact_pattern, geometry.base_pattern_id);
        float filler_angle = pass.as_base ? base_angle : angle;
        double filler_z = unscaled(z);
        const bool flange = bottom <= 0;
        if (flange) {
            // The native first layer fills its flange with the first-layer flow and the raft density.
            // The base flange keeps the filler phase at Z = 0 and turns the base angle by a quarter turn;
            // a contact printed with its own filler uses no angle and the print Z as phase.
            flow = geometry.first_flow;
            spacing = geometry.first_flow_mm.spacing;
            density = float(geometry.raft_first_layer_density);
            if (base_filler) {
                filler_angle = Common::radians(float(params.base_angle_degrees + 90.));
                filler_z = 0.;
            } else {
                filler_angle = 0;
            }
        }
        // The base filler wraps its pattern in a perimeter when the sheath is on; the first layer wraps every pass in a rectilinear one.
        const bool sheath = base_filler && geometry.with_sheath;
        const bool perimeter = flange || sheath;
        const std::string inner = flange ? std::string("rectilinear") : pattern;
        const raw_extrusion_role fill_role = pass.as_base ? RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL : RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE;
        const raw_infill_pattern_params recipe = Common::base_fill_recipe(payload, perimeter ? inner : pattern, fill_role, flow, spacing,
                                                                          density, filler_angle, filler_z);
        if (pass.contact && !overhang.empty()) contact_loops(storage, out.mutable_view(), pass.polygons, overhang, flow, geometry.resolution);
        StoredExPolygonCollection fused = sparse_filler ?
            clipper_offset2(clipper(pass.polygons), double(SCALED_EPSILON), -double(SCALED_EPSILON)).to_expolygon_collection() :
            clipper_union_with_safety_offset(clipper(pass.polygons)).to_expolygon_collection();
        fused.ensure_valid(geometry.resolution);
        StoredExtrusionEntity paths(storage);
        Common::fill_base_areas(context, payload, storage, paths.mutable_view(), fused, layer, recipe, perimeter);
        // A bottom contact also overlaps sparse and interface slabs, so it collects them for modulation.
        const bool bottom_contact_pass = role == SurfaceRole::Bottom && &pass == &passes.front();
        Internal::modulate(storage, paths.mutable_view(), z, overlaps(storage, payload.object, z, bottom, bottom_contact_pass), geometry.resolution);
        Common::move_children(out.mutable_view(), paths.mutable_view());
    }
    // A part printed as base joins the support bucket and the support extruder, on this role's own
    // surface; otherwise the role keeps its interface bucket and extruder.
    const bool as_base = role == SurfaceRole::Sparse || (role == SurfaceRole::Top ? top_as_base : bottom_as_base);
    const raw_extrusion_role bucket = as_base ? RAW_EXTRUSION_ROLE_SUPPORT : RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE;
    const int32_t tool = as_base ? params.base_tool : params.interface_tool;
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

// Split each leaf path of out against the lower footprints, in place: a path becomes its fragments in printing order. The
// fragments and their group are neither reversible nor sortable, like the native multipath. Preconditions: z and each
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
void generate_paths(const plugin_run_context *context, SurfaceRole role)
{
    const run_ctx_support_toolpath *payload = plugin_ctx_as_support_toolpath(context);
    if (!payload || !payload->object || !payload->print) throw std::runtime_error("Missing support toolpath payload");
    if (object_count_auxiliary_layer(payload->object) == 0) return;
    const Print print(payload->print);
    const Object object(payload->object);
    const PathParams params(print, object, payload->object);
    for (uint32_t layer_index = 0; layer_index < object_count_auxiliary_layer(payload->object); ++layer_index) {
        throw_if_cancelled(context);
        layer_handle *layer = object_get_auxiliary_layer(payload->object, layer_index);
        if (!plugin_property_has(layer_get_properties(layer), PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT)) continue;
        process_layer(context, *payload, context->plugin_storage, object, params, role, layer);
    }
}
} // namespace slic3r_api::Support::Toolpath
