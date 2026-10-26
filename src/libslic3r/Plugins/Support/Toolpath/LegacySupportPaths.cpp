///|/ Copyright (c) Prusa Research 2023 Vojtech Bubnik, Pavel Mikus.
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "LegacySupportPaths.hpp"

#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_toolpath.h"
#include "libslic3r/Api/plugin/c/slic3r_extrusion_entity.h"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Fill/FillBase.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/PointUtils.hpp"
#include "libslic3r/Plugins/Support/Grid/GridSupportParams.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"

/* Support surfaces describe printable areas, not paths. This file selects
   native fillers and keeps their spacing fixed between layers. Contacts are
   separated from intermediate interfaces by adjacent object slices; bridge
   surfaces identify non-soluble bottom contacts. Contact loops leave small
   circular breaks for removal. Overlapping slabs split paths and reduce their
   flow in proportion to the remaining thickness. Each role prints only its own
   surfaces: with no interface layers its contact is printed as base on that
   same surface, and no area moves to another role. API insertion transfers the
   generated paths to the first island intersecting them, with the role's tool.
   Filling precedes island assignment so nearby contours can join during the
   native safety-offset union. */

namespace slic3r_api::Support::Toolpath {
namespace {
using namespace Slic3r;
struct PathParams {
    // Derive pattern choices and densities while sharing the grid family's flow settings.
    explicit PathParams(const PrintObject &object);
    SupportGrid::GridSupportParams grid;
    PrintRegionConfig region;
    double density;
    InfillPattern base_pattern, top_pattern, bottom_pattern;
    // Distinct support print Z values in ascending order; a layer's rank in it replaces the
    // native support layer index, which counts one layer per print Z.
    std::vector<coord_t> support_z;
};
// One support layer's surfaces split into the parts the native generator fills separately.
struct LayerParts {
    Polygons base;             // sparse columns
    Polygons top_contact;      // top surfaces touching the object above
    Polygons top_interface;    // remaining top surfaces
    Polygons bottom_contact;   // bottom bridge surfaces, or bottom surfaces touching the object below
    Polygons bottom_interface; // remaining bottom surfaces
};
// Collect the polygons for one role from a region island, optionally filtering bridge surfaces.
Polygons role_polygons(const layer_region_island_handle *region, SurfaceRole role, int bridge = -1);
// Collect one role across a support layer before native polygon fusion.
Polygons layer_polygons(const layer_handle *layer, SurfaceRole role, int bridge = -1);
// Split one support layer's typed surfaces into the parts the native generator fills separately.
LayerParts layer_parts(const PrintObject &object, const layer_handle *layer, const SupportGrid::GridSupportParams &grid);
// Join the upper contact and its intermediate interfaces into one filler pass of the top role.
void fuse_parts(LayerParts &parts);
// Cut removable contact loops out of a contact part and append their interface paths to out.
void contact_loops(ExtrusionEntityCollection &out, Polygons &contact, const Polygons &overhang, const Flow &flow, coord_t resolution);
// Fill owned polygons with fixed spacing and append native flow-bearing paths to the destination.
void fill_paths(ExtrusionEntityCollection &out, ExPolygons polygons, Fill &filler, float density, ExtrusionRole role, const Flow &flow, double spacing, const PrintRegionConfig &config);
// Apply overlap splitting recursively to native extrusion leaves, keeping collection ordering and flags.
void modulate_entity(ExtrusionEntity &entity, coord_t z, const std::vector<Internal::Overlap> &overlapping, coord_t resolution);
// Gather lower support footprints whose top lies inside the current printable slab.
std::vector<Internal::Overlap> overlaps(const object_handle *object, coord_t z, coord_t bottom, bool bottom_contact);
// Generate one role across a layer and assign its paths to intersecting islands.
void process_layer(const PrintObject &object, const PathParams &params, SurfaceRole role, layer_handle *layer);

// Resolve contact pattern defaults and base density from an already sliced object; the instance owns its configuration copy and flows.
PathParams::PathParams(const PrintObject &object) : grid(object)
{
    const PrintObjectConfig &config = object.config();
    region = object.default_region_config(object.print()->default_region_config());
    region.parent = &config;
    // The native generator sorts its support layers by print Z and keeps one per Z, so the
    // distinct Z of the published auxiliary layers give the same per-Z layer index.
    for (const Layer &layer : object.auxiliary_layers())
        if (layer.get_property<LayerSupportProperty>() != nullptr) support_z.push_back(layer.scaled_print_z());
    std::sort(support_z.begin(), support_z.end());
    support_z.erase(std::unique(support_z.begin(), support_z.end()), support_z.end());
    density = std::min(1., grid.base_flow.spacing() / (config.support_material_spacing.value + grid.base_flow.spacing()));
    base_pattern = config.support_material_pattern.value == smpHoneycomb ? ipHoneycomb :
        density > 0.95 || config.support_material_with_sheath.value ? ipRectilinear : ipSupportBase;
    top_pattern = config.support_material_top_interface_pattern.value;
    bottom_pattern = config.support_material_bottom_interface_pattern.value;
    if (top_pattern == ipAuto)
        top_pattern = grid.soluble ? ipConcentric : grid.interface_density > 0.95 ? ipRectilinear : ipSupportBase;
    if (bottom_pattern == ipAuto)
        bottom_pattern = top_pattern != ipHilbertCurve && top_pattern != ipSmooth && top_pattern != ipSawtooth ? top_pattern :
            grid.soluble ? ipConcentric : grid.interface_density > 0.95 ? ipRectilinear : ipSupportBase;
}

// Read support surface bits and copy matching contours; the region remains owned by the data tree and the returned polygons own their points.
Polygons role_polygons(const layer_region_island_handle *region, SurfaceRole role, int bridge)
{
    Polygons result;
    const raw_surface_type mask = RAW_SURFACE_TYPE_MOD_SUPPORT | (role == SurfaceRole::Sparse ?
        RAW_SURFACE_TYPE_POS_INTERNAL | RAW_SURFACE_TYPE_DENS_SPARSE : role == SurfaceRole::Top ?
        RAW_SURFACE_TYPE_POS_TOP | RAW_SURFACE_TYPE_DENS_SOLID : RAW_SURFACE_TYPE_POS_BOTTOM | RAW_SURFACE_TYPE_DENS_SOLID);
    for (uint32_t index = 0; index < layer_region_island_count_fill_surface(region); ++index) {
        const surface_handle *surface = layer_region_island_get_fill_surface(region, index);
        const raw_surface_type type = surface_get_type(surface);
        if ((type & mask) != mask || (bridge >= 0 && bool(type & RAW_SURFACE_TYPE_MOD_BRIDGE) != bool(bridge))) continue;
        polygons_append(result, *reinterpret_cast<const ExPolygon *>(surface_get_expolygon(surface)));
    }
    return result;
}

// Copy matching surface contours across all islands; the layer stays host-owned and the returned polygons remain valid during filling.
Polygons layer_polygons(const layer_handle *layer, SurfaceRole role, int bridge)
{
    Polygons result;
    for (uint32_t island_index = 0; island_index < layer_count_island(layer); ++island_index) {
        const layer_island_handle *island = layer_get_island(layer, island_index);
        for (uint32_t region_index = 0; region_index < layer_island_count_region_island(island); ++region_index)
            polygons_append(result, role_polygons(layer_island_get_region_island(island, region_index), role, bridge));
    }
    return result;
}

// Split one support layer's typed surfaces into the parts the native generator fills separately; adjacent object layers separate contacts from interfaces, and roles without interface layers keep one part.
LayerParts layer_parts(const PrintObject &object, const layer_handle *layer, const SupportGrid::GridSupportParams &grid)
{
    const PrintObjectConfig &config = object.config();
    const bool top_enabled = config.support_material_interface_layers.value != 0;
    const bool bottom_enabled = config.support_material_bottom_interface_layers.is_enabled() ?
        top_enabled && config.support_material_bottom_interface_layers.value != 0 : top_enabled;
    LayerParts parts;
    parts.base = layer_polygons(layer, SurfaceRole::Sparse);
    const Polygons top = layer_polygons(layer, SurfaceRole::Top);
    const Polygons bottom = layer_polygons(layer, SurfaceRole::Bottom);
    const coord_t z = layer_get_print_z(layer), layer_bottom = z - layer_get_height(layer);
    if (!bottom_enabled) {
        // Without interface layers the published bottom surfaces are the contact alone.
        parts.bottom_contact = bottom;
    } else if (!grid.soluble) {
        // A non-soluble bottom contact is printed as a bridge over the object top surface.
        parts.bottom_contact = layer_polygons(layer, SurfaceRole::Bottom, 1);
        parts.bottom_interface = layer_polygons(layer, SurfaceRole::Bottom, 0);
    } else {
        Polygons adjacent;
        for (const Layer &object_layer : object.layers())
            if (std::abs(object_layer.scaled_print_z() - layer_bottom) <= SCALED_EPSILON) polygons_append(adjacent, object_layer.lslices());
        parts.bottom_contact = intersection(bottom, adjacent);
        parts.bottom_interface = diff(bottom, parts.bottom_contact);
    }
    if (!top_enabled) {
        // Without interface layers the published top surfaces are the contact alone.
        parts.top_contact = top;
    } else {
        Polygons adjacent;
        for (const Layer &object_layer : object.layers())
            if (std::abs(object_layer.scaled_bottom_z() - z - scale_to_layer_coord(grid.gap_above)) <= SCALED_EPSILON) polygons_append(adjacent, object_layer.lslices());
        parts.top_contact = intersection(top, adjacent);
        parts.top_interface = diff(top, parts.top_contact);
    }
    return parts;
}

// Join the upper contact and its intermediate interfaces, which the top role prints in one filler
// pass; every other part stays with the role that owns it, because each role now runs its own
// algorithm and never prints another role's area.
void fuse_parts(LayerParts &parts)
{
    if (!parts.top_contact.empty() && !parts.top_interface.empty()) {
        // The contact and its intermediate interfaces share one filler and one union.
        polygons_append(parts.top_contact, parts.top_interface);
        parts.top_interface.clear();
    }
}

// Initialize each polygon's filler with the specified millimetre spacing and flow; native paths become owned by out and filler errors reach PluginBase.
void fill_paths(ExtrusionEntityCollection &out, ExPolygons polygons, Fill &filler, float density, ExtrusionRole role, const Flow &flow, double spacing, const PrintRegionConfig &config)
{
    FillParams params;
    params.density = density;
    params.dont_adjust = true;
    params.config = &config;
    params.flow = flow;
    params.role = role;
    ExtrusionEntitiesPtr entities;
    for (ExPolygon &polygon : polygons) {
        filler.init_spacing(spacing, params);
        Surface surface(stPosInternal | stDensSparse, std::move(polygon));
        filler.fill_surface_extrusion(&surface, params, entities);
    }
    out.append(std::move(entities));
}

// Place hexagonal breaks along contact contours facing the overhang, stitch clipped loops and subtract their footprint from contact; out owns the resulting interface paths.
void contact_loops(ExtrusionEntityCollection &out, Polygons &contact, const Polygons &overhang, const Flow &flow, coord_t resolution)
{
    const coordf_t radius = 1.5 * flow.scaled_width();
    const coordf_t distance = radius * 3.;
    Polygon circle;
    for (size_t index = 0; index < 6; ++index) {
        const double angle = double(index) * M_PI / 3.;
        circle.points.emplace_back(radius * std::cos(angle), radius * std::sin(angle));
    }
    struct PointAccessor {
        // Return the stored center for the nearby-circle lookup.
        const Point *operator()(const Point &point) const { return &point; }
    };
    typedef ClosestPointInRadiusLookup<Point, PointAccessor> CircleLookup;
    ExPolygons inset = offset_ex(union_ex(contact), -0.5f * flow.scaled_width());
    const Polygons margin = offset(union_ex(overhang), 0.5f * flow.scaled_width());
    Polygons contours, circles;
    for (ExPolygon &polygon : inset) {
        CircleLookup lookup(coord_t(distance - SCALED_EPSILON));
        Points centers;
        Point last;
        for (size_t contour_index = 0; contour_index <= polygon.holes.size(); ++contour_index) {
            Polygon &contour = contour_index == 0 ? polygon.contour : polygon.holes[contour_index - 1];
            if (intersection_pl(contour.split_at_first_point(), margin).empty()) continue;
            const Point *current = nullptr;
            coordf_t current_t = 0.;
            if (centers.empty()) {
                current = &contour.points.front();
                last = *current;
                lookup.insert(last);
                centers.push_back(last);
            }
            for (Points::const_iterator point = contour.points.begin() + 1; point != contour.points.end(); ++point) {
                const Point &p1 = *(point - 1), &p2 = *point;
                const Vec2d segment(coordf_t(p2.x()) - p1.x(), coordf_t(p2.y()) - p1.y());
                const Vec2d center(coordf_t(p1.x() - last.x()), coordf_t(p1.y() - last.y()));
                const coordf_t a = segment.squaredNorm(), b = 2. * segment.dot(center);
                const coordf_t c = center.squaredNorm() - distance * distance;
                const coordf_t discriminant = b * b - 4. * a * c;
                if (discriminant > 0.) {
                    const coordf_t t1 = (-b - std::sqrt(discriminant)) / (2. * a);
                    const coordf_t t2 = (-b + std::sqrt(discriminant)) / (2. * a);
                    const coordf_t t0 = current == &p1 ? current_t : 0.;
                    coordf_t t;
                    if (t0 <= t1) t = t0;
                    else if (t2 <= 1.) t = t2;
                    else { current = nullptr; continue; }
                    current = &p1;
                    current_t = t;
                    last = Point(p1.x() + coord_t(segment.x() * t), p1.y() + coord_t(segment.y() * t));
                    if (lookup.find(last).first != nullptr) { --point; continue; }
                } else {
                    current = &p1;
                    current_t = 0.;
                    last = p1;
                }
                lookup.insert(last);
                centers.push_back(last);
            }
            contours.push_back(std::move(contour));
            for (const Point &center : centers) {
                circles.push_back(circle);
                circles.back().translate(center);
            }
        }
    }
    const Polygons loop_polygons = diff(contours, circles);
    Polylines lines;
    std::unordered_map<Point, int, PointHash> split_points;
    for (const Polygon &polygon : loop_polygons) {
        split_points[polygon.first_point()] = -1;
        lines.push_back(polygon.split_at_first_point());
    }
    lines = intersection_pl(lines, expand(overhang, scale_d(1.5)));
    // A closed contour may be returned as two lines meeting at its original
    // split point. Join these before excluding the loop footprint from infill.
    for (int index = 0; index < int(lines.size()); ++index) {
        Polyline &line = lines[index];
        std::unordered_map<Point, int, PointHash>::iterator split = split_points.find(line.first_point());
        const bool at_start = split != split_points.end();
        if (!at_start) split = split_points.find(line.last_point());
        if (split == split_points.end()) continue;
        assert(split->second != -2);
        if (split->second < 0) split->second = index;
        else {
            Polyline &first = lines[split->second];
            if (first.first_point() == split->first) first.reverse();
            if (!at_start) line.reverse();
            first.append(std::move(line));
            split->second = -2;
        }
    }
    remove_degenerate(lines);
    contact = ensure_valid(diff(contact, offset(lines, radius * 1.1)), resolution);
    ensure_valid(lines, resolution);
    extrusion_entities_append_paths(out, std::move(lines), {ExtrusionRole::SupportMaterialInterface, flow});
}

// Clip one path from highest overlap down, scale its original flow by remaining height and rechain fragments from its original start; child collections retain ownership and order.
void modulate_entity(ExtrusionEntity &entity, coord_t z, const std::vector<Internal::Overlap> &overlapping, coord_t resolution)
{
    if (!entity.has_polyline()) {
        for (ExtrusionEntityUPtr &child : entity.children()) modulate_entity(*child, z, overlapping, resolution);
        return;
    }
    const ExtrusionAttributes *stored_attributes = entity.get_property<ExtrusionAttributes>();
    if (!stored_attributes) throw std::runtime_error("Missing support path flow attributes");
    const ExtrusionAttributes attributes = *stored_attributes;
    struct Fragment { ExtrusionAttributes attributes; Polylines lines; };
    std::vector<Fragment> fragments(overlapping.size() + 1, Fragment{attributes, {}});
    fragments.back().lines.push_back(entity.polyline_ref().to_polyline());
    const Point start = fragments.back().lines.front().front();
    for (size_t index = overlapping.size(); index-- > 0;) {
        Polygons trimming = offset(union_ex(overlapping[index].polygons), scale_d(0.5 * attributes.width));
        ensure_valid(trimming, resolution);
        std::pair<Polylines, Polylines> split = split_pl(fragments.back().lines, trimming);
        fragments[index].lines = ensure_valid(std::move(split.first), resolution);
        fragments.back().lines = std::move(split.second);
        const float height = float(unscaled(z - overlapping[index].z));
        fragments[index].attributes.height = height;
        fragments[index].attributes.mm3_per_mm *= height / attributes.height;
    }
    ExtrusionEntityCollection result;
    result.set_can_sort_reverse(false, entity.can_reverse());
    Point current = start;
    size_t remaining = 0;
    for (const Fragment &fragment : fragments) remaining += fragment.lines.size();
    while (remaining > 0) {
        Fragment *found_fragment = nullptr;
        Polyline *found_line = nullptr;
        bool reverse = false;
        double best = 49.;
        for (Fragment &fragment : fragments)
            for (Polyline &line : fragment.lines) {
                if (line.empty()) continue;
                const double first = (line.front() - current).cast<double>().squaredNorm();
                const double last = (line.back() - current).cast<double>().squaredNorm();
                if (first < best || last < best) {
                    reverse = last < first;
                    best = std::min(first, last);
                    found_fragment = &fragment;
                    found_line = &line;
                }
            }
        if (!found_line) {
            // Disconnected rounding fragments remain printable rather than
            // disappearing when Clipper cannot reconnect their endpoints.
            for (Fragment &fragment : fragments)
                extrusion_entities_append_paths(result, std::move(fragment.lines), ExtrusionAttributes(fragment.attributes));
            break;
        }
        if (reverse) found_line->reverse();
        found_line->points.front() = current;
        current = found_line->back();
        ExtrusionPath path(found_fragment->attributes);
        path.set_polyline(std::move(*found_line));
        found_line->points.clear();
        result.append(std::move(path));
        --remaining;
    }
    entity = std::move(result);
}

// Read marked auxiliary layers between bottom and z and union their relevant surfaces; the returned footprint copies remain valid throughout this callback.
std::vector<Internal::Overlap> overlaps(const object_handle *object, coord_t z, coord_t bottom, bool bottom_contact)
{
    std::vector<Internal::Overlap> result;
    for (uint32_t index = 0; index < object_count_auxiliary_layer(object); ++index) {
        const layer_handle *layer = object_get_auxiliary_layer(object, index);
        const coord_t other_z = layer_get_print_z(layer);
        if (other_z <= bottom || other_z >= z || !plugin_property_has(layer_get_properties(layer), PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT)) continue;
        Polygons polygons;
        for (uint32_t island_index = 0; island_index < layer_count_island(layer); ++island_index) {
            const layer_island_handle *island = layer_get_island(layer, island_index);
            for (uint32_t region_index = 0; region_index < layer_island_count_region_island(island); ++region_index) {
                const layer_region_island_handle *region = layer_island_get_region_island(island, region_index);
                polygons_append(polygons, role_polygons(region, SurfaceRole::Top));
                if (bottom_contact) {
                    polygons_append(polygons, role_polygons(region, SurfaceRole::Sparse));
                    polygons_append(polygons, role_polygons(region, SurfaceRole::Bottom, 0));
                }
            }
        }
        if (!polygons.empty()) result.push_back(Internal::Overlap{other_z, union_(polygons)});
    }
    std::sort(result.begin(), result.end(), [](const Internal::Overlap &a, const Internal::Overlap &b) { return a.z < b.z; });
    return result;
}

// Fill one role's share of a support layer after fuse_parts joined its own contact and interfaces, then move each path to its first intersecting island and configured tool; missing islands or rejected API destinations throw without suppressing failures.
void process_layer(const PrintObject &object, const PathParams &params, SurfaceRole role, layer_handle *layer)
{
    const PrintObjectConfig &config = object.config();
    const SupportGrid::GridSupportParams &grid = params.grid;
    const coord_t z = layer_get_print_z(layer), height = layer_get_height(layer), bottom = z - height;
    const bool bridging = !grid.soluble;
    const bool top_enabled = config.support_material_interface_layers.value != 0;
    const bool bottom_enabled = config.support_material_bottom_interface_layers.is_enabled() ?
        top_enabled && config.support_material_bottom_interface_layers.value != 0 : top_enabled;
    LayerParts parts = layer_parts(object, layer, grid);
    fuse_parts(parts);
    const LayerSupportProperty &property = *static_cast<const LayerSupportProperty *>(
        plugin_property_data(layer_get_properties(layer), PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT));
    float base_angle = Geometry::deg2rad(float(config.support_material_angle.value));
    const float interface_angle = Geometry::deg2rad(float(config.support_material_interface_angle.value));
    // The native generator switches a rectilinear-grid base to the interface angle on every
    // second support layer, and counts those layers per print Z, not per auxiliary layer.
    const size_t z_index = size_t(std::lower_bound(params.support_z.begin(), params.support_z.end(), z) - params.support_z.begin());
    if (config.support_material_pattern.value == smpRectilinearGrid && z_index % 2 == 1)
        base_angle = interface_angle;
    else if (config.support_material_angle_height.value > 0 && int(unscaled(z) / config.support_material_angle_height.value) % 2 == 1)
        base_angle += float(M_PI / 2.);
    float angle = interface_angle;
    if (config.support_material_interface_angle_increment.value == 0 && config.support_material_style.value == smsSnug)
        angle += property.interface_id & 1 ? float(-M_PI / 4.) : float(M_PI / 4.);
    else if (config.support_material_interface_angle_increment.value > 0)
        angle += interface_angle + property.interface_id * Geometry::deg2rad(float(config.support_material_interface_angle_increment.value));
    /* One filler pass over a part of this role's own surface. A support layer is made of:
       - a contact: the part touching the object, above for the top role and below for the bottom
         one;
       - interfaces: the dense layers printed between a contact and the sparse base;
       - the base: the sparse columns that carry the load down to the bed.
       Contacts and interfaces belong to the same role here; only the base belongs to another one.
       With no interface layer on its side, the native generator printed the contact with the base
       settings (base flow, spacing, density and angle, SupportMaterial role) because it treated
       that contact as a generic base layer. It additionally merged the contact area into the base
       layer when the support regions could merge, so the area was then filled by the base filler
       itself. Areas no longer change owner here: the contact stays on this role's surface, takes
       the same base settings, and always its own filler, because a contact that the native generator
       would have merged into the base is published as base by the family and never reaches this
       plugin as a contact. */
    struct Pass {
        Polygons polygons;
        bool contact = false; // prints with the configured contact pattern
        bool as_base = false; // prints with the base settings: flow, spacing, angle, density and role
        bool bridge = false;  // prints with the bridging flow
    };
    const bool top_as_base = !top_enabled;
    const bool bottom_as_base = !bottom_enabled;
    std::vector<Pass> passes;
    if (role == SurfaceRole::Sparse) {
        if (!parts.base.empty()) passes.push_back({std::move(parts.base), false, true, false});
    } else if (role == SurfaceRole::Top) {
        if (!parts.top_contact.empty()) passes.push_back({std::move(parts.top_contact), true, top_as_base, false});
        if (!parts.top_interface.empty()) passes.push_back({std::move(parts.top_interface), false, top_as_base, false});
    } else {
        // A bottom contact keeps the configured contact pattern even when it bridges, like the native filler; only its flow becomes a bridging flow.
        if (!parts.bottom_contact.empty()) passes.push_back({std::move(parts.bottom_contact), true, bottom_as_base, bridging});
        if (!parts.bottom_interface.empty()) passes.push_back({std::move(parts.bottom_interface), false, bottom_as_base, false});
    }
    if (passes.empty()) return;
    Polygons overhang;
    if (role == SurfaceRole::Top && top_enabled && config.support_material_interface_contact_loops.value)
        for (const Layer &object_layer : object.layers())
            if (std::abs(object_layer.scaled_bottom_z() - z - scale_to_layer_coord(grid.gap_above)) <= SCALED_EPSILON)
                for (const LayerSliceIsland &island : object_layer.islands()) polygons_append(overhang, island.support_demand());
    const InfillPattern contact_pattern = role == SurfaceRole::Top ? params.top_pattern : params.bottom_pattern;
    ExtrusionEntityCollection out;
    for (Pass &pass : passes) {
        const bool sparse_filler = role == SurfaceRole::Sparse;
        /* Only the sparse role uses the base filler. A contact the native generator would have merged
           into the base is published as base by the family and reaches this plugin as a sparse
           surface, so a contact printed with the base settings here always keeps its own filler: the
           family decides the type, and this plugin only prints the surfaces it is given. */
        const bool base_filler = pass.as_base && sparse_filler;
        const Flow flow_source = pass.as_base ? grid.base_flow : grid.interface_flow;
        // The native filler picks the bridging flow first, even for a contact printed with the base
        // settings, so a bridging bottom contact keeps its bridging section.
        Flow flow = pass.bridge ? Flow::bridging_flow(float(unscaled(height)), grid.bottom_flow.nozzle_diameter()) :
            flow_source.with_height(float(unscaled(height)));
        double spacing = flow_source.spacing();
        float density = float(pass.as_base ? params.density : grid.interface_density);
        const InfillPattern pattern = Internal::surface_pattern(pass.as_base, pass.contact, pass.bridge, base_filler, contact_pattern, params.base_pattern);
        std::unique_ptr<Fill> filler;
        // The base filler wraps its pattern in a perimeter when the sheath is on.
        if (bottom <= 0 || (base_filler && config.support_material_with_sheath.value)) {
            FillWithPerimeter *perimeter = new FillWithPerimeter(Fill::new_from_type(bottom <= 0 ? ipRectilinear : pattern));
            perimeter->ratio_fill_inside = 0.2f;
            filler.reset(perimeter);
        } else filler.reset(Fill::new_from_type(pattern));
        filler->set_bounding_box(BoundingBox(Point(-scale_i(1.), -scale_i(1.)), Point(scale_i(1.), scale_i(1.))));
        filler->z = unscaled(z);
        filler->angle = pass.as_base ? base_angle : angle;
        if (bottom <= 0) {
            // The native first layer fills its flange with the first-layer flow and the raft density.
            // The base flange keeps the filler phase at Z = 0 and turns the base angle by a quarter
            // turn; a contact printed with its own filler uses the native interface first-layer
            // settings instead: no angle and the print Z as phase.
            flow = grid.first_flow;
            spacing = flow.spacing();
            density = float(config.raft_first_layer_density.get_effective_value(1.));
            if (base_filler) {
                filler->angle = Geometry::deg2rad(float(config.support_material_angle.value + 90.));
                filler->z = 0.;
            } else {
                filler->angle = 0;
            }
        }
        filler->link_max_length = 0;
        if (pass.contact && !overhang.empty())
            contact_loops(out, pass.polygons, overhang, flow, grid.resolution);
        ExPolygons fused = sparse_filler ? closing_ex(pass.polygons, float(SCALED_EPSILON), float(SCALED_EPSILON)) : union_safety_offset_ex(pass.polygons);
        ensure_valid(fused, grid.resolution);
        ExtrusionEntityCollection paths;
        fill_paths(paths, std::move(fused), *filler, density, pass.as_base ? ExtrusionRole::SupportMaterial : ExtrusionRole::SupportMaterialInterface,
            flow, spacing, params.region);
        // A bottom contact also overlaps sparse and interface slabs, so it collects them for modulation.
        const bool bottom_contact_pass = role == SurfaceRole::Bottom && &pass == &passes.front();
        Internal::modulate(paths, z, overlaps(reinterpret_cast<const object_handle *>(&object), z, bottom, bottom_contact_pass), grid.resolution);
        out.append_move_from(paths);
    }
    // A part printed as base joins the support bucket and the support extruder, on this role's own
    // surface; otherwise the role keeps its interface bucket and extruder.
    const bool as_base = role == SurfaceRole::Sparse || (role == SurfaceRole::Top ? top_as_base : bottom_as_base);
    const raw_extrusion_role bucket = as_base ? RAW_EXTRUSION_ROLE_SUPPORT : RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE;
    const int tool = as_base ? config.support_material_extruder.value : config.support_material_interface_extruder.value;
    for (ExtrusionEntityUPtr &entity : out.children()) {
        layer_region_island_handle *region = nullptr;
        const Polylines lines = to_polylines(entity->as_polylines());
        for (uint32_t island_index = 0; island_index < layer_count_island(layer); ++island_index) {
            layer_island_handle *island = layer_get_island_mutable(layer, island_index);
            const ExPolygon &slice = *reinterpret_cast<const ExPolygon *>(layer_island_get_slice(island));
            if (intersection_pl(lines, slice).empty()) continue;
            region = layer_island_get_or_create_region_island(island, nullptr, 0, tool > 0 ? tool - 1 : EXTRUDER_ID_UNKNOWN);
            break;
        }
        if (!region) throw std::runtime_error("Cannot assign support path to an island");
        extrusion_entity_handle *destination = layer_region_island_get_mutable_extrusion(region, bucket);
        if (!destination) throw std::runtime_error("Cannot access support extrusion destination");
        if (extrusion_insert_child_move(destination, extrusion_child_count(destination), reinterpret_cast<extrusion_entity_handle *>(entity.get())) == EXTRUSION_INDEX_INVALID)
            throw std::runtime_error("Cannot publish support extrusion");
    }
}
} // namespace

// Visit the owned native tree and split each path against lower footprints; empty overlap lists leave out intact and geometry errors are not suppressed.
void Internal::modulate(ExtrusionEntityCollection &out, coord_t z, const std::vector<Overlap> &overlapping, coord_t resolution)
{
    if (overlapping.empty()) return;
    for (ExtrusionEntityUPtr &entity : out.children())
        modulate_entity(*entity, z, overlapping, resolution);
}

// Choose the filler pattern of one surface part; a part printed as base uses the base pattern where
// the native generator merged it into the base layer and the intermediate rectilinear filler
// otherwise, and FillSupportBase rejects full-infill and bridging parameters, so a bridging contact
// that is not printed as base falls back to rectilinear.
InfillPattern Internal::surface_pattern(bool as_base, bool contact, bool bridge, bool base_filler, InfillPattern contact_pattern, InfillPattern base_pattern)
{
    const InfillPattern pattern = as_base ? base_filler ? base_pattern : ipRectilinear : !contact ? ipRectilinear : contact_pattern;
    return bridge && pattern == ipSupportBase ? ipRectilinear : pattern;
}

// Walk the callback object's marked layers through the C API and append this role's paths; per-call parameters isolate concurrent objects, and invalid payloads fail visibly.
void generate_paths(const plugin_run_context *context, SurfaceRole role)
{
    const run_ctx_support_toolpath *payload = plugin_ctx_as_support_toolpath(context);
    if (!payload || !payload->object || !payload->print) throw std::runtime_error("Missing support toolpath payload");
    const Slic3r::PrintObject &object = *reinterpret_cast<const Slic3r::PrintObject *>(payload->object);
    if (object_count_auxiliary_layer(payload->object) == 0) return;
    const PathParams params(object);
    for (uint32_t layer_index = 0; layer_index < object_count_auxiliary_layer(payload->object); ++layer_index) {
        throw_if_cancelled(context);
        layer_handle *layer = object_get_auxiliary_layer(payload->object, layer_index);
        if (!plugin_property_has(layer_get_properties(layer), PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT)) continue;
        process_layer(object, params, role, layer);
    }
}
}
