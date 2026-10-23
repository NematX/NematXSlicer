///|/ Copyright (c) Prusa Research 2016-2023 Vojtech Bubnik and contributors.
///|/ Copyright (c) SuperSlicer 2023 Remi Durand.
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "GridSupportGenerator.hpp"

#include <agg/agg_path_storage.h>
#include <agg/agg_pixfmt_gray.h>
#include <agg/agg_rasterizer_scanline_aa.h>
#include <agg/agg_renderer_scanline.h>
#include <agg/agg_scanline_p.h>
#include <algorithm>
#include <cfloat>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/PrintHelpers.hpp"
#include "libslic3r/Plugins/Support/Common/ClipperResults.hpp"
#include "libslic3r/Plugins/Support/Common/SupportSettings.hpp"
#include "libslic3r/Plugins/Support/Common/SupportInterfaces.hpp"
#include "libslic3r/Plugins/Support/Common/SupportZPlacement.hpp"

/* A contact is a dense slab next to the object; a column is its projection
   towards the bed. Grid support rasterizes columns into small AGG pixels and
   fills larger grid cells without crossing the object mask. Snug support uses
   closing and smoothing instead. Z slabs divide the distance between contacts, a placement
   shared with the tree family (Common/SupportZPlacement.hpp); interfaces take portions of
   base slabs near contacts. All geometry here is temporary and belongs to
   GeneratedSupport. Demand already includes painting,
   modifiers, bridge removal and the buildplate-only filter, so it follows one
   projection path rather than a separate enforced-support path. With
   build-plate-only support the contacts and columns are also cut by the area
   the object covers below them. Region raw slices supply the object footprint;
   subtracting the upper layer finds exposed tops including their perimeter
   strips. Typed fill surfaces live in island work regions and supply bridge
   footprints. height_block is the downward Z span used for object collisions,
   independently of printable height.

   The object is read through the data-tree views and every polygon set is a
   StoredPolygonCollection in the run's storage. Clipper operations take
   ClipperOperand adapters (ClipperContext) and give their result back as a
   new collection:

       // native: Polygons touching = intersection(top_surfaces, projection);
       StoredPolygonCollection touching = polygons(clipper_intersection(
           run.clipper(top_surfaces), run.clipper(projection)));

   The native ExPolygons booleans (diff_ex, intersection_ex, union_ex) drop
   small rings and unite the rest again; Common::native_expolygons() does the
   same. The AGG raster works on plain point copies. */

namespace slic3r_api::Support::Grid {
namespace {
using Polys = StoredPolygonCollection;
using ExPolys = StoredExPolygonCollection;
constexpr clipper_join_type_t join_type = CLIPPER_JOIN_SQUARE;

// The views, parameters and object settings every phase reads, gathered once per run.
struct Run {
    Run(const plugin_run_context *context, const Print &print, const Object &object, const Common::SupportParams &params,
        storage_handle *storage);
    const plugin_run_context *context;
    const Config print_config;
    const Object &object;
    const Common::SupportParams &p;
    storage_handle *storage;
    ClipperContext clipper;
    // The object layers, bottom first.
    std::vector<Layer> layers;
    Common::SupportStyle style;
    Common::ContactDistanceType distance_type;
    int32_t threshold, enforce_layers, interface_layers, bottom_interface_layers;
    bool synchronize, bottom_interface_enabled;
    // Build-plate-only support: support may stand only on the bed, so contacts and columns are cut by the area the object covers below.
    bool buildplate_only;
    c_float_or_percent layer_height, interface_layer_height;
    double angle, closing_radius, spacing;
};
struct Grid {
    const Run &run;
    const Polys &source, &mask;
    double angle, width, closing_radius, pixel_size = 0;
    c_bounding_box bbox = bounding_box_empty();
    int size_x = 0, size_y = 0;
    Polys rotated_source, rotated_mask;
    std::vector<unsigned char> pixels;
    // Prepare raster cells for grid support, or retain contours for snug support.
    Grid(const Run &run, const Polys &source, const Polys &mask);
    // Extract expanded print contours or slightly shrunken propagation contours.
    Polys extract(coord_t offset, bool fill_holes) const;
};
// Materialize a Clipper result as a flat polygon collection, like the native Polygons booleans and offsets.
Polys polygons(ClipperOperand &&result);
// Append the contours and holes of expolygons to dst, like the native polygons_append(Polygons, ExPolygons).
void append_polygons(Polys &dst, const ExPolygonCollection &expolygons);
// Return the flat polygons of expolygons, like the native to_polygons(ExPolygons).
Polys to_polygons(storage_handle *storage, const ExPolygonCollection &expolygons);
// Rotate every polygon around the origin, like the native polygons_rotate().
void rotate(Polys &polys, double angle);
// Bounding box of every point, like the native get_extents(Polygons); empty polys give an empty box.
c_bounding_box extents(const Polys &polys);
// Order points by x, then y, like the native Point operator<.
bool point_less(c_point a, c_point b);
// Return, for each object layer, the area covered by the layers below it; build-plate-only contacts and columns must not stand on it.
std::vector<Polys> buildplate_covered(const Run &run);
// Build and merge top-contact slabs from the sliced object's island demand.
void top_contacts(const Run &run, const std::vector<Polys> &covered, Common::GeneratedSupport &out);
// Project each top contact downwards and create bottom contacts on object top surfaces.
std::vector<Polys> project_columns(const Run &run, const std::vector<Polys> &covered, Common::GeneratedSupport &out);
// Place intermediate slabs between contact extremes and assign missing top-contact heights.
void intermediate_layers(const Run &run, Common::GeneratedSupport &out);
// Subtract object slices and bridge footprints from slabs with the configured clearance.
void trim_by_object(const Run &run, Common::GeneratorLayers &layers);
// Fill intermediate slabs from projected columns and remove portions inside contacts.
void base_layers(const Run &run, const std::vector<Polys> &areas, Common::GeneratedSupport &out);
// Rasterize polygons into a zero-bordered grayscale image using AGG's winding fill.
std::vector<unsigned char> rasterize(int size_x, int size_y, double pixel, c_point origin, const Polys &polys);
// Chain raster boundary edges into scaled contours, optionally closing single-pixel holes.
Polys contours(const Grid &grid, coord_t offset, bool fill_holes);
} // namespace

// Run geometric phases on a sliced object with demand; the result owns every slab and the object receives no layers or extrusions here.
void generate_support(const plugin_run_context *context, const Print &print, const Object &object,
                      const Common::SupportParams &params, Common::GeneratedSupport &out)
{
    const Run run(context, print, object, params, out.storage);
    const std::vector<Polys> covered = run.buildplate_only ? buildplate_covered(run) : std::vector<Polys>();
    top_contacts(run, covered, out);
    if (out.top.empty()) return;
    const std::vector<Polys> areas = project_columns(run, covered, out);
    intermediate_layers(run, out);
    trim_by_object(run, out.top);
    base_layers(run, areas, out);
    for (Common::GeneratorLayer *top : out.top)
        for (const Common::GeneratorLayer *bottom : out.bottom)
            if (bottom->bottom_print_z() <= top->bottom_z && top->print_z <= bottom->print_z)
                top->polygons = polygons(clipper_diff(run.clipper(top->polygons), run.clipper(bottom->polygons)));
    Common::carve_interfaces(params, run.style == Common::SupportStyle::Snug, out);
    // The foot of the support is widened, as in every family. The support ignores the brim and the skirt.
    Common::expand_first_layer_foot(run.object, run.layers, params, out);
}

namespace {
Run::Run(const plugin_run_context *context, const Print &print, const Object &object, const Common::SupportParams &params,
         storage_handle *storage)
    : context(context), print_config(print.config()), object(object), p(params), storage(storage), clipper(storage)
{
    for (uint32_t id = 0; id < object.layer_count(); ++id)
        layers.push_back(object.layer(id));
    const Config config = object.config();
    style = Common::support_style(config);
    distance_type = Common::contact_distance_type(config);
    threshold = config.get("support_material_threshold").get_int();
    enforce_layers = config.get("support_material_enforce_layers").get_int();
    interface_layers = config.get("support_material_interface_layers").get_int();
    const ConfigOption bottom_interfaces = config.get("support_material_bottom_interface_layers");
    // Index -1 asks whether the option as a whole is enabled, like the native is_enabled().
    bottom_interface_enabled = config_option_is_enabled(bottom_interfaces.handle(), -1) != 0;
    bottom_interface_layers = bottom_interfaces.get_int();
    synchronize = config.get("support_material_synchronize_layers").get_bool();
    buildplate_only = config.get("support_material_buildplate_only").get_bool();
    layer_height = config.get("support_material_layer_height").get_float_or_percent();
    interface_layer_height = config.get("support_material_interface_layer_height").get_float_or_percent();
    angle = config.get("support_material_angle").get_float();
    closing_radius = config.get("support_material_closing_radius").get_float();
    spacing = config.get("support_material_spacing").get_float();
}

Polys polygons(ClipperOperand &&result)
{
    return result.to_polygon_collection();
}

void append_polygons(Polys &dst, const ExPolygonCollection &expolygons)
{
    for (const ExPolygon expolygon : expolygons) {
        dst.push_back(expolygon.contour());
        for (const Polygon hole : expolygon.holes())
            dst.push_back(hole);
    }
}

Polys to_polygons(storage_handle *storage, const ExPolygonCollection &expolygons)
{
    Polys out(storage);
    append_polygons(out, expolygons);
    return out;
}

void rotate(Polys &polys, double angle)
{
    for (uint32_t idx = 0; idx < polys.size(); ++idx)
        multipoint_rotate_origin(polygon_as_multipoint(polygons_at(polys.mutable_handle(), idx)), angle);
}

c_bounding_box extents(const Polys &polys)
{
    c_bounding_box box = bounding_box_empty();
    for (const Polygon polygon : polys)
        if (!polygon.empty())
            bounding_box_merge(box, polygon.bounding_box());
    return box;
}

bool point_less(c_point a, c_point b)
{
    return a.x < b.x || (a.x == b.x && a.y < b.y);
}

// Accumulate the layer slices from the bottom up, each grown by 0.01 mm, so that entry id covers every object part below layer id.
std::vector<Polys> buildplate_covered(const Run &run)
{
    const ClipperContext &clipper = run.clipper;
    std::vector<Polys> covered;
    covered.reserve(run.layers.size());
    covered.emplace_back(run.storage);
    for (size_t id = 1; id < run.layers.size(); ++id) {
        throw_if_cancelled(run.context);
        covered.push_back(polygons(clipper_union2(clipper(covered.back()), clipper_offset(clipper(run.layers[id - 1].slices()), scale_d(0.01)))));
    }
    return covered;
}

// Intersect demand with each region, apply the active lower-slice margin, and rasterize contacts; allocated slabs remain in out even when merging empties them.
void top_contacts(const Run &run, const std::vector<Polys> &covered, Common::GeneratedSupport &out)
{
    const Common::SupportParams &p = run.p;
    const ClipperContext &clipper = run.clipper;
    const coord_t first_z = scale_to_layer_coord(p.first_layer_height);
    for (size_t id = 1; id < run.layers.size(); ++id) {
        throw_if_cancelled(run.context);
        const Layer &layer = run.layers[id];
        const Layer &lower = run.layers[id - 1];
        ClipperOperand demand = clipper();
        for (uint32_t island = 0; island < layer.island_count(); ++island)
            demand += clipper(layer.island(island).support_demand());
        if (demand.empty()) continue;
        const Polys lower_polygons = to_polygons(run.storage, lower.slices());
        Polys overhangs(run.storage), contacts(run.storage), margin(run.storage);
        float no_interface_offset = FLT_MAX;
        for (uint32_t region = 0; region < layer.region_count(); ++region)
            no_interface_offset = std::min(no_interface_offset,
                float(layer.region(region).flow(raw_extrusion_role(RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER)).width));
        const double threshold = run.threshold > 0 ? PI * double(run.threshold + 1) / 180. : 0.;
        float cached_offset = -1;
        for (uint32_t region_idx = 0; region_idx < layer.region_count(); ++region_idx) {
            const LayerRegion region = layer.region(region_idx);
            const ExPolys raw = Common::native_expolygons(clipper_intersection(demand, clipper(region.slices())));
            if (raw.empty()) continue;
            append_polygons(overhangs, raw);
            const coord_t lower_offset = id < size_t(run.enforce_layers) ? 0 :
                threshold > 0 ? scale_to_layer_coord(unscaled(lower.height()) / std::tan(threshold)) :
                region.flow(raw_extrusion_role(RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER)).width / 2;
            const float offset = float(std::min(lower_offset, p.gap_xy));
            if (offset != cached_offset) {
                cached_offset = offset;
                if (offset == 0)
                    margin.copy_from(lower_polygons);
                else
                    margin = polygons(clipper_offset2(clipper(lower.slices()), -no_interface_offset * 0.5f,
                        offset + no_interface_offset * 0.5f, join_type, 0.));
                // Build-plate-only support: contacts stay off the object below too, so the covered area joins the margin.
                if (run.buildplate_only && !covered[id].empty()) {
                    margin = polygons(clipper_union2(clipper(margin), clipper(covered[id])));
                    margin.ensure_valid(p.resolution);
                }
            }
            // The active margin removes thin lower features before expanding the collision mask.
            ExPolys kept = Common::native_expolygons(clipper_diff(clipper(raw), clipper(margin)));
            kept.ensure_valid(p.resolution);
            contacts = polygons(clipper_union2(clipper(contacts), clipper(kept)));
        }
        if (contacts.empty() && overhangs.empty()) continue;
        const Common::TopContactSlabs slabs = Common::add_top_contacts(run.layers, run.print_config, p, id, out);
        if (!slabs.contact) continue;
        Common::GeneratorLayer &contact = *slabs.contact;
        const Grid grid(run, contacts, margin);
        contact.contact_polygons.emplace(grid.extract(-3, true));
        contact.contact_polygons->ensure_valid(p.resolution);
        if (run.style == Common::SupportStyle::Grid && !p.soluble) {
            Polys dense = polygons(clipper_diff(clipper(overhangs), clipper_offset2(clipper(lower_polygons),
                -(no_interface_offset * 0.5f), no_interface_offset * 1.1f, join_type, 0.)));
            if (!dense.empty()) {
                dense = polygons(clipper_diff(clipper_offset(clipper(dense), no_interface_offset * 0.1f), clipper(margin)));
                const Polys trimmed = polygons(clipper_intersection(clipper(dense), clipper(*contact.contact_polygons)));
                contact.polygons = Grid(run, trimmed, margin).extract(p.base_flow.spacing / 2 + 5, false);
            }
        } else
            contact.polygons = grid.extract(p.base_flow.spacing / 2 + 5, true);
        contact.polygons.ensure_valid(p.resolution);
        contact.overhang_polygons.emplace(std::move(overhangs));
        if (slabs.bridge) {
            Common::GeneratorLayer &bridge = *slabs.bridge;
            bridge.polygons = contact.polygons.readonly().clone(run.storage);
            bridge.contact_polygons.emplace(contact.contact_polygons->readonly().clone(run.storage));
            bridge.overhang_polygons.emplace(contact.overhang_polygons->readonly().clone(run.storage));
        }
    }
    Common::merge_contacts(p, out);
}

// Project a sliced object's owned contacts into one area per object layer; bottom collision spans use the next object height for plane gaps or initial contact height otherwise, unchanged by Z alignment; cancellation propagates.
std::vector<Polys> project_columns(const Run &run, const std::vector<Polys> &covered, Common::GeneratedSupport &out)
{
    const Common::SupportParams &p = run.p;
    const ClipperContext &clipper = run.clipper;
    std::vector<Polys> areas;
    areas.reserve(run.layers.size());
    for (size_t id = 0; id < run.layers.size(); ++id) areas.emplace_back(run.storage);
    Polys projection(run.storage);
    int contact_idx = int(out.top.size()) - 1;
    for (int id = int(run.layers.size()) - 2; id >= 0; --id) {
        throw_if_cancelled(run.context);
        const Layer &layer = run.layers[id];
        for (; contact_idx >= 0 && out.top[contact_idx]->print_z >= layer.print_z(); --contact_idx) {
            Common::GeneratorLayer &top = *out.top[contact_idx];
            Polys polygons_above(run.storage);
            polygons_above.move_from(*top.contact_polygons);
            polygons_above.append_move_from(polygons(clipper_offset(clipper(*top.overhang_polygons), double(SCALED_EPSILON))));
            projection.append_move_from(polygons(clipper_union(clipper(polygons_above))));
        }
        if (projection.empty()) continue;
        projection = polygons(clipper_union(clipper(projection)));
        projection.ensure_valid(p.resolution);
        const Polys top_surfaces = polygons(clipper_diff(clipper(layer.slices()), clipper(run.layers[id + 1].slices())));
        Polys touching = polygons(clipper_intersection(clipper(top_surfaces), clipper(projection)));
        touching.ensure_valid(p.resolution);
        if (!touching.empty()) {
            const Common::GeneratorLayer &contact = Common::add_bottom_contact(run.layers, p, run.distance_type, size_t(id), touching, top_surfaces, out);
            touching = polygons(clipper_offset(clipper(touching), double(SCALED_EPSILON)));
            for (size_t above = size_t(id + 1); above < run.layers.size() && run.layers[above].print_z() < contact.print_z; ++above)
                areas[above] = polygons(clipper_diff(clipper(areas[above]), clipper(touching)));
        }
        // Build-plate-only support may stand only on the bed: the buildplate-only demand plugin already removed the demand above the object, and the columns are cut by all object area below, so none lands on the object.
        const Polys mask = run.buildplate_only ? covered[id].readonly().clone(run.storage) :
            polygons(clipper_offset(clipper(layer.slices()), double(SCALED_EPSILON)));
        Polys raw = polygons(clipper_diff(clipper(projection), clipper(mask)));
        raw.remove_sticks();
        raw.remove_degenerate();
        const Grid grid(run, raw, mask);
        areas[id] = grid.extract(p.base_flow.spacing / 2 + 5, true);
        projection = grid.extract(-3, true);
    }
    std::reverse(out.bottom.begin(), out.bottom.end());
    trim_by_object(run, out.bottom);
    return areas;
}

// Build the grid's slab spacing from its settings and place the intermediate slabs; the middle is spaced uniformly,
// or synchronized with the object layers. The borrowed run supplies nozzle-based heights; out owns the appended slabs and allocation failures propagate.
void intermediate_layers(const Run &run, Common::GeneratedSupport &out)
{
    const Common::SupportParams &p = run.p;
    const coord_t base_step = Common::slab_step(run.layer_height, p.base_flow_mm.nozzle_diameter, p.max_support_height);
    const coord_t interface_step = Common::slab_step(run.interface_layer_height, p.interface_flow_mm.nozzle_diameter, p.max_support_height);
    Common::SlabSpacing spacing;
    spacing.first_z = scale_to_layer_coord(p.first_layer_height);
    spacing.min_height = p.min_height;
    spacing.machine_step = scale_to_layer_coord(p.z_step);
    spacing.base_step = base_step;
    spacing.interface_step = interface_step;
    spacing.interface_layers = std::max(0, run.interface_layers);
    spacing.bottom_interface_layers = run.bottom_interface_enabled ? run.bottom_interface_layers : std::max(0, run.interface_layers);
    spacing.soluble = p.soluble;
    spacing.synchronize = run.synchronize;
    Common::place_intermediate_slabs(spacing, run.layers, out);
}

// Remove object silhouettes within the slab's gap-expanded Z range, including thick bridge fills and perimeter footprints above it; slab geometry is replaced in place.
void trim_by_object(const Run &run, Common::GeneratorLayers &layers)
{
    const Common::SupportParams &p = run.p;
    const ClipperContext &clipper = run.clipper;
    for (Common::GeneratorLayer *support : layers) {
        if (support->polygons.empty()) continue;
        ExPolys trimming(run.storage);
        size_t id = 0;
        const coord_t lower = support->print_z - support->height_block - scale_to_layer_coord(p.gap_below);
        while (id < run.layers.size() && run.layers[id].print_z() <= lower) ++id;
        const coord_t upper = support->print_z + scale_to_layer_coord(p.gap_above);
        for (; id < run.layers.size(); ++id) {
            const Layer &layer = run.layers[id];
            if (layer.bottom_z() >= upper) break;
            trimming.append_move_from(clipper_offset(clipper(layer.slices()), double(p.gap_xy), join_type, 0.).to_expolygon_collection());
        }
        trimming.append_move_from(Common::bridge_footprint(run.storage, run.layers, run.print_config, p, upper, *support));
        trimming.ensure_valid(p.resolution);
        const ExPolys united = Common::native_expolygons(clipper_union(clipper(trimming)));
        support->polygons = polygons(clipper_diff(clipper(support->polygons), clipper(united)));
        support->polygons.ensure_valid(p.resolution);
    }
}

// Select the cached projection at each slab's top, subtract enclosing contacts, and trim against the object; base slabs stay owned by out.
void base_layers(const Run &run, const std::vector<Polys> &areas, Common::GeneratedSupport &out)
{
    const ClipperContext &clipper = run.clipper;
    for (Common::GeneratorLayer *base : out.base) {
        int below = -1;
        for (size_t id = 0; id < run.layers.size() && run.layers[id].print_z() <= base->print_z; ++id) below = int(id);
        Polys polys = (below < 0 ? areas.front() : areas[below]).readonly().clone(run.storage);
        if (below < 0)
            for (const Common::GeneratorLayer *top : out.top)
                if (top->bottom_z >= base->print_z && top->print_z <= run.layers[0].print_z()) polys.append_copy_from(top->polygons);
        Polys trimming(run.storage);
        for (const Common::GeneratorLayer *top : out.top) {
            assert(!(base->print_z > top->bottom_z + SCALED_EPSILON && base->bottom_z < top->bottom_z - SCALED_EPSILON));
            if (top->bottom_z < base->print_z && top->print_z > base->bottom_z &&
                base->print_z <= top->print_z && base->bottom_z >= top->bottom_z) trimming.append_copy_from(top->polygons);
        }
        for (const Common::GeneratorLayer *bottom : out.bottom) {
            assert(!(base->print_z > bottom->print_z + SCALED_EPSILON && base->bottom_z < bottom->print_z - SCALED_EPSILON));
            if (bottom->bottom_print_z() < base->print_z && bottom->print_z > base->bottom_print_z() &&
                base->print_z <= bottom->print_z && base->bottom_z >= bottom->bottom_print_z()) trimming.append_copy_from(bottom->polygons);
        }
        base->polygons = trimming.empty() ? std::move(polys) :
            polygons(clipper_diff_with_safety_offset(clipper(polys), clipper(trimming)));
        base->polygons.ensure_valid(run.p.resolution);
        base->type = Common::LayerType::Base;
    }
    trim_by_object(run, out.base);
}

// Map scaled polygon coordinates to grayscale pixels; holes follow polygon winding and the returned vector owns the complete raster.
std::vector<unsigned char> rasterize(int size_x, int size_y, double pixel, c_point origin, const Polys &polys)
{
    std::vector<unsigned char> data(size_x * size_y);
    agg::rendering_buffer buffer(data.data(), unsigned(size_x), unsigned(size_y), size_x);
    agg::pixfmt_gray8 pixels(buffer);
    agg::renderer_base<agg::pixfmt_gray8> base(pixels);
    agg::renderer_scanline_aa_solid<agg::renderer_base<agg::pixfmt_gray8>> renderer(base);
    renderer.color(agg::pixfmt_gray8::color_type(255));
    base.clear(agg::pixfmt_gray8::color_type(0));
    agg::scanline_p8 scanline;
    agg::rasterizer_scanline_aa<> rasterizer;
    for (const Polygon polygon : polys) {
        const std::vector<c_point> points = polygon.points();
        if (points.empty()) continue;
        agg::path_storage path;
        const c_point &first = points.front();
        path.move_to((first.x - origin.x) / pixel, (first.y - origin.y) / pixel);
        for (size_t i = 1; i < points.size(); ++i)
            path.line_to((points[i].x - origin.x) / pixel, (points[i].y - origin.y) / pixel);
        path.line_to((first.x - origin.x) / pixel, (first.y - origin.y) / pixel);
        rasterizer.add_path(std::move(path));
    }
    agg::render_scanlines(rasterizer, scanline, renderer);
    return data;
}

// Rotate grid inputs, oversample each support cell, and propagate occupied pixels while avoiding the eroded object mask; input contours must outlive this temporary grid.
Grid::Grid(const Run &run, const Polys &src, const Polys &trim)
    : run(run), source(src), mask(trim), angle(PI * run.angle / 180.), width(run.p.base_flow_mm.spacing),
      closing_radius(run.closing_radius), rotated_source(run.storage), rotated_mask(run.storage)
{
    if (run.style != Common::SupportStyle::Grid || source.empty()) return;
    rotated_source.copy_from(source);
    rotated_mask.copy_from(mask);
    if (angle != 0) { rotate(rotated_source, -angle); rotate(rotated_mask, -angle); }
    const double spacing = run.spacing + width;
    bbox = extents(rotated_source);
    bbox.min = bbox.min - c_point{20, 20};
    bbox.max = bbox.max + c_point{20, 20};
    bbox.min = c_point{align_to_grid(bbox.min.x, scale_i(spacing)), align_to_grid(bbox.min.y, scale_i(spacing))};
    const coord_t extrusion = scale_i(width);
    const int oversampling = std::clamp(int(scale_i(spacing) / (extrusion + 100)), 1, 8);
    pixel_size = double(std::max(extrusion + 21, scale_i(spacing / oversampling)));
    bbox.min = bbox.min - c_point{coord_t(pixel_size), coord_t(pixel_size)};
    bbox.max = bbox.max + c_point{coord_t(pixel_size), coord_t(pixel_size)};
    const int raw_x = int(std::ceil(double(bbox.max.x - bbox.min.x - 1) / pixel_size));
    const int raw_y = int(std::ceil(double(bbox.max.y - bbox.min.y - 1) / pixel_size));
    const int blocks_x = (raw_x + oversampling - 3) / oversampling, blocks_y = (raw_y + oversampling - 3) / oversampling;
    size_x = blocks_x * oversampling + 2;
    size_y = blocks_y * oversampling + 2;
    assert(size_x >= raw_x && size_y >= raw_y);
    pixels = rasterize(size_x, size_y, pixel_size, bbox.min, rotated_source);
    const std::vector<unsigned char> trimming = rasterize(size_x, size_y, pixel_size, bbox.min, rotated_mask);
    std::vector<unsigned char> dilated(trimming.size(), 0);
    for (int r = 1; r + 1 < size_y; ++r)
        for (int col = 1; col + 1 < size_x; ++col) {
            bool inside = true;
            for (int y = -1; y <= 1; ++y)
                for (int x = -1; x <= 1; ++x) inside = inside && trimming[col + x + (r + y) * size_x] != 0;
            dilated[col + r * size_x] = inside;
        }
    for (int r = 0; r < blocks_y; ++r)
        for (int col = 0; col < blocks_x; ++col) {
            const int origin = col * oversampling + 1 + (r * oversampling + 1) * size_x;
            // Two sweeps spread support in each cell, stopping at the eroded object mask.
            const auto propagate = [&](int address, int offset) {
                address += origin;
                if (pixels[address + offset] && !dilated[address] && !dilated[address + offset]) pixels[address] = 1;
            };
            for (int y = 0; y < oversampling; ++y) {
                if (y > 0) for (int x = 0; x < oversampling; ++x) propagate(y * size_x + x, -size_x);
                for (int x = 1; x < oversampling; ++x) propagate(y * size_x + x, -1);
                for (int x = oversampling - 2; x >= 0; --x) propagate(y * size_x + x, 1);
            }
            for (int y = oversampling - 2; y >= 0; --y) {
                for (int x = 0; x < oversampling; ++x) propagate(y * size_x + x, size_x);
                for (int x = 1; x < oversampling; ++x) propagate(y * size_x + x, -1);
                for (int x = oversampling - 2; x >= 0; --x) propagate(y * size_x + x, 1);
            }
        }
}

// Convert the zero-bordered raster to closed contours; offset must be less than half a pixel and adjusts only corners so repeated projection does not grow columns.
Polys contours(const Grid &grid, coord_t offset, bool fill_holes)
{
    assert(std::abs(2 * offset) < grid.pixel_size - 10);
    struct Edge { c_point a, b; };
    std::vector<unsigned char> pixels = grid.pixels;
    const int stride = grid.size_x;
    if (fill_holes)
        for (int r = 1; r + 1 < grid.size_y; ++r)
            for (int c = 1; c + 1 < grid.size_x; ++c) {
                const int addr = r * stride + c;
                if ((grid.pixels[addr - 1] && grid.pixels[addr + 1]) || (grid.pixels[addr - stride] && grid.pixels[addr + stride])) pixels[addr] = 1;
            }
    std::vector<Edge> lines;
    std::vector<std::pair<c_point, int>> starts;
    for (int r = 1; r < grid.size_y; ++r)
        for (int c = 1; c < grid.size_x; ++c) {
            const int addr = r * stride + c;
            const bool left = pixels[addr - 1] != 0, top = pixels[addr - stride] != 0, current = pixels[addr] != 0;
            if (left != current) {
                lines.push_back(left ? Edge{{c, r + 1}, {c, r}} : Edge{{c, r}, {c, r + 1}});
                starts.emplace_back(lines.back().a, int(lines.size()) - 1);
            }
            if (top != current) {
                lines.push_back(top ? Edge{{c, r}, {c + 1, r}} : Edge{{c + 1, r}, {c, r}});
                starts.emplace_back(lines.back().a, int(lines.size()) - 1);
            }
        }
    const auto point_order =
        [](const std::pair<c_point, int> &a, const std::pair<c_point, int> &b) { return point_less(a.first, b.first); };
    std::sort(starts.begin(), starts.end(), point_order);
    std::vector<char> processed(lines.size(), false);
    std::vector<std::vector<c_point>> rings;
    for (int candidate = 0; candidate < int(lines.size()); ++candidate) {
        if (processed[candidate]) continue;
        std::vector<c_point> ring;
        processed[candidate] = true;
        ring.push_back(lines[candidate].b);
        int current = candidate;
        bool closed = false;
        while (!closed) {
            const auto range = std::equal_range(starts.begin(), starts.end(), std::make_pair(lines[current].b, 0), point_order);
            assert(range.first != range.second);
            int next = -1;
            for (auto it = range.first; it != range.second; ++it) {
                if (it->second == candidate) { closed = true; break; }
                if (processed[it->second]) continue;
                if (next == -1) next = it->second;
                else {
                    const c_point a = lines[current].b - lines[current].a, b = lines[it->second].b - lines[it->second].a;
                    if (int64_t(a.x) * b.y - int64_t(b.x) * a.y > 0) { next = it->second; break; }
                }
            }
            if (closed) break;
            assert(next >= 0);
            processed[next] = true;
            current = next;
            ring.push_back(lines[current].b);
        }
        // The native generator stores each ring as a moved Polygon while its points are still grid
        // cells, and that move drops a last point lying within SCALED_EPSILON / 2 of the first one.
        if (ring.size() > 1 && std::abs(ring.front().x - ring.back().x) < SCALED_EPSILON / 2 &&
            std::abs(ring.front().y - ring.back().y) < SCALED_EPSILON / 2)
            ring.pop_back();
        rings.push_back(std::move(ring));
    }
    Polys out(grid.run.storage);
    for (std::vector<c_point> &ring : rings) {
        for (c_point &point : ring)
            point = c_point{coord_t(point.x * grid.pixel_size), coord_t(point.y * grid.pixel_size)} + grid.bbox.min;
        std::vector<c_point> corners;
        for (size_t i = 0; i < ring.size(); ++i) {
            const c_point v = ring[(i + 1) % ring.size()] - ring[i == 0 ? ring.size() - 1 : i - 1];
            if (v.x != 0 && v.y != 0) {
                c_point point = ring[i];
                point.y += v.x < 0 ? -offset : offset;
                point.x += v.y > 0 ? -offset : offset;
                corners.push_back(point);
            }
        }
        StoredPolygon polygon(grid.run.storage);
        if (!corners.empty()) polygon.insert_array(0, corners.data(), uint32_t(corners.size()));
        out.push_back_move(std::move(polygon));
    }
    return out;
}

// Extract grid contours, trim by the object and retain only islands containing source samples; snug contours instead undergo closing and outward smoothing.
Polys Grid::extract(coord_t offset, bool fill_holes) const
{
    const ClipperContext &clipper = run.clipper;
    if (source.empty()) return Polys(run.storage);
    if (run.style == Common::SupportStyle::Snug) {
        Polys closed = polygons(clipper_offset2(clipper(source), scale_d(closing_radius), -scale_d(closing_radius), join_type, 0.));
        closed.smooth_outward(coord_t(scale_d(width)));
        return closed;
    }
    const Polys rings = contours(*this, offset, fill_holes);
    const ExPolys islands = Common::native_expolygons(clipper_diff(clipper(rings), clipper(rotated_mask)));
    const ExPolys sample_source = offset > 0 ? Common::native_expolygons(clipper_union(clipper(rotated_source))) :
        Common::native_expolygons(clipper_intersection(clipper(rotated_source), clipper(islands)));
    std::vector<c_point> samples;
    for (const ExPolygon polygon : sample_source) {
        const Polys shrunk = polygons(clipper_offset(clipper(polygon), -20.f));
        for (const Polygon contour : shrunk)
            if (!contour.empty()) {
                const std::vector<c_point> points = contour.points();
                const int count = int(points.size()), stride = count / std::min(count, 4);
                for (int i = 0; i < count; i += stride) samples.push_back(points[i]);
                break;
            }
    }
    std::sort(samples.begin(), samples.end(), point_less);
    Polys result(run.storage);
    for (const ExPolygon island : islands) {
        const c_bounding_box bounds = island.contour().bounding_box();
        const auto begin = std::lower_bound(samples.begin(), samples.end(), bounds.min - c_point{1, 1}, point_less);
        const auto end = std::upper_bound(samples.begin(), samples.end(), bounds.max + c_point{1, 1}, point_less);
        for (auto sample = begin; sample != end; ++sample)
            if (sample->x >= bounds.min.x && sample->x <= bounds.max.x && sample->y >= bounds.min.y && sample->y <= bounds.max.y &&
                island.contains(*sample)) {
                result.push_back(island.contour());
                for (const Polygon hole : island.holes()) result.push_back(hole);
                break;
            }
    }
    if (angle != 0) rotate(result, angle);
    return result;
}

} // namespace
} // namespace slic3r_api::Support::Grid
