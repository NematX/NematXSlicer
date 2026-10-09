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
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Api/plugin/cpp/VolumeViews.hpp"
#include "libslic3r/Plugins/Support/Common/ClipperResults.hpp"
#include "libslic3r/Plugins/Support/Common/SupportSettings.hpp"

/* A contact is a dense slab next to the object; a column is its projection
   towards the bed. Grid support rasterizes columns into small AGG pixels and
   fills larger grid cells without crossing the object mask. Snug support uses
   closing and smoothing instead. Z slabs divide the distance between contacts;
   interfaces take portions of base slabs near contacts. All geometry here is
   temporary and belongs to GeneratedSupport. Demand already includes painting,
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
    int32_t threshold, enforce_layers, interface_layers, bottom_interface_layers, raft_layers;
    bool synchronize, bottom_interface_enabled;
    // Build-plate-only support: support may stand only on the bed, so contacts and columns are cut by the area the object covers below.
    bool buildplate_only;
    c_float_or_percent layer_height, interface_layer_height;
    double angle, closing_radius, spacing, raft_first_layer_expansion, brim_width, brim_width_interior;
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
// Reverse the point order of every polygon, like the native polygons_reverse().
void reverse(Polys &polys);
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
// Carve dense top and bottom interface regions from the sparse base slabs.
void interface_layers(const Run &run, Common::GeneratedSupport &out);
// Expand column feet in several steps while avoiding object and brim areas.
void expand_feet(const Run &run, Common::GeneratedSupport &out);
// Rasterize polygons into a zero-bordered grayscale image using AGG's winding fill.
std::vector<unsigned char> rasterize(int size_x, int size_y, double pixel, c_point origin, const Polys &polys);
// Chain raster boundary edges into scaled contours, optionally closing single-pixel holes.
Polys contours(const Grid &grid, coord_t offset, bool fill_holes);
// Detect bridge extrusion paths so thicker object bridges get a separate contact Z.
bool has_bridges(const Run &run, const Layer &layer);
// Detect bridge paths with the nearest attributes in a generic extrusion tree.
bool bridging_fills(const ExtrusionEntity &entity, const EPropertyAttributes *attributes = nullptr);
// Collect the XY footprint of overhang perimeter paths, enlarged by the object clearance.
void bridge_areas(const Run &run, const ExtrusionEntity &entity, coord_t gap, ExPolys &out,
                  const EPropertyAttributes *attributes = nullptr);
// Height of the bridges of a layer region, the native LayerRegion::bridging_height_avg_mm().
double bridging_height_avg_mm(const Run &run, const Layer &layer, const LayerRegion &region);
// Whether the object has a brim under its first layer, the native PrintObject::has_brim().
bool has_brim(const Run &run);
// Allocate a slab owned by the result's deque and initialize its integer Z dimensions.
Common::GeneratorLayer &add_layer(Common::GeneratedSupport &out, Common::LayerType type, coord_t z, coord_t bottom, coord_t height);
// Merge close top contacts into the lower contact, preserving the first-layer thickness.
void merge_contacts(const Run &run, Common::GeneratorLayers &layers);
// Return the number of equal steps needed to cover a positive distance.
size_t ceil_steps(coord_t distance, coord_t step);
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
    interface_layers(run, out);
    expand_feet(run, out);
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
    raft_layers = config.get("raft_layers").get_int();
    synchronize = config.get("support_material_synchronize_layers").get_bool();
    buildplate_only = config.get("support_material_buildplate_only").get_bool();
    layer_height = config.get("support_material_layer_height").get_float_or_percent();
    interface_layer_height = config.get("support_material_interface_layer_height").get_float_or_percent();
    angle = config.get("support_material_angle").get_float();
    closing_radius = config.get("support_material_closing_radius").get_float();
    spacing = config.get("support_material_spacing").get_float();
    raft_first_layer_expansion = config.get("raft_first_layer_expansion").get_float();
    brim_width = config.get("brim_width").get_float();
    brim_width_interior = config.get("brim_width_interior").get_float();
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

void reverse(Polys &polys)
{
    for (uint32_t idx = 0; idx < polys.size(); ++idx)
        multipoint_reverse(polygon_as_multipoint(polygons_at(polys.mutable_handle(), idx)));
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
        coord_t z = layer.bottom_z() - scale_to_layer_coord(p.gap_above);
        coord_t bottom = z, height = 0;
        if (p.soluble) {
            height = lower.height();
            bottom = id == 1 ? 0 : run.layers[id - 2].print_z();
        } else {
            if (z < first_z) continue;
            if (z < first_z + p.min_height) { z = first_z; bottom = 0; height = first_z; }
        }
        Common::GeneratorLayer &contact = add_layer(out, Common::LayerType::TopContact, z, bottom, height);
        contact.object_above = id;
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
        out.top.push_back(&contact);
        if (!p.soluble && has_bridges(run, layer)) {
            coord_t bridge_height = layer.height();
            if (run.distance_type == Common::ContactDistanceType::Filament) {
                double total = 0;
                for (uint32_t region = 0; region < layer.region_count(); ++region)
                    total += bridging_height_avg_mm(run, layer, layer.region(region));
                bridge_height = scale_to_layer_coord(total / layer.region_count());
            }
            coord_t bridge_z = layer.print_z() - bridge_height - scale_to_layer_coord(p.gap_above);
            if (bridge_z >= first_z && bridge_z < z) {
                if (bridge_z < first_z + p.min_height) bridge_z = first_z;
                Common::GeneratorLayer &bridge = add_layer(out, Common::LayerType::TopContact, bridge_z,
                    bridge_z == first_z ? 0 : bridge_z, bridge_z == first_z ? first_z : 0);
                bridge.object_above = id;
                bridge.polygons = contact.polygons.readonly().clone(run.storage);
                bridge.contact_polygons.emplace(contact.contact_polygons->readonly().clone(run.storage));
                bridge.overhang_polygons.emplace(contact.overhang_polygons->readonly().clone(run.storage));
                out.top.push_back(&bridge);
            }
        }
    }
    merge_contacts(run, out.top);
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
            const coord_t height_block = run.distance_type == Common::ContactDistanceType::Plane ?
                run.layers[id + 1].height() : (p.soluble ? layer.upper_layer().height() : scale_to_layer_coord(p.bottom_flow_mm.height));
            const coord_t bottom = layer.print_z() + scale_to_layer_coord(p.gap_below);
            const coord_t z = p.soluble ? layer.upper_layer().print_z() : bottom + height_block;
            Common::GeneratorLayer &contact = add_layer(out, Common::LayerType::BottomContact, z, bottom, z - bottom);
            contact.height_block = height_block;
            contact.object_below = size_t(id);
            contact.bridging = !p.soluble;
            contact.polygons = touching.readonly().clone(run.storage);
            if (p.bottom_expansion.value > 0)
                contact.polygons = polygons(clipper_intersection(clipper(top_surfaces), clipper_offset(clipper(touching),
                    scale_d(c_float_or_percent_get_effective_value(&p.bottom_expansion, p.bottom_flow_mm.width)), join_type, 0.)));
            if (!p.soluble) {
                for (size_t idx = size_t(std::max(0, contact_idx)); idx < out.top.size() && out.top[idx]->print_z <= z + p.min_height; ++idx) {
                    const coord_t thickness = out.top[idx]->print_z - contact.bottom_z;
                    if (out.top[idx]->print_z >= z - p.min_height && thickness >= p.min_height &&
                        thickness <= scale_to_layer_coord(p.max_support_height)) {
                        contact.print_z = out.top[idx]->print_z;
                        contact.height = contact.print_z - contact.bottom_z;
                        assert(contact.height >= p.min_height);
                        break;
                    }
                }
            }
            contact.polygons.ensure_valid(p.resolution);
            out.bottom.push_back(&contact);
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

// Divide owned contact intervals into printable slabs, sharing synchronized heights and rounding inserted boundaries to the machine step; only contacts below the first-layer height acquire collision spans, and incompatible thicknesses assert.
void intermediate_layers(const Run &run, Common::GeneratedSupport &out)
{
    const Common::SupportParams &p = run.p;
    const bool sync = run.synchronize;
    if (sync) {
        // Contacts at one print Z need a common printable bottom, otherwise a thinner top slab would shorten the published bottom contact's gap.
        for (Common::GeneratorLayer *top : out.top)
            for (const Common::GeneratorLayer *bottom : out.bottom)
                if (top->height == 0 && top->print_z == bottom->print_z) {
                    top->bottom_z = bottom->bottom_z;
                    top->height = bottom->height;
                }
    }
    Common::GeneratorLayers extremes = out.top;
    extremes.insert(extremes.end(), out.bottom.begin(), out.bottom.end());
    std::sort(extremes.begin(), extremes.end(), [](const Common::GeneratorLayer *a, const Common::GeneratorLayer *b) {
        return a->extreme_z() < b->extreme_z() || (a->extreme_z() == b->extreme_z() &&
            a->type == Common::LayerType::TopContact && b->type == Common::LayerType::BottomContact);
    });
    const coord_t first = scale_to_layer_coord(p.first_layer_height);
    const coord_t base_step = scale_to_layer_coord(run.layer_height.value == 0 ? p.max_support_height :
        std::min(p.max_support_height, c_float_or_percent_get_effective_value(&run.layer_height, p.base_flow_mm.nozzle_diameter)));
    const coord_t interface_step = scale_to_layer_coord(run.interface_layer_height.value == 0 ? p.max_support_height :
        std::min(p.max_support_height, c_float_or_percent_get_effective_value(&run.interface_layer_height, p.interface_flow_mm.nozzle_diameter)));
    const coord_t machine_step = scale_to_layer_coord(p.z_step);
    coord_t previous_extreme = 0;
    Common::GeneratorLayer *previous = nullptr;
    for (Common::GeneratorLayer *current : extremes) {
        coord_t end = current->extreme_z();
        if (end < first) {
            assert(current->type == Common::LayerType::TopContact);
            current->height = current->print_z;
            // The first bed contact already has its thickness and is excluded from collision-span planning.
            if (current != extremes.front() || current->bottom_z != 0)
                current->height_block = current->print_z;
            previous = current;
            previous_extreme = end;
            continue;
        }
        coord_t start = previous_extreme;
        if (start == 0) {
            if (out.base.empty() || out.base.back()->print_z < first)
                out.base.push_back(&add_layer(out, Common::LayerType::Intermediate, first, 0, first));
            start = first;
        }
        if (end <= start) { previous = current; previous_extreme = end; continue; }
        coord_t distance = end - start;
        assert(distance >= p.min_height);
        if (sync) {
            if (previous && previous->type == Common::LayerType::TopContact && current->type == Common::LayerType::BottomContact &&
                previous->print_z == current->print_z && previous->bottom_z == current->bottom_z) {
                // This interval is the shared contact slab, not several smaller object-Z slabs; sparse columns at the same Z keep that thickness too.
                out.base.push_back(&add_layer(out, Common::LayerType::Intermediate, end, start, distance));
                previous = current;
                previous_extreme = end;
                continue;
            }
            // Soluble contacts already have a bottom on an object Z. Non-soluble contacts retain their top Z and acquire a printable thickness here.
            const bool unknown_top = current->type == Common::LayerType::TopContact && current->height == 0;
            coord_t last = start;
            for (const Layer &layer : run.layers) {
                const coord_t z = layer.print_z();
                if (z <= start) continue;
                if (z > end || (unknown_top && z == end)) break;
                if (z - last < p.min_height) continue;
                out.base.push_back(&add_layer(out, Common::LayerType::Intermediate, z, last, z - last));
                last = z;
            }
            if (unknown_top) {
                if (end - last < p.min_height && !out.base.empty() && out.base.back()->print_z == last) {
                    last = out.base.back()->bottom_z;
                    out.base.pop_back();
                }
                // Round cumulative boundaries, not one repeated thickness, so residual machine steps reach the unchanged contact Z.
                const coord_t limit = machine_step > SCALED_EPSILON ? interface_step - interface_step % machine_step : interface_step;
                const size_t count = ceil_steps(end - last, limit);
                const coord_t start_z = last;
                for (size_t i = 1; i < count; ++i) {
                    const coord_t next = check_z_step(scale_to_layer_coord(unscaled(start_z) + unscaled(end - start_z) * i / count), machine_step);
                    assert(next - last >= p.min_height && next - last <= interface_step);
                    out.base.push_back(&add_layer(out, Common::LayerType::Intermediate, next, last, next - last));
                    last = next;
                }
                current->bottom_z = last;
                current->height = end - last;
                assert(current->height >= p.min_height && current->height <= interface_step);
                end = last;
            } else if (last < end) {
                // A bottom contact keeps its gap above the object; close the interval at its top before returning to object Z positions.
                if (end - last < p.min_height && !out.base.empty() && out.base.back()->print_z == last) {
                    last = out.base.back()->bottom_z;
                    out.base.pop_back();
                }
                const coord_t limit = machine_step > SCALED_EPSILON ? base_step - base_step % machine_step : base_step;
                const size_t count = ceil_steps(end - last, limit);
                const coord_t start_z = last;
                for (size_t i = 1; i <= count; ++i) {
                    const coord_t z = i == count ? end : check_z_step(scale_to_layer_coord(unscaled(start_z) + unscaled(end - start_z) * i / count), machine_step);
                    assert(z - last >= p.min_height && z - last <= base_step);
                    out.base.push_back(&add_layer(out, Common::LayerType::Intermediate, z, last, z - last));
                    last = z;
                }
            }
        } else {
            // Distribute the interval across the requested top, bottom and sparse thicknesses, as in native support generation.
            size_t top = size_t(std::max(0, run.interface_layers));
            const int bottom_count = run.bottom_interface_enabled ? run.bottom_interface_layers : int(top);
            size_t bottom = bottom_count > 0 ? size_t(bottom_count - 1) : 0;
            size_t middle = 0;
            double step = 0, dense_step = interface_step;
            const coord_t dense_height = interface_step * coord_t(top + bottom);
            if (distance <= dense_height - EPSILON) {
                top = ceil_steps(distance, interface_step);
                dense_step = double(distance) / top;
                bottom = std::min(bottom, top / 2);
                top -= bottom;
            } else {
                middle = ceil_steps(distance - dense_height, base_step);
                step = double(distance) / (top + bottom + middle);
                if (step <= base_step && step <= interface_step) dense_step = step;
                else if (interface_step <= base_step) step = (distance - dense_step * (top + bottom)) / middle;
                else { step = base_step; dense_step = (distance - step * middle) / (top + bottom); }
            }
            size_t total = top + bottom + middle;
            if (previous && previous->type == Common::LayerType::TopContact &&
                previous->print_z + p.min_height > previous->bottom_z + scale_to_layer_coord(unscaled(step))) {
                start = previous->print_z;
                out.base.push_back(&add_layer(out, Common::LayerType::Intermediate, start, previous->bottom_z, previous->height));
                distance = end - start;
                if (distance == 0) { previous = current; previous_extreme = end; continue; }
                --total;
                if (bottom > 0) --bottom;
                else if (middle > 0) --middle;
                else --top;
                const coord_t remaining_dense = interface_step * coord_t(top + bottom);
                if (distance <= remaining_dense) {
                    top = ceil_steps(distance, interface_step);
                    dense_step = double(distance) / top;
                    middle = 0;
                    bottom = std::min(bottom, top - 1);
                    top -= bottom;
                } else {
                    middle = ceil_steps(distance - remaining_dense, base_step);
                    step = double(distance) / (top + bottom + middle);
                    if (step <= base_step && step <= interface_step) dense_step = step;
                    else if (interface_step <= base_step) { dense_step = interface_step; step = double(distance - remaining_dense) / middle; }
                    else { step = base_step; dense_step = double(remaining_dense) / (top + bottom); }
                }
                total = top + bottom + middle;
            }
            if (!p.soluble && current->type == Common::LayerType::TopContact && current->height == 0) {
                if (top > 0) { --top; current->height = scale_to_layer_coord(unscaled(dense_step)); }
                else if (middle > 0) { --middle; current->height = scale_to_layer_coord(unscaled(step)); }
                else { --bottom; current->height = scale_to_layer_coord(unscaled(dense_step)); }
                end = current->bottom_z = current->print_z - current->height;
                --total;
            }
            coord_t last = start;
            double wanted = unscaled(start);
            for (size_t i = 0; i < total; ++i) {
                coord_t z = end;
                if (i + 1 < total) {
                    const bool dense = i < bottom || i >= total - top;
                    wanted += unscaled(dense ? dense_step : step);
                    z = last + std::min(check_z_step(scale_to_layer_coord(wanted) - last,
                        scale_to_layer_coord(p.z_step)), dense ? interface_step : base_step);
                }
                assert(z > last);
                out.base.push_back(&add_layer(out, Common::LayerType::Intermediate, z, last, z - last));
                last = z;
            }
        }
        previous = current;
        previous_extreme = end;
    }
    for (const Common::GeneratorLayer *layer : out.top) assert(layer->height > 0);
}

// Remove object silhouettes within the slab's gap-expanded Z range, including thick bridge fills and perimeter footprints above it; slab geometry is replaced in place.
void trim_by_object(const Run &run, Common::GeneratorLayers &layers)
{
    const Common::SupportParams &p = run.p;
    const ClipperContext &clipper = run.clipper;
    const bool filament = run.distance_type == Common::ContactDistanceType::Filament;
    const raw_surface_type bridge_fill = srf_type().bottom().solid().bridge().raw();
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
        if (!p.soluble) {
            for (; id < run.layers.size(); ++id) {
                const Layer &layer = run.layers[id];
                bool overlaps = false;
                for (uint32_t region_idx = 0; region_idx < layer.region_count(); ++region_idx) {
                    const LayerRegion region = layer.region(region_idx);
                    const coord_t bridge_height = filament ?
                        scale_to_layer_coord(bridging_height_avg_mm(run, layer, region)) : layer.height();
                    if (layer.print_z() - bridge_height >= upper) break;
                    overlaps = true;
                    for (uint32_t island = 0; island < layer.island_count(); ++island) {
                        const LayerIsland layer_island = layer.island(island);
                        for (uint32_t work_idx = 0; work_idx < layer_island.region_island_count(); ++work_idx) {
                            const LayerRegionIsland work = layer_island.region_island(work_idx);
                            bool member = false;
                            for (uint32_t idx = 0; idx < work.region_count(); ++idx)
                                member = member || work.region(idx).handle() == region.handle();
                            if (!member) continue;
                            ExPolys bridges(run.storage);
                            for (const Surface &surface : work.fill_surfaces())
                                if (surface.type() == bridge_fill) bridges.push_back(surface.expolygon());
                            trimming.append_move_from(clipper_offset(clipper(bridges), double(p.gap_xy), join_type, 0.).to_expolygon_collection());
                        }
                    }
                }
                for (uint32_t island = 0; island < layer.island_count(); ++island) {
                    const LayerIsland layer_island = layer.island(island);
                    for (uint32_t work_idx = 0; work_idx < layer_island.region_island_count(); ++work_idx) {
                        const LayerRegionIsland work = layer_island.region_island(work_idx);
                        if (!work.has_extrusion(raw_extrusion_role(RAW_EXTRUSION_ROLE_PERIMETER))) continue;
                        bool too_high = false;
                        for (uint32_t idx = 0; idx < work.region_count(); ++idx) {
                            const coord_t bridge_height = filament ?
                                scale_to_layer_coord(bridging_height_avg_mm(run, layer, work.region(idx))) : layer.height();
                            if (layer.print_z() - bridge_height >= upper) { too_high = true; break; }
                        }
                        if (!too_high)
                            bridge_areas(run, ExtrusionEntity(work.extrusion(raw_extrusion_role(RAW_EXTRUSION_ROLE_PERIMETER))),
                                         p.gap_xy, trimming);
                    }
                }
                if (!overlaps) break;
            }
        }
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

// Project contacts through the configured number of base slabs and regularize their union; top and bottom regions are separated after union so their published surface types stay distinct.
void interface_layers(const Run &run, Common::GeneratedSupport &out)
{
    const Common::SupportParams &p = run.p;
    const ClipperContext &clipper = run.clipper;
    const bool snug = run.style == Common::SupportStyle::Snug;
    const double smoothing = p.interface_flow.spacing * 1.5;
    const double radius = p.interface_flow.spacing / p.interface_density;
    // Snug interfaces are closed then smoothed outwards; grid ones are only united with a safety offset.
    const auto regularize = [&](const Polys &polys) {
        if (!snug)
            return polygons(clipper_union_with_safety_offset(clipper(polys)));
        Polys closed = polygons(clipper_offset2(clipper(polys), smoothing + radius, -smoothing, join_type, 0.));
        closed.smooth_outward(coord_t(smoothing));
        return closed;
    };
    for (size_t i = 0; i < out.base.size(); ++i) {
        Common::GeneratorLayer &base = *out.base[i];
        Polys top(run.storage), bottom(run.storage);
        if (p.top_interfaces > 0) {
            const coord_t z = out.base[std::min(out.base.size() - 1, i + p.top_interfaces - 1)]->print_z;
            for (const Common::GeneratorLayer *contact : out.top)
                if (contact->print_z >= base.print_z && contact->bottom_z <= z)
                    top.append_copy_from(snug ? *contact->overhang_polygons : contact->polygons);
        }
        if (p.bottom_interfaces > 0) {
            const coord_t z = out.base[i + 1 > p.bottom_interfaces ? i + 1 - p.bottom_interfaces : 0]->bottom_z;
            for (const Common::GeneratorLayer *contact : out.bottom)
                if (contact->print_z >= z && contact->print_z <= base.bottom_z) bottom.append_copy_from(contact->polygons);
        }
        if (top.empty() && bottom.empty()) continue;
        Polys united = bottom.readonly().clone(run.storage);
        united.append_copy_from(top);
        const Polys regular = regularize(united);
        united = polygons(clipper_intersection(clipper(regular), clipper(base.polygons)));
        united.ensure_valid(p.resolution);
        if (united.empty()) continue;
        // When both projections meet, the regularized top projection takes the overlap and the bottom interface takes the remaining union.
        Polys top_part(run.storage);
        if (!top.empty() && bottom.empty())
            top_part = united.readonly().clone(run.storage);
        else if (!top.empty()) {
            const Polys regular_top = regularize(top);
            top_part = polygons(clipper_intersection(clipper(united), clipper(regular_top)));
        }
        Polys bottom_part = polygons(clipper_diff(clipper(united), clipper(top_part)));
        for (const std::pair<Common::LayerType, Polys *> &part : {
                std::pair<Common::LayerType, Polys *>{Common::LayerType::TopInterface, &top_part},
                std::pair<Common::LayerType, Polys *>{Common::LayerType::BottomInterface, &bottom_part}}) {
            if (part.second->empty()) continue;
            Common::GeneratorLayer &layer = add_layer(out, part.first, base.print_z, base.bottom_z, base.height);
            layer.polygons = std::move(*part.second);
            out.interfaces.push_back(&layer);
        }
        base.polygons = polygons(clipper_diff(clipper(base.polygons), clipper(united)));
        base.polygons.ensure_valid(p.resolution);
    }
}

// Enlarge the first base slab by the configured foot expansion without crossing the first object slice; subtract the configured brim from all bed-contacting slabs.
void expand_feet(const Run &run, Common::GeneratedSupport &out)
{
    const Common::SupportParams &p = run.p;
    const ClipperContext &clipper = run.clipper;
    const coord_t first = scale_to_layer_coord(p.first_layer_height);
    Polys interface(run.storage);
    for (Common::GeneratorLayers *layers : {&out.top, &out.interfaces})
        for (const Common::GeneratorLayer *layer : *layers)
            if (layer->bottom_print_z() == 0)
                interface.append_move_from(polygons(clipper_offset(clipper(layer->polygons), float(scale_d(EPSILON)), join_type, 0.)));
    if (!out.base.empty() && out.base.front()->bottom_print_z() == 0) {
        Polys &foot = out.base.front()->polygons;
        const Polys trimming = polygons(clipper_offset(clipper(run.layers[0].slices()), double(p.gap_xy), join_type, 0.));
        const double expansion = std::max(0., scale_d(run.raft_first_layer_expansion) - scale_d(EPSILON));
        if (expansion > SCALED_EPSILON) {
            const int steps = std::max(5, int(std::ceil(expansion / p.first_flow.width)));
            for (int i = 0; i < steps; ++i)
                foot = polygons(clipper_diff(clipper_offset(clipper(foot), float(expansion / steps)), clipper(trimming)));
        } else foot = polygons(clipper_diff(clipper(foot), clipper(trimming)));
        foot = polygons(clipper_diff(clipper(foot), clipper(interface)));
        foot.ensure_valid(p.resolution);
    }
    if (has_brim(run)) {
        Polys brim(run.storage);
        const double distance = scale_d(run.brim_width);
        const double round_tolerance = float(scale_d(0.1));
        for (const ExPolygon slice : run.layers[0].slices()) {
            if (run.brim_width > 0 && run.brim_width_interior > 0) brim.append_move_from(polygons(clipper_offset(clipper(slice), distance)));
            else {
                if (run.brim_width > 0)
                    brim.append_move_from(polygons(clipper_offset(clipper(slice.contour()), distance, CLIPPER_JOIN_ROUND, round_tolerance)));
                else brim.push_back(slice.contour());
                Polys holes(run.storage);
                for (const Polygon hole : slice.holes()) holes.push_back(hole);
                if (run.brim_width_interior > 0) {
                    reverse(holes);
                    holes = polygons(clipper_offset(clipper(holes), -distance, CLIPPER_JOIN_ROUND, round_tolerance));
                    reverse(holes);
                }
                brim.append_move_from(holes);
            }
        }
        brim = polygons(clipper_union(clipper(brim)));
        for (Common::GeneratorLayers *layers : {&out.top, &out.base, &out.interfaces})
            for (Common::GeneratorLayer *layer : *layers)
                if (layer->print_z <= first) layer->polygons = polygons(clipper_diff(clipper(layer->polygons), clipper(brim)));
    }
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

// Check perimeter loops and the INFILLS bucket independently; bridge fills never require a perimeter bucket to exist.
bool has_bridges(const Run &run, const Layer &layer)
{
    for (uint32_t island = 0; island < layer.island_count(); ++island) {
        const LayerIsland layer_island = layer.island(island);
        for (uint32_t work_idx = 0; work_idx < layer_island.region_island_count(); ++work_idx) {
            const LayerRegionIsland region = layer_island.region_island(work_idx);
            if (region.has_extrusion(raw_extrusion_role(RAW_EXTRUSION_ROLE_PERIMETER))) {
                ExPolys areas(run.storage);
                bridge_areas(run, ExtrusionEntity(region.extrusion(raw_extrusion_role(RAW_EXTRUSION_ROLE_PERIMETER))), 0, areas);
                if (!areas.empty()) return true;
            }
            if (region.has_extrusion(raw_extrusion_role(RAW_EXTRUSION_ROLE_INFILL)) &&
                bridging_fills(ExtrusionEntity(region.extrusion(raw_extrusion_role(RAW_EXTRUSION_ROLE_INFILL)))))
                return true;
        }
    }
    return false;
}

// Read generic tree children and inherited flow attributes, returning true for a bridge polyline without assuming a concrete path subclass.
bool bridging_fills(const ExtrusionEntity &entity, const EPropertyAttributes *attributes)
{
    if (const EPropertyAttributes *local = entity.get(EPropertyAttributes::key)) attributes = local;
    if (entity.has_polyline() && attributes && (attributes->extrusion_role() & RAW_EXTRUSION_ROLE_BRIDGE) != 0) return true;
    for (const ExtrusionEntity child : entity.children())
        if (bridging_fills(child, attributes)) return true;
    return false;
}

// Expand overhang polylines using their nearest flow attributes and append scaled XY footprints; generic nodes and descendants remain owned by the object.
// An arc in a path (a plugin may store some) is replaced by points on it, like the native ArcPolyline::to_polyline().
void bridge_areas(const Run &run, const ExtrusionEntity &entity, coord_t gap, ExPolys &out, const EPropertyAttributes *attributes)
{
    const ClipperContext &clipper = run.clipper;
    if (const EPropertyAttributes *local = entity.get(EPropertyAttributes::key)) attributes = local;
    const int32_t overhang = RAW_EXTRUSION_ROLE_OVERHANG_PERIMETER;
    if (entity.has_polyline() && attributes && (attributes->extrusion_role() & overhang) == overhang) {
        const uint32_t size = entity.point_count();
        const double expansion = scale_d(attributes->c_extrusion_property_attributes::width / 2) + gap;
        if (size >= 3 && entity.local_front() == entity.local_back()) {
            // The native Polygon built from the closed path drops its repeated last point.
            std::vector<c_point> points = entity.linear_points();
            points.pop_back();
            StoredPolygon polygon(run.storage);
            polygon.insert_array(0, points.data(), uint32_t(points.size()));
            if (polygon.area() < 0) polygon.reverse();
            out.append_move_from(Common::native_expolygons(clipper_diff(clipper_offset(clipper(polygon), expansion, join_type, 0.),
                                                                        clipper_offset(clipper(polygon), -expansion, join_type, 0.))));
        } else if (size >= 2) {
            StoredPolylineCollection path(run.storage);
            StoredPolyline polyline(run.storage);
            const std::vector<c_point> points = entity.linear_points();
            polyline.insert_array(0, points.data(), uint32_t(points.size()));
            path.push_back(polyline);
            out.append_move_from(clipper_union(clipper_offset_polylines(run.storage, path, scale_d(attributes->c_extrusion_property_attributes::width / 2) + expansion,
                                                                        join_type, 0.)).to_expolygon_collection());
        }
    }
    for (const ExtrusionEntity child : entity.children()) bridge_areas(run, child, gap, out, attributes);
}

// The three native bridge types: the averaged region nozzles, the layer height, or a round thread with the cross
// section of the region's infill flow; the last two are float flow heights in the native code.
double bridging_height_avg_mm(const Run &run, const Layer &layer, const LayerRegion &region)
{
    const Config config = region.print_region().config();
    const std::string type = config.get("bridge_type").serialize();
    const double ratio = config.get("bridge_flow_ratio").get_effective_value(1.);
    if (type == "nozzle") {
        const ConfigOption nozzle = run.print_config.get("nozzle_diameter");
        const auto nozzle_of = [&](const char *key) { return nozzle.get_float(uint32_t(config.get(key).get_int() - 1)); };
        const double average = (nozzle_of("perimeter_extruder") + nozzle_of("infill_extruder") + nozzle_of("solid_infill_extruder")) / 3.;
        return average * std::sqrt(ratio);
    }
    if (type == "height")
        return unscaled(layer.height());
    if (type == "flow") {
        const double diameter = std::sqrt(4 * region.flow(raw_extrusion_role(RAW_EXTRUSION_ROLE_INFILL)).mm3_per_mm / PI);
        return float(std::sqrt(ratio) * diameter);
    }
    throw std::invalid_argument("Unknown bridge type " + type);
}

bool has_brim(const Run &run)
{
    for (uint32_t idx = 0; idx < run.object.volume_count(); ++idx)
        if (run.object.volume(idx).type() == RAW_VOLUME_TYPE_BRIM_PATCH) return true;
    return run.brim_width > 0 && run.brim_width_interior > 0 && run.raft_layers <= 0;
}

// Append a slab to the owning deque and return its stable reference; Z dimensions must lie on the layer grid or assert, and only intermediate/base slabs start with a full-height collision span.
Common::GeneratorLayer &add_layer(Common::GeneratedSupport &out, Common::LayerType type, coord_t z, coord_t bottom, coord_t height)
{
    assert(scale_to_layer_coord(unscaled(z)) == z && scale_to_layer_coord(unscaled(height)) == height);
    out.slabs.emplace_back(out.storage);
    Common::GeneratorLayer &layer = out.slabs.back();
    layer.type = type;
    layer.print_z = z;
    layer.bottom_z = bottom;
    layer.height = height;
    // A top contact is already cleared against the lower layer; its zero collision span excludes the coplanar object layer from trimming.
    layer.height_block = type == Common::LayerType::Intermediate || type == Common::LayerType::Base ? height : 0;
    return layer;
}

// Sort contacts and union slabs closer than the minimum thickness into the lower slab without changing its collision span; moved source geometry is cleared while deque storage stays alive.
void merge_contacts(const Run &run, Common::GeneratorLayers &layers)
{
    const Common::SupportParams &p = run.p;
    const ClipperContext &clipper = run.clipper;
    std::sort(layers.begin(), layers.end(), [](const Common::GeneratorLayer *a, const Common::GeneratorLayer *b) { return a->print_z < b->print_z; });
    Common::GeneratorLayers merged;
    for (size_t i = 0; i < layers.size();) {
        Common::GeneratorLayer &target = *layers[i];
        const coord_t first = scale_to_layer_coord(p.first_layer_height);
        const bool first_group = target.print_z < first + p.min_height;
        const coord_t limit = first_group ? first + p.min_height - 1 : target.print_z + p.min_height;
        size_t j = i + 1;
        for (; j < layers.size() && layers[j]->print_z <= limit; ++j) {
            Common::GeneratorLayer &source = *layers[j];
            target.polygons = polygons(clipper_union2(clipper(target.polygons), clipper(source.polygons)));
            *target.contact_polygons = polygons(clipper_union2(clipper(*target.contact_polygons), clipper(*source.contact_polygons)));
            *target.overhang_polygons = polygons(clipper_union2(clipper(*target.overhang_polygons), clipper(*source.overhang_polygons)));
            source.reset();
        }
        if (first_group) { target.print_z = first; target.bottom_z = 0; target.height = first; }
        target.polygons.ensure_valid(p.resolution);
        merged.push_back(&target);
        i = j;
    }
    layers = std::move(merged);
}

// Compute ceiling division with the native rounding tolerance; distance and step must both be positive and the result is at least one.
size_t ceil_steps(coord_t distance, coord_t step)
{
    assert(distance > 0 && step > 0);
    const size_t count = size_t(std::ceil((double(distance) - EPSILON) / double(step)));
    assert(count > 0);
    return count;
}
} // namespace
} // namespace slic3r_api::Support::Grid
