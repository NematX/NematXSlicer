///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "DefaultSkirtGenerator.hpp"
#include "AdhesionLayerHelpers.hpp"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <iterator>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_skirt_brim.h"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/GuiRuleHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/SkirtBrimStepViews.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Layer.hpp"

/*
Default skirt generator
=======================

This plugin generates the classic skirt and first-layer skirt-brim loops from
the first-layer envelope. The geometry is built in the plugin, while auxiliary
layer storage and adhesion metadata are published through the shared API
helper. The skirt is generated after the brim so an option that places the
skirt relative to the brim can use the brim geometry that was actually built.

The normal execution flow is:

The hull is object-local for per-object skirts and print-level for shared
skirts. Draft-shield settings can extend the considered layer range, while
`min_skirt_length` may request additional loop length. The output roots are
explicitly non-sortable and non-reversible because their order is part of the
skirt's behavior. This plugin creates extrusion trees and auxiliary layers,
but does not generate final machine G-code or perform general extrusion
ordering.

Local skirt hulls use the configured height range and the selected instance's geometry. Their filled
outer boundaries, including deposited width, must not overlap foreign first-layer
geometry; rejected instances produce plugin warnings instead of auxiliary layers.
STEP_WIPETOWER runs after STEP_SKIRT_BRIM, so its generated paths are not yet available.

Operation notes:
- validate the STEP_SKIRT_BRIM context and read print settings
- return when no skirt, skirt-brim, or draft-shield work is requested
- choose per-object generation when complete-object mode requires it
- otherwise collect all print objects for one shared envelope
- collect object, support, and attributable brim points
- build the convex-hull offset loops
- resolve first-layer flow and distribute loops across extruders
- keep normal and first-layer-only loops in print order
- local_skirt_conflict(): reject local loops around foreign first-layer geometry
- publish_object_skirt() or publish_print_skirt()
- create tagged auxiliary layers through the common helper

Representative direct calls (branches are not a timeline):

    generate_default_skirt()
        |-- make_skirt()
        |-- publish_object_skirt()
        `-- publish_print_skirt()
*/

namespace slic3r_api { namespace SkirtBrim { namespace DefaultSkirtGeneratorPlugin {
// Register the permanent borrowed plugin and copy its alternative-selection setting description into the application.
// Print data is unchanged; allocation failures or rejected descriptions throw.
void register_default_skirt_generator_plugin(orchestrator_handle *orchestrator);

namespace {

const char *const k_no_dependencies[] = { nullptr };
const char *const k_group_id = "skirt_brim.skirt";
constexpr double k_rounding_overlap = 1.0 - 0.25 * PI;

const raw_used_config_key k_used_config_keys[] = {
    { "skirts", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "skirt_height", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "skirt_distance", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "skirt_distance_from_brim", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "skirt_brim", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "min_skirt_length", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "draft_shield", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "print_plan_builder", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "complete_objects_step", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "complete_objects_one_skirt", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "resolution_internal", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "first_layer_height", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "skirt_extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "first_layer_extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "first_layer_extrusion_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "perimeter_extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "perimeter_extrusion_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "extrusion_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "nozzle_diameter", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "filament_diameter", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "extrusion_multiplier", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "use_volumetric_e", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "perimeters", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "brim_width", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "brim_width_interior", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "fill_density", RAW_CO_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "top_solid_layers", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "bottom_solid_layers", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "solid_infill_every_layers", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "perimeter_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "infill_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "solid_infill_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "support_material_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "support_material_interface_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "support_material_interface_layers", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE }
};

const char *const k_defined_config_keys[] = {
    "complete_objects_one_skirt"
};

// Register the option definitions owned by this plugin, then the activation
// rules that decide when they stay editable. Both run once from
// inilialize_impl().
void initialize_default_skirt_generator_settings(orchestrator_handle *orchestrator);
void add_default_skirt_generator_gui_rules(orchestrator_handle *orchestrator);

struct SkirtOutput
{
    StoredExtrusionEntity skirt;
    StoredExtrusionEntity first_layer;
    StoredPolygonCollection convex_hull_points;

    explicit SkirtOutput(storage_handle *storage) :
        skirt(storage),
        first_layer(storage),
        convex_hull_points(storage)
    {
        /*
        The generated loop order is meaningful: the host should not sort these
        transport roots before G-code planning sees them.
        */
        skirt.disable_sort().disable_reverse();
        first_layer.disable_sort().disable_reverse();
    }
};

// A placed area and its box let distant pairs skip polygon intersections.
struct SkirtArea
{
    Slic3r::ExPolygons polygons;
    Slic3r::BoundingBox box;
};

// Cache each physical copy's filled skirt envelope, deposited skirt, slices and attributable auxiliary geometry.
struct LocalSkirtGeometry
{
    SkirtArea envelope;
    SkirtArea skirt;
    SkirtArea body;
    SkirtArea auxiliary;
};

// Return true when config values request any skirt-like output.
bool has_skirt_work(const Config &print_config);

// Return true for draft-shield mode where the skirt is intentionally tall.
bool has_infinite_skirt(const Config &print_config);

// Resolve the width/spacing settings exactly as the classic skirt flow did.
c_float_or_percent effective_skirt_width_option(const Config &print_config);
c_float_or_percent effective_skirt_spacing_option(const Config &print_config);
c_flow make_skirt_flow_from_options(c_float_or_percent width_option,
                                    c_float_or_percent spacing_option,
                                    double nozzle_diameter,
                                    double height);
c_flow skirt_flow(const Print &print, extruder_index_t extruder_id);

// Copy one point range into a normal std::vector used as the plugin point cloud.
void append_points_from_polygon(std::vector<c_point> &points, const Polygon &polygon);

// Add every point from an extrusion tree; empty optional trees are ignored.
void append_points_from_extrusion(std::vector<c_point> &points, const ExtrusionEntity &entity);

bool append_points_from_print_brim_auxiliary_layers(std::vector<c_point> &points, const Print &print);

// Append borrowed object and attributable support contours up to the requested height in object coordinates; instance-specific brim is added separately, output points belong to the caller and geometry/allocation failures propagate.
void collect_object_local_hull_points(std::vector<c_point> &object_points,
                                      storage_handle *storage,
                                      const SkirtBrimStep &step,
                                      const Print &print,
                                      const Object &object,
                                      coord_t skirt_height_z,
                                      std::optional<uint32_t> instance_idx = std::nullopt);

// Build a storage-owned hull over the configured height range in plate coordinates or one selected instance's local coordinates; borrowed layers stay unchanged and invalid-owner/geometry/allocation failures propagate.
StoredPolygon collect_print_hull(storage_handle *storage,
                                 const SkirtBrimStep &step,
                                 const Print &print,
                                 const std::vector<Object> &objects,
                                 bool object_local,
                                 uint32_t instance_idx = 0);

// Build one skirt loop entity from an offset polygon and flow.
StoredExtrusionEntity make_skirt_loop(storage_handle *storage,
                                      const Polygon &loop,
                                      const c_flow &flow,
                                      coord_t min_first_layer_height);

// Generate all skirt and first-layer-only loops around one convex hull.
SkirtOutput make_skirt(storage_handle *storage,
                       const SkirtBrimStep &step,
                       const Print &print,
                       const std::vector<Object> &objects,
                        bool object_local,
                        uint32_t instance_idx = 0);

// Return a conflict reason, or empty for this pass: null eligible checks global geometry and other instances' slices/brim/support.
// With eligible, check only those other instances' skirts accepted by the first pass.
std::string local_skirt_conflict(const std::vector<std::vector<LocalSkirtGeometry>> &geometry,
                                const SkirtArea &global, uint32_t object_idx, uint32_t instance_idx,
                                const std::vector<std::vector<bool>> *eligible = nullptr);

// Project slices and auxiliary layers once and cache both filled envelopes and deposited skirt coverage.
std::vector<std::vector<LocalSkirtGeometry>> prepare_local_skirt_geometry(
    const Print &print, const std::vector<std::vector<SkirtOutput>> &outputs, SkirtArea &global);

// Collect the deposited width of normal and first-layer-only skirt loops without filling their holes.
Slic3r::ExPolygons local_skirt_coverage(const SkirtOutput &output);

// Place and merge owned polygons, optionally fill their holes, and compute the resulting box.
SkirtArea make_skirt_area(Slic3r::ExPolygons polygons, c_point shift = {}, bool fill_holes = false);

// Test cached polygons only when their defined boxes overlap.
bool skirt_envelope_overlaps(const SkirtArea &envelope, const SkirtArea &other);

// Publish one skirt tree, optionally limited to a physical instance, using its actual deposited coverage.
bool publish_skirt_tree(storage_handle *storage, orchestrator_handle *orchestrator, const Print &print,
                        const Object &object, const Layer &reference_layer, StoredExtrusionEntity &extrusion,
                        raw_layer_adhesion_flag flags, std::optional<uint32_t> owner_instance = std::nullopt);

// Publish print-owned or object-owned skirt results as tagged auxiliary layers.
void publish_print_skirt(storage_handle *storage,
                         orchestrator_handle *orchestrator,
                         const Print &print,
                         const std::vector<Object> &objects,
                         SkirtOutput &output);
// Publish local skirt trees, optionally limited to one physical instance.
void publish_object_skirt(storage_handle *storage,
                          orchestrator_handle *orchestrator,
                          const Print &print,
                          const Object &object,
                          SkirtOutput &output,
                          std::optional<uint32_t> owner_instance = std::nullopt);

// Generate shared skirts or validate local candidates; report rejected objects and instances in one warning.
void generate_default_skirt(const plugin_run_context *run_ctx, orchestrator_handle *orchestrator);

bool has_skirt_work(const Config &print_config)
{
    return (print_config.get("skirt_height").get_int() > 0 && print_config.get("skirts").get_int() > 0) ||
           has_infinite_skirt(print_config) ||
           print_config.get("draft_shield").get_int() != 0 ||
           print_config.get("skirt_brim").get_int() > 0;
}

bool has_infinite_skirt(const Config &print_config)
{
    return print_config.get("draft_shield").get_int() == 2 && print_config.get("skirts").get_int() > 0;
}

c_float_or_percent effective_skirt_width_option(const Config &print_config)
{
    ConfigOption option = print_config.get("skirt_extrusion_width");
    c_float_or_percent out = option.get_float_or_percent();

    /*
    A one-layer draft shield uses the first-layer width before falling back to
    the normal perimeter/extrusion width chain. That keeps the shield printable
    with the same first-layer tuning as the rest of the adhesion geometry.
    */
    if (out.value == 0.0 && print_config.float_or_default("first_layer_extrusion_width", 0.0) > 0.0 &&
        print_config.int_or_default("skirt_height", 0) == 1 &&
        print_config.enum_or_default("draft_shield", 0) != 0)
        out = print_config.get("first_layer_extrusion_width").get_float_or_percent();

    if (out.value == 0.0 && print_config.has("perimeter_extrusion_width"))
        out = print_config.get("perimeter_extrusion_width").get_float_or_percent();
    if (out.value == 0.0 && print_config.has("extrusion_width"))
        out = print_config.get("extrusion_width").get_float_or_percent();
    return out;
}

c_float_or_percent effective_skirt_spacing_option(const Config &print_config)
{
    ConfigOption option = print_config.get("skirt_extrusion_width");
    c_float_or_percent out = option.get_float_or_percent();

    /*
    Spacing has its own first-layer fallback. Width and spacing can be tuned
    independently, so the plugin resolves both options before building the
    rounded-rectangle flow used by skirt paths.
    */
    if (out.value == 0.0 && print_config.float_or_default("first_layer_extrusion_spacing", 0.0) > 0.0 &&
        print_config.int_or_default("skirt_height", 0) == 1 &&
        print_config.enum_or_default("draft_shield", 0) != 0)
        out = print_config.get("first_layer_extrusion_spacing").get_float_or_percent();

    if (out.value == 0.0 && print_config.has("perimeter_extrusion_spacing"))
        out = print_config.get("perimeter_extrusion_spacing").get_float_or_percent();
    if (out.value == 0.0 && print_config.has("extrusion_spacing"))
        out = print_config.get("extrusion_spacing").get_float_or_percent();
    return out;
}

c_flow make_skirt_flow_from_options(const c_float_or_percent width_option,
                                    const c_float_or_percent spacing_option,
                                    const double nozzle_diameter,
                                    const double height)
{
    if (height <= 0.0)
        throw std::runtime_error("Cannot compute skirt flow without a positive first-layer height.");

    double width = (!width_option.percent && width_option.value == 0.0) ?
        1.125 * nozzle_diameter :
        c_float_or_percent_get_effective_value(&width_option, nozzle_diameter);
    double spacing = width - height * k_rounding_overlap;

    /*
    A percentage width equal to zero means "auto width". If the user provided a
    concrete spacing instead, the classic flow keeps that spacing and derives
    the physical width needed to obtain it.
    */
    if (width_option.percent && width_option.value == 0.0 && spacing_option.value != 0.0) {
        spacing = c_float_or_percent_get_effective_value(&spacing_option, nozzle_diameter);
        width = spacing + height * k_rounding_overlap;
    }

    if (spacing <= 0.0 || width <= 0.0)
        throw std::runtime_error("Skirt flow produced a non-positive width or spacing.");

    c_flow out = {};
    out.width = scale_i(width);
    out.spacing = scale_i(spacing);
    out.height = scale_to_layer_coord(height);
    out.nozzle_diameter = scale_i(nozzle_diameter);
    out.spacing_ratio = 1.0f;
    out.is_bridge = 0;
    out.mm3_per_mm = height * (width - height * k_rounding_overlap);
    return out;
}

c_flow skirt_flow(const Print &print, const extruder_index_t extruder_id)
{
    const Config print_config = print.config();

    /*
    The skirt is a print-level object. The classic code used the largest nozzle
    that can participate in first-layer printing, so all skirt loops share one
    conservative width even when several objects or supports use different
    extruders.
    */
    double max_nozzle_diameter = 0.0;
    for (uint32_t object_idx = 0; object_idx < print.object_count(); ++object_idx) {
        const Object object = print.object(object_idx);
        const std::set<extruder_index_t> extruders = object_extruders(print, object);
        for (extruder_index_t object_extruder : extruders)
            max_nozzle_diameter = std::max(
                max_nozzle_diameter,
                print_config.vector_float_or_default("nozzle_diameter", object_extruder, 0.4));
    }
    if (max_nozzle_diameter <= 0.0)
        max_nozzle_diameter = print_config.vector_float_or_default("nozzle_diameter", extruder_id, 0.4);

    return make_skirt_flow_from_options(
        effective_skirt_width_option(print_config),
        effective_skirt_spacing_option(print_config),
        max_nozzle_diameter,
        unscaled(get_min_first_layer_height(print)));
}

void append_points_from_polygon(std::vector<c_point> &points, const Polygon &polygon)
{
    const std::vector<c_point> polygon_points = polygon.points();
    points.insert(points.end(), polygon_points.begin(), polygon_points.end());
}

void append_points_from_extrusion(std::vector<c_point> &points, const ExtrusionEntity &entity)
{
    if (entity.empty())
        return;
    const std::vector<c_point> extrusion_points = entity.collect_points();
    points.insert(points.end(), extrusion_points.begin(), extrusion_points.end());
}

void append_points_from_brim_layer(std::vector<c_point> &points, const Layer &layer, bool &found)
{
    for (uint32_t island_idx = 0; island_idx < layer.island_count(); ++island_idx) {
        const LayerIsland island = layer.island(island_idx);
        for (uint32_t region_island_idx = 0; region_island_idx < island.region_island_count();
             ++region_island_idx) {
            const LayerRegionIsland region_island = island.region_island(region_island_idx);
            if (!region_island.has_extrusion(RAW_EXTRUSION_ROLE_PERIMETER))
                continue;
            append_points_from_extrusion(
                points,
                ExtrusionEntity(region_island.extrusion(RAW_EXTRUSION_ROLE_PERIMETER)));
            found = true;
        }
    }
}

bool append_points_from_print_brim_auxiliary_layers(std::vector<c_point> &points, const Print &print)
{
    bool found = false;
    const Object auxiliary_object = print.auxiliary_object();
    for (uint32_t layer_idx = 0; layer_idx < auxiliary_object.auxiliary_layer_count(); ++layer_idx) {
        const Layer layer = auxiliary_object.auxiliary_layer(layer_idx);
        if (layer_is_brim(layer))
            append_points_from_brim_layer(points, layer, found);
    }
    return found;
}

void collect_object_local_hull_points(std::vector<c_point> &object_points,
                                      storage_handle *storage,
                                      const SkirtBrimStep &step,
                                      const Print &print,
                                      const Object &object,
                                      const coord_t skirt_height_z,
                                      const std::optional<uint32_t> instance_idx)
{
    /*
    The skirt hull is based on visible outer contours only. Holes are ignored:
    the skirt is outside the object/support envelope, so internal voids cannot
    shrink the required boundary.
    */
    for (uint32_t layer_idx = 0; layer_idx < object.layer_count(); ++layer_idx) {
        const Layer layer = object.layer(layer_idx);
        if (layer.print_z() > skirt_height_z)
            break;
        for (ExPolygon expolygon : layer.slices())
            append_points_from_polygon(object_points, expolygon.contour());
    }

    /*
    Support layers are treated like object geometry for skirt placement. They
    are already generated before STEP_SKIRT_BRIM in the new pipeline, so no
    support-specific host shortcut is needed here.
    */
    for (uint32_t layer_idx = 0; layer_idx < object.auxiliary_layer_count(); ++layer_idx) {
        const Layer support_layer = object.auxiliary_layer(layer_idx);
        if (support_layer.properties().get(LayerSupportProperty::key) == nullptr)
            continue;
        const std::optional<uint32_t> owner = layer_instance_owner(support_layer);
        if (owner && instance_idx && *owner != *instance_idx)
            continue;
        if (support_layer.print_z() > skirt_height_z)
            break;
        for (ExPolygon expolygon : support_layer.slices())
            append_points_from_polygon(object_points, expolygon.contour());
    }

    if (print.config().get("skirt_distance_from_brim").get_bool()) {
        /*
        The common object/support envelope keeps its configured brim-distance
        allowance. Instance-owned generated brim is added separately by the
        caller so one copy's local points never become another copy's geometry.
        */
        StoredPolygon object_polygon(storage);
        for (c_point point : object_points)
            object_polygon.push_back(point);
        if (object_polygon.size() >= 3) {
            const double brim_width = object.config().get("brim_width").get_float();
            const ClipperContext clipper(storage);
            const StoredPolygon object_hull = convex_hull(storage, object_polygon.readonly());
            const StoredPolygonCollection widened =
                clipper_offset(clipper(object_hull.readonly()), scale_d(brim_width), CLIPPER_JOIN_ROUND,
                               double(std::max<coord_t>(SCALED_EPSILON, skirt_flow(print, 0).width / 10)))
                    .to_polygon_collection();
            for (Polygon polygon : widened)
                append_points_from_polygon(object_points, polygon);
        }
    }
}

StoredPolygon collect_print_hull(storage_handle *storage,
                                 const SkirtBrimStep &step,
                                 const Print &print,
                                 const std::vector<Object> &objects,
                                 const bool object_local,
                                 const uint32_t selected_instance)
{
    const Config print_config = print.config();
    coord_t skirt_height_z = 0;
    for (Object object : objects) {
        if (object.layer_count() == 0)
            continue;
        const uint32_t layer_count = has_infinite_skirt(print_config) ?
            object.layer_count() :
            std::min<uint32_t>(uint32_t(std::max(1, print_config.get("skirt_height").get_int())),
                               object.layer_count());
        skirt_height_z = std::max(skirt_height_z, object.layer(layer_count - 1).print_z());
    }

    std::vector<c_point> points;
    for (Object object : objects) {
        std::vector<c_point> object_points;
        collect_object_local_hull_points(object_points, storage, step, print, object, skirt_height_z,
                                         object_local ? std::optional<uint32_t>(selected_instance) : std::nullopt);

        /*
        Slices are object-local. A print-level skirt must cover every object
        instance, so it translates the local hull by each instance shift. An
        object-owned skirt is stored in object coordinates, so it deliberately
        keeps a single origin copy and lets the object instance transform apply
        later.
        */
        if (object_points.size() >= 3) {
            StoredPolygon local_polygon(storage);
            for (c_point point : object_points)
                local_polygon.push_back(point);
            StoredPolygon local_hull = convex_hull(storage, local_polygon.readonly());
            object_points = local_hull.points();
        }

        const uint32_t instance_count = object_local ? 1 : object.instance_count();
        for (uint32_t instance_idx = 0; instance_idx < instance_count; ++instance_idx) {
            const c_point shift = object_local ? c_point{} : object.instance_shift(instance_idx);
            for (c_point point : object_points) {
                point.x += shift.x;
                point.y += shift.y;
                points.push_back(point);
            }
            if (print_config.get("draft_shield").get_int() == 0 || print_config.get("skirt_distance_from_brim").get_bool()) {
                for (uint32_t layer_idx = 0; layer_idx < object.auxiliary_layer_count(); ++layer_idx) {
                    const Layer brim_layer = object.auxiliary_layer(layer_idx);
                    if (!layer_is_brim(brim_layer))
                        continue;
                    if (object_local) {
                        const std::optional<uint32_t> owner = layer_instance_owner(brim_layer);
                        if (owner && *owner != selected_instance) continue;
                        bool found = false;
                        append_points_from_brim_layer(points, brim_layer, found);
                        continue;
                    }
                    for (const std::pair<uint32_t, c_point> &projection : layer_instance_projections(object, brim_layer)) {
                        if (projection.first != (object_local ? selected_instance : instance_idx))
                            continue;
                        std::vector<c_point> local;
                        bool found = false;
                        append_points_from_brim_layer(local, brim_layer, found);
                        const c_point brim_shift = object_local ? c_point{} : projection.second;
                        for (c_point point : local) {
                            point.x += brim_shift.x;
                            point.y += brim_shift.y;
                            points.push_back(point);
                        }
                    }
                }
            }
        }
    }

    if (!object_local && (print_config.get("draft_shield").get_int() == 0 || print_config.get("skirt_distance_from_brim").get_bool())) {
        append_points_from_print_brim_auxiliary_layers(points, print);
    }

    StoredPolygon point_cloud(storage);
    for (c_point point : points)
        point_cloud.push_back(point);
    return point_cloud.size() < 3 ? StoredPolygon(storage) : convex_hull(storage, point_cloud.readonly());
}

StoredExtrusionEntity make_skirt_loop(storage_handle *storage,
                                      const Polygon &loop,
                                      const c_flow &flow,
                                      const coord_t min_first_layer_height)
{
    std::vector<c_point> points = loop.points();
    if (points.empty())
        return StoredExtrusionEntity(storage);
    points.push_back(points.front());

    StoredExtrusionEntity out(storage);
    if (!out.set_points(points) || out.point_count() != points.size())
        throw std::runtime_error("Default skirt generator could not create a skirt loop polyline.");
    out.disable_sort().disable_reverse();
    EPropertyAttributes &attributes = out.get_or_add(EPropertyAttributes::key);
    attributes.extrusion_role(RAW_EXTRUSION_ROLE_SKIRT)
        .mm3_per_mm(flow.mm3_per_mm)
        .width(float(unscaled(flow.width)))
        .height(float(unscaled(min_first_layer_height)));
    out.get_or_add(EPropertyPerimeter::key)
        .shell_count(0)
        .perimeter_flags(C_EXTRUSION_PERIMETER_FLAG_LOOP | C_EXTRUSION_PERIMETER_FLAG_SKIRT);

    if (loop.is_clockwise())
        out.reverse();
    return out;
}

SkirtOutput make_skirt(storage_handle *storage,
                       const SkirtBrimStep &step,
                       const Print &print,
                       const std::vector<Object> &objects,
                        const bool object_local,
                        const uint32_t instance_idx)
{
    SkirtOutput output(storage);
    const Config print_config = print.config();
    StoredPolygon convex_hull_polygon = collect_print_hull(storage, step, print, objects, object_local, instance_idx);
    if (convex_hull_polygon.size() < 3)
        return output;

    std::vector<extruder_index_t> extruders;
    const std::set<extruder_index_t> extruder_set = collect_print_extruders_for_skirt(print, objects);
    extruders.insert(extruders.end(), extruder_set.begin(), extruder_set.end());
    if (extruders.empty())
        extruders.push_back(0);

    uint32_t n_skirts = uint32_t(std::max(0, print_config.get("skirts").get_int()));
    const uint32_t n_skirts_first_layer = n_skirts + uint32_t(std::max(0, print_config.get("skirt_brim").get_int()));
    if (has_infinite_skirt(print_config) && n_skirts == 0)
        n_skirts = 1;

    const uint32_t max_loop_count = std::max(n_skirts, n_skirts_first_layer);
    if (max_loop_count == 0)
        return output;

    const c_flow last_flow = skirt_flow(print, extruders.back());
    coord_t distance = scale_i(print_config.get("skirt_distance").get_float()) - last_flow.spacing / 2;
    const coord_t min_first_layer_height = get_min_first_layer_height(print);
    const uint32_t lines_per_extruder =
        uint32_t((n_skirts + uint32_t(extruders.size()) - 1) / uint32_t(extruders.size()));
    uint32_t current_lines_per_extruder =
        n_skirts - lines_per_extruder * (uint32_t(extruders.size()) - 1);
    std::vector<double> extruded_length(extruders.size(), 0.0);

    /*
    The legacy algorithm walks from the inner line outward, then reverses the
    output so printing starts at the outermost skirt. Keeping that order makes
    min_skirt_length and multi-extruder distribution match the classic path.
    Instead of storing every temporary loop and reversing later, each new loop
    is inserted at child index zero. The final tree therefore has the same
    outer-to-inner order without keeping extra storage-owned entities alive.
    */
    for (uint32_t remaining = max_loop_count, extruder_idx = 0, nb_skirts = 1; remaining > 0; --remaining) {
        const bool first_layer_only = remaining <= (n_skirts_first_layer - n_skirts);
        const extruder_index_t extruder_id = extruders[extruders.size() - (1 + extruder_idx)];
        const c_flow flow = skirt_flow(print, extruder_id);
        distance += flow.spacing / 2;

        const ClipperContext clipper(storage);
        StoredPolygonCollection loops =
            clipper_offset(clipper(convex_hull_polygon.readonly()), double(distance), CLIPPER_JOIN_ROUND,
                           double(std::max<coord_t>(SCALED_EPSILON, flow.width / 10)))
                .to_polygon_collection();
        uint32_t loop_idx = 0;
        for (; loop_idx < loops.size(); ++loop_idx) {
            Polygon candidate = loops[loop_idx];
            if (!candidate.empty() && candidate.is_valid())
                break;
        }
        if (loop_idx == loops.size())
            break;

        Polygon loop = loops[loop_idx];
        StoredExtrusionEntity entity = make_skirt_loop(storage, loop, flow, min_first_layer_height);
        distance += flow.spacing / 2;

        if (!entity.empty()) {
            if (n_skirts_first_layer > n_skirts) {
                if (first_layer_only) {
                    const uint32_t inserted_idx = output.first_layer.insert_child_move(0, entity.mutable_view());
                    if (is_invalid_index(inserted_idx))
                        throw std::runtime_error("Default skirt generator could not add first-layer skirt loop.");
                } else {
                    StoredExtrusionEntity first_layer_entity(storage, entity.readonly());
                    const uint32_t inserted_idx = output.first_layer.insert_child_move(0, first_layer_entity.mutable_view());
                    if (is_invalid_index(inserted_idx))
                        throw std::runtime_error("Default skirt generator could not add first-layer skirt copy.");
                }
            }

            if (!first_layer_only) {
                const uint32_t inserted_idx = output.skirt.insert_child_move(0, entity.mutable_view());
                if (is_invalid_index(inserted_idx))
                    throw std::runtime_error("Default skirt generator could not add skirt loop.");
            }

            /*
            The first child insertion turns an empty entity into a generic
            collection and the host default for such collections is sortable.
            Skirt loops are intentionally ordered from outside to inside, so
            reapply the step contract after insertions have created children.
            */
            output.skirt.disable_sort().disable_reverse();
            output.first_layer.disable_sort().disable_reverse();
        }

        if (print_config.get("min_skirt_length").get_float() > 0.0 && !first_layer_only) {
            extruded_length[extruder_idx] += unscaled(loop.length()) * e_per_mm(print, extruder_id, flow.mm3_per_mm);
            if (extruded_length[extruder_idx] < print_config.get("min_skirt_length").get_float()) {
                if (remaining == 1 && extruded_length[extruder_idx] > 0.0)
                    ++remaining;
            } else if (extruder_idx + 1 < extruders.size()) {
                if (nb_skirts < current_lines_per_extruder) {
                    ++nb_skirts;
                } else {
                    current_lines_per_extruder = lines_per_extruder;
                    nb_skirts = 1;
                    ++extruder_idx;
                }
            }
        }
    }

    const c_flow hull_flow = skirt_flow(print, extruders.back());
    output.convex_hull_points =
        clipper_offset(ClipperContext(storage)(convex_hull_polygon.readonly()),
                       double(distance + hull_flow.spacing / 2), CLIPPER_JOIN_ROUND, double(scale_d(0.1)))
            .to_polygon_collection();
    return output;
}

StoredExPolygonCollection skirt_subject_from_extrusion(storage_handle *storage, const StoredExtrusionEntity &extrusion)
{
    StoredExPolygonCollection stored_subject(storage);
    const Slic3r::ExtrusionEntity *native =
        reinterpret_cast<const Slic3r::ExtrusionEntity *>(extrusion.readonly().handle());
    if (native == nullptr || native->empty())
        return stored_subject;

    const Slic3r::Polygons coverage = native->polygons_covered_by_width(float(SCALED_EPSILON));
    Slic3r::ExPolygons subject = coverage.empty() ? Slic3r::ExPolygons{} : union_ex(coverage);
    for (const Slic3r::ExPolygon &expolygon : subject)
        stored_subject.push_back(ExPolygon(reinterpret_cast<const expolygon_handle *>(&expolygon)));
    return stored_subject;
}

bool publish_skirt_tree(storage_handle *storage,
                        orchestrator_handle *orchestrator,
                        const Print &print,
                        const Object &object,
                        const Layer &reference_layer,
                        StoredExtrusionEntity &extrusion,
                        raw_layer_adhesion_flag flags,
                        std::optional<uint32_t> owner_instance)
{
    if (extrusion.empty())
        return true;

    StoredExPolygonCollection subject = skirt_subject_from_extrusion(storage, extrusion);
    if (subject.empty())
        return false;

    return publish_adhesion_extrusion_to_auxiliary_layer(storage,
                                                         orchestrator,
                                                         print,
                                                         object,
                                                         subject.readonly(),
                                                         reference_layer.height(),
                                                         reference_layer.print_z(),
                                                         reference_layer.slice_z(),
                                                         RAW_LAYER_ADHESION_KIND_SKIRT,
                                                         flags,
                                                          extrusion.mutable_view(), owner_instance);
}

Layer first_print_layer_or_throw(const std::vector<Object> &objects)
{
    for (const Object &object : objects)
        if (object.layer_count() > 0)
            return object.layer(0);
    throw std::runtime_error("Default skirt generator could not find a first object layer.");
}

void publish_print_skirt(storage_handle *storage,
                         orchestrator_handle *orchestrator,
                         const Print &print,
                         const std::vector<Object> &objects,
                         SkirtOutput &output)
{
    const Object auxiliary_object = print.auxiliary_object();
    const Layer reference_layer = first_print_layer_or_throw(objects);
    if (!publish_skirt_tree(storage, orchestrator, print, auxiliary_object, reference_layer, output.skirt, 0))
        throw std::runtime_error("Default skirt generator could not publish print skirt.");
    if (!publish_skirt_tree(storage,
                            orchestrator,
                            print,
                            auxiliary_object,
                            reference_layer,
                            output.first_layer,
                            RAW_LAYER_ADHESION_FLAG_FIRST_LAYER_ONLY))
        throw std::runtime_error("Default skirt generator could not publish first-layer skirt.");
}

void publish_object_skirt(storage_handle *storage,
                          orchestrator_handle *orchestrator,
                          const Print &print,
                          const Object &object,
                          SkirtOutput &output,
                          std::optional<uint32_t> owner_instance)
{
    if (object.layer_count() == 0)
        return;

    const Layer reference_layer = object.layer(0);
    if (!publish_skirt_tree(storage, orchestrator, print, object, reference_layer, output.skirt, 0, owner_instance))
        throw std::runtime_error("Default skirt generator could not publish object skirt.");
    if (!publish_skirt_tree(storage,
                            orchestrator,
                            print,
                            object,
                            reference_layer,
                            output.first_layer,
                            RAW_LAYER_ADHESION_FLAG_FIRST_LAYER_ONLY, owner_instance))
        throw std::runtime_error("Default skirt generator could not publish object first-layer skirt.");
}

Slic3r::ExPolygons local_skirt_coverage(const SkirtOutput &output)
{
    Slic3r::Polygons coverage;
    for (const StoredExtrusionEntity *tree : {&output.skirt, &output.first_layer}) {
        const Slic3r::ExtrusionEntity *native =
            reinterpret_cast<const Slic3r::ExtrusionEntity *>(tree->readonly().handle());
        if (native == nullptr || native->empty()) continue;
        Slic3r::Polygons part = native->polygons_covered_by_width(float(SCALED_EPSILON));
        coverage.insert(coverage.end(), std::make_move_iterator(part.begin()), std::make_move_iterator(part.end()));
    }
    return Slic3r::union_ex(coverage);
}

SkirtArea make_skirt_area(Slic3r::ExPolygons polygons, const c_point shift, const bool fill_holes)
{
    for (Slic3r::ExPolygon &area : polygons) {
        area.translate(shift.x, shift.y);
        if (fill_holes) area.holes.clear();
    }
    SkirtArea out;
    out.polygons = Slic3r::union_ex(polygons);
    for (const Slic3r::ExPolygon &area : out.polygons) out.box.merge(area.contour.points);
    return out;
}

bool skirt_envelope_overlaps(const SkirtArea &envelope, const SkirtArea &other)
{
    return envelope.box.defined && other.box.defined && envelope.box.overlap(other.box) &&
        !Slic3r::intersection_ex(envelope.polygons, other.polygons).empty();
}

std::vector<std::vector<LocalSkirtGeometry>> prepare_local_skirt_geometry(
    const Print &print, const std::vector<std::vector<SkirtOutput>> &outputs, SkirtArea &global)
{
    std::vector<std::vector<LocalSkirtGeometry>> geometry(print.object_count());
    // Global auxiliary geometry has no real-instance owner, even when it surrounds only this object.
    Slic3r::ExPolygons global_polygons;
    const Object auxiliary = print.auxiliary_object();
    for (uint32_t idx = 0; idx < auxiliary.auxiliary_layer_count(); ++idx) {
        const Layer layer = auxiliary.auxiliary_layer(idx);
        if (!layer_is_brim(layer) && layer.print_z() > get_min_first_layer_height(print)) continue;
        const Slic3r::Layer *native = reinterpret_cast<const Slic3r::Layer *>(layer.handle());
        global_polygons.insert(global_polygons.end(), native->lslices().begin(), native->lslices().end());
    }
    global = make_skirt_area(std::move(global_polygons), {}, true);

    for (uint32_t object_idx = 0; object_idx < print.object_count(); ++object_idx) {
        const Object object = print.object(object_idx);
        geometry[object_idx].resize(object.instance_count());
        if (object.layer_count() == 0) continue;
        const Slic3r::Layer *first = reinterpret_cast<const Slic3r::Layer *>(object.layer(0).handle());
        for (uint32_t copy = 0; copy < object.instance_count(); ++copy) {
            const c_point placement = object.instance_shift(copy);
            LocalSkirtGeometry &cached = geometry[object_idx][copy];
            cached.body = make_skirt_area(first->lslices(), placement);
            cached.skirt = make_skirt_area(local_skirt_coverage(outputs[object_idx][copy]), placement);
            cached.envelope = make_skirt_area(cached.skirt.polygons, {}, true);
        }
        // Enumerate a common layer's K projections once, not again inside each of the K copy comparisons.
        for (uint32_t idx = 0; idx < object.auxiliary_layer_count(); ++idx) {
            const Layer layer = object.auxiliary_layer(idx);
            if (layer.print_z() > object.layer(0).print_z()) continue;
            if (!layer_is_brim(layer) && layer.properties().get(LayerSupportProperty::key) == nullptr) continue;
            const Slic3r::Layer *native = reinterpret_cast<const Slic3r::Layer *>(layer.handle());
            for (const std::pair<uint32_t, c_point> &projection : layer_instance_projections(object, layer)) {
                Slic3r::ExPolygons placed = native->lslices();
                for (Slic3r::ExPolygon &area : placed) area.translate(projection.second.x, projection.second.y);
                Slic3r::ExPolygons &destination = geometry[object_idx][projection.first].auxiliary.polygons;
                destination.insert(destination.end(), std::make_move_iterator(placed.begin()), std::make_move_iterator(placed.end()));
            }
        }
        for (LocalSkirtGeometry &cached : geometry[object_idx])
            cached.auxiliary = make_skirt_area(std::move(cached.auxiliary.polygons));
    }
    return geometry;
}

std::string local_skirt_conflict(const std::vector<std::vector<LocalSkirtGeometry>> &geometry,
                                const SkirtArea &global, const uint32_t object_idx, const uint32_t instance_idx,
                                const std::vector<std::vector<bool>> *eligible)
{
    const SkirtArea &envelope = geometry[object_idx][instance_idx].envelope;
    if (!envelope.box.defined) return {};
    if (eligible == nullptr && skirt_envelope_overlaps(envelope, global))
        return "intersects or is nested with global first-layer geometry";
    for (uint32_t other_idx = 0; other_idx < geometry.size(); ++other_idx) {
        for (uint32_t copy = 0; copy < geometry[other_idx].size(); ++copy) {
            if (other_idx == object_idx && copy == instance_idx) continue;
            const LocalSkirtGeometry &other = geometry[other_idx][copy];
            if (eligible == nullptr) {
                if (skirt_envelope_overlaps(envelope, other.body))
                    return "crosses or encloses another object or instance";
                if (skirt_envelope_overlaps(envelope, other.auxiliary))
                    return "crosses or encloses another instance's brim or support";
                continue;
            }
            // Skirts already rejected by object geometry will never be printed and cannot block another skirt.
            if ((*eligible)[other_idx][copy] && skirt_envelope_overlaps(envelope, other.skirt))
                return "crosses or encloses another instance's skirt";
        }
    }
    return {};
}

void generate_default_skirt(const plugin_run_context *run_ctx, orchestrator_handle *orchestrator)
{
    const run_ctx_skirt_brim *ctx = plugin_ctx_as_skirt_brim(run_ctx);
    if (ctx == nullptr || ctx->print == nullptr)
        return;

    const SkirtBrimStep step(ctx);
    const Print print = step.print();
    const Config print_config = print.config();
    if (!has_skirt_work(print_config))
        return;

    storage_handle *storage = run_ctx->plugin_storage;
    // The API exposes the stored selection; with one active builder it can differ from the effective selection.
    if (print_config.get("print_plan_builder").serialize() == "ordering.plan_builder.by_object" &&
        print_config.get("complete_objects_step").get_float() == 0. &&
        !print_config.get("complete_objects_one_skirt").get_bool()) {
        std::vector<std::vector<SkirtOutput>> outputs(print.object_count());
        for (uint32_t object_idx = 0; object_idx < print.object_count(); ++object_idx) {
            throw_if_cancelled(run_ctx);
            const Object object = print.object(object_idx);
            for (uint32_t instance = 0; instance < object.instance_count(); ++instance)
                outputs[object_idx].push_back(make_skirt(storage, step, print, std::vector<Object>{ object }, true, instance));
        }
        // Record all decisions before publication; newly published skirts must not alter later decisions.
        SkirtArea global;
        const std::vector<std::vector<LocalSkirtGeometry>> geometry = prepare_local_skirt_geometry(print, outputs, global);
        std::vector<std::vector<bool>> accepted(print.object_count());
        std::vector<std::vector<std::string>> reasons(print.object_count());
        for (uint32_t object_idx = 0; object_idx < print.object_count(); ++object_idx)
            for (uint32_t instance = 0; instance < print.object(object_idx).instance_count(); ++instance) {
                throw_if_cancelled(run_ctx);
                reasons[object_idx].push_back(local_skirt_conflict(geometry, global, object_idx, instance));
                accepted[object_idx].push_back(reasons[object_idx].back().empty());
            }
        const std::vector<std::vector<bool>> eligible = accepted;
        std::string warnings;
        for (uint32_t object_idx = 0; object_idx < print.object_count(); ++object_idx) {
            const Object object = print.object(object_idx);
            for (uint32_t instance = 0; instance < object.instance_count(); ++instance) {
                throw_if_cancelled(run_ctx);
                const std::string reason = reasons[object_idx][instance].empty() ?
                    local_skirt_conflict(geometry, global, object_idx, instance, &eligible) : reasons[object_idx][instance];
                accepted[object_idx][instance] = reason.empty();
                if (!reason.empty()) {
                    warnings += "\n'" + object.name() + "', instance " +
                        std::to_string(instance + 1) + ": " + reason + ".";
                }
            }
        }
        if (!warnings.empty()) {
            const std::string message = "Local skirt omitted:" + warnings;
            report_warning(run_ctx, message.c_str());
        }
        for (uint32_t object_idx = 0; object_idx < print.object_count(); ++object_idx) {
            const Object object = print.object(object_idx);
            bool common = std::all_of(accepted[object_idx].begin(), accepted[object_idx].end(), [](bool value) { return value; });
            for (uint32_t idx = 0; idx < object.auxiliary_layer_count(); ++idx)
                common = common && !layer_instance_owner(object.auxiliary_layer(idx));
            for (uint32_t instance = 0; instance < object.instance_count(); ++instance) {
                if (!accepted[object_idx][instance]) continue;
                publish_object_skirt(storage, orchestrator, print, object, outputs[object_idx][instance],
                                     common ? std::nullopt : std::optional<uint32_t>(instance));
                if (common) break;
            }
        }
        return;
    }

    std::vector<Object> objects;
    objects.reserve(print.object_count());
    for (uint32_t object_idx = 0; object_idx < print.object_count(); ++object_idx) {
        throw_if_cancelled(run_ctx);
        objects.push_back(print.object(object_idx));
    }
    throw_if_cancelled(run_ctx);
    SkirtOutput output = make_skirt(storage, step, print, objects, false);
    throw_if_cancelled(run_ctx);
    publish_print_skirt(storage, orchestrator, print, objects, output);
    throw_if_cancelled(run_ctx);
}

void initialize_default_skirt_generator_settings(orchestrator_handle *orchestrator)
{
    /*
    Option definitions owned by this module. Each entry fixes the option type,
    override scope, default and GUI metadata; defined keys are published in the
    print presets, so profiles read and write them by name like any built-in
    setting. Called once from inilialize_impl().
    */
    raw_config_option_def def = raw_config_option_def_init();
    def.opt_key = "complete_objects_one_skirt";
    def.type = RAW_CO_BOOL;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_SKIRT_BRIM;
    def.category = RAW_OPTION_CATEGORY_OUTPUT;
    def.label = "Allow only one skirt loop";
    def.tooltip = "When using 'Complete individual objects', the default behavior is to draw the skirt around each object."
                  " if you prefer to have only one skirt for the whole platter, use this option.";
    def.mode = RAW_CONFIG_OPTION_MODE_ADV_EXP | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "0";
    orchestrator_create_option_def(orchestrator, &def);
}

void add_default_skirt_generator_gui_rules(orchestrator_handle *orchestrator)
{
    using slic3r_api::GuiCondition;
    using slic3r_api::GuiRule;

    const GuiCondition sequential = slic3r_api::string_equals("print_plan_builder", "ordering.plan_builder.by_object");
    GuiRule::activate("complete_objects_one_skirt")
        .when(sequential)
        .scope(RAW_PRESET_TYPE_FFF_PRINT)
        .add_to(orchestrator);
}

class DefaultSkirtGenerator : public PluginBase
{
public:
    static DefaultSkirtGenerator &instance(orchestrator_handle *orchestrator)
    {
        static DefaultSkirtGenerator instance(orchestrator);
        return instance;
    }

    explicit DefaultSkirtGenerator(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

private:
    const char *id_impl() const noexcept override { return "skirt_brim.skirt.default"; }
    const char *name_impl() const noexcept override { return "Default skirt generator"; }
    const char *description_impl() const noexcept override
    {
        return "Generates the classic skirt and first-layer skirt-brim loops using only the plugin API.";
    }
    const char *exclusive_group_impl() const noexcept override { return k_group_id; }
    slicing_step_t step_impl() const noexcept override { return STEP_SKIRT_BRIM; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    int32_t priority_impl() const noexcept override { return 100; }
    const char *progress_message_format_impl() const noexcept override { return "Generating skirt"; }

    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        if (keys == nullptr)
            return int32_t(std::size(k_used_config_keys));
        std::copy(std::begin(k_used_config_keys), std::end(k_used_config_keys), keys);
        return int32_t(std::size(k_used_config_keys));
    }

    int32_t defined_config_keys(const char **keys) const noexcept override
    {
        if (keys != nullptr)
            for (size_t idx = 0; idx < std::size(k_defined_config_keys); ++idx)
                keys[idx] = k_defined_config_keys[idx];
        return int32_t(std::size(k_defined_config_keys));
    }

    void inilialize_impl(storage_handle *) const override
    {
        initialize_default_skirt_generator_settings(m_orchestrator);
        add_default_skirt_generator_gui_rules(m_orchestrator);
    }

    void run_impl(const plugin_run_context *run_ctx) const override
    {
        throw_if_cancelled(run_ctx);
        generate_default_skirt(run_ctx, m_orchestrator);
    }
};

} // namespace

void register_default_skirt_generator_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(orchestrator, DefaultSkirtGenerator::instance(orchestrator).c_instance());
    const plugin_instance member = DefaultSkirtGenerator::instance(orchestrator).c_instance();
    raw_exclusive_group_def definition = raw_exclusive_group_def_init();
    definition.exclusive_group = member.vt->get_exclusive_group(member.ctx);
    definition.label = "Skirt generator";
    definition.tooltip = "Selects which plugin generates skirt extrusion around the first-layer envelope.";
    DefaultSkirtGenerator::instance(orchestrator).register_exclusive_group(STEP_SKIRT_BRIM, definition);
}

}}} // namespace slic3r_api::SkirtBrim::DefaultSkirtGeneratorPlugin
