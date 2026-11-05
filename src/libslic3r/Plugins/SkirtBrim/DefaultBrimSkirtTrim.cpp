///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "DefaultBrimSkirtTrim.hpp"
#include "AdhesionLayerHelpers.hpp"

#include <algorithm>
#include <cassert>
#include <iterator>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_skirt_brim.h"
#include "libslic3r/Api/internal/PrintObjectAccess.hpp"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/SkirtBrimStepViews.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/PrintObject.hpp"

/*
Default brim/skirt trim
=======================

This post-plugin prevents a generated brim from occupying the band already
used by the final skirt. It consumes the brim and skirt extrusion trees
published by earlier STEP_SKIRT_BRIM plugins; it does not generate either
feature.

The normal execution flow is:

For the global trim operation, the helper functions perform the geometric work:

Object-owned brim is clipped one source layer and instance at a time. Its mask
stays in the same local coordinate system: a print-level fallback is copied and
translated back by that instance's shift before skirt_trim_area(). The trimmed
root is published with its owner before its source layer is removed, preserving
placement even when equal-Z publication changes layer indices.

Only closed skirt polylines with usable extrusion dimensions contribute to the
trim band. Open or invalid skirt paths are ignored, and the plugin clips
existing brim centerlines rather than regenerating their extrusion widths or
producing final machine G-code.

Operation notes:
- trim_brim_against_skirt()
- validate the step context and the trim setting
- collect print-level brim and skirt trees
- trim the print-level brim when both trees exist
- snapshot each object's brim layer identities and validate their owners
- collect the matching local skirt, or translate the print-level fallback
- replace each affected brim auxiliary layer
- collect closed skirt loops and offset their outer/inner bounds
- append_trimmed_brim_tree()
- recursively clip every brim leaf against that band
- preserve the original leaf properties on every remaining piece
- remove and republish the brim auxiliary layer

Representative direct calls (branches are not a timeline):

    trim_brim_against_skirt()
        |-- collect_auxiliary_layer_extrusions()
        |-- trim_print_brim()
        |-- layer_instance_projections()
        |-- translate_trim_tree()
        |-- publish_brim()
        |-- skirt_trim_area()
        `-- append_trimmed_brim_tree()

    trim_print_brim()
        |-- skirt_trim_area()
        |-- append_trimmed_brim_tree()
        |-- remove_auxiliary_layers()
        `-- publish_brim()
*/

namespace slic3r_api { namespace SkirtBrim { namespace DefaultBrimSkirtTrimPlugin {
// Register the permanent borrowed plugin and copy its alternative-selection setting description into the application.
// Print data is unchanged; allocation failures or rejected descriptions throw.
void register_default_brim_skirt_trim_plugin(orchestrator_handle *orchestrator);

namespace {

const char *const k_no_dependencies[] = { nullptr };
const char *const k_group_id = "skirt_brim.brim_skirt_trim";
const char *const k_trim_enabled_key = "brim_skirt_trim";
const char *const k_defined_config_keys[] = { k_trim_enabled_key };
constexpr double k_rounding_overlap = 1.0 - 0.25 * PI;

const raw_used_config_key k_used_config_keys[] = {
    { k_trim_enabled_key, RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "draft_shield", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "skirt_distance", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "brim_width", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "first_layer_height", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "skirt_extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "first_layer_extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "first_layer_extrusion_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "perimeter_extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "perimeter_extrusion_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "extrusion_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "nozzle_diameter", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "perimeters", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "brim_width_interior", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "fill_density", RAW_CO_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "top_solid_layers", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "bottom_solid_layers", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "solid_infill_every_layers", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "perimeter_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "infill_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "solid_infill_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE }
};

// Traverse a mutable view into owned storage and translate only polylines, leaving commands/properties unchanged; no ownership transfer occurs and a rejected mutation throws.
void translate_trim_tree(MutableExtrusionEntity entity, c_point shift);

// Borrow local trimmed geometry and publish its moved root with optional instance provenance; return false for empty output and propagate geometry/allocation failures without changing unrelated layers.
bool publish_brim(storage_handle *storage,
                  orchestrator_handle *orchestrator,
                  const Print &print,
                  const Object &object,
                  coord_t height,
                  coord_t print_z,
                  coord_t slice_z,
                  StoredExtrusionEntity &brim,
                  std::optional<uint32_t> owner_instance = std::nullopt);

// Borrow the print and storage, trim global brim in plate space and each object layer separately against a locally translated skirt mask. Republish instance-specific local roots before removing their sources, preserving provenance and propagating failures.
void trim_brim_against_skirt(const plugin_run_context *run_ctx, orchestrator_handle *orchestrator);

void append_closed_leaf_polygons(storage_handle *storage,
                                 const ExtrusionEntity &entity,
                                 StoredPolygonCollection &out,
                                 double &max_width,
                                 double &max_height)
{
    /*
    Skirt geometry is stored as closed extrusion loops. For clipping, only the
    centerline polygon is needed. The duplicate closing point is dropped because
    Polygon stores an implicit closing edge between back() and front().
    */
    if (entity.has_polyline()) {
        if (!entity.local_is_closed() || entity.point_count() < 4)
            return;

        const std::vector<c_point> points = entity.points();
        StoredPolygon polygon(storage);
        for (size_t point_idx = 0; point_idx + 1 < points.size(); ++point_idx)
            polygon.push_back(points[point_idx]);
        if (!polygon.empty() && polygon.is_valid())
            out.push_back(polygon.readonly());

        /*
        The trim plugin runs after the skirt generator, so the real flow is
        already attached to each skirt path. Reading the printed width/height
        from the extrusion avoids recomputing skirt settings in this plugin.
        */
        if (const EPropertyAttributes *attributes = entity.get(EPropertyAttributes::key)) {
            max_width = std::max(max_width, double(attributes->c_extrusion_property_attributes::width));
            max_height = std::max(max_height, double(attributes->c_extrusion_property_attributes::height));
        }
        return;
    }

    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        append_closed_leaf_polygons(storage, entity.child(child_idx), out, max_width, max_height);
}

StoredExPolygonCollection skirt_trim_area(storage_handle *storage, const ExtrusionEntity &skirt)
{
    StoredPolygonCollection skirt_polygons(storage);
    double max_width = 0.0;
    double max_height = 0.0;
    append_closed_leaf_polygons(storage, skirt, skirt_polygons, max_width, max_height);
    if (skirt_polygons.empty())
        return StoredExPolygonCollection(storage);
    if (max_width <= 0.0 || max_height <= 0.0)
        return StoredExPolygonCollection(storage);

    /*
    The generated skirt is ordered outside-to-inside. The trim area is the
    physical band occupied by the skirt: the outermost loop expanded by half
    spacing, minus the innermost loop shrunk by half spacing. With a single
    skirt line, front() and back() are the same loop, which naturally produces
    a one-line band.
    */
    const double spacing = max_width - max_height * k_rounding_overlap;
    if (spacing <= 0.0)
        return StoredExPolygonCollection(storage);

    const double half_spacing = 0.5 * scale_d(spacing);
    const double cleanup = double(std::max<coord_t>(SCALED_EPSILON, scale_i(max_width) / 10));
    const ClipperContext clipper(storage);

    ClipperOperand outer =
        clipper_offset(clipper(skirt_polygons.front()), half_spacing, CLIPPER_JOIN_ROUND, cleanup);
    ClipperOperand inner =
        clipper_offset(clipper(skirt_polygons.back()), -half_spacing, CLIPPER_JOIN_ROUND, cleanup);
    return clipper_diff(std::move(outer), inner).to_expolygon_collection();
}

void append_trimmed_brim_leaf(storage_handle *storage,
                              const ExtrusionEntity &leaf,
                              const ExPolygonCollection &trim_area,
                              StoredExtrusionEntity &out)
{
    if (!leaf.has_polyline() || leaf.point_count() < 2)
        return;

    StoredPolyline source(storage);
    for (c_point point : leaf.points())
        source.push_back(point);

    StoredPolylineCollection pieces =
        clipper_diff_polyline_expolygons(storage, source.readonly(), trim_area);
    for (Polyline piece : pieces) {
        if (piece.size() < 2)
            continue;

        /*
        Clone first so the clipped piece keeps the original flow, role and any
        other direct properties attached by the brim generator. Then replace
        only the centerline points.
        */
        StoredExtrusionEntity clipped(storage, leaf);
        if (!clipped.set(piece))
            continue;
        out.append_child_move(clipped.mutable_view());
    }
}

void append_trimmed_brim_tree(storage_handle *storage,
                              const ExtrusionEntity &entity,
                              const ExPolygonCollection &trim_area,
                              StoredExtrusionEntity &out)
{
    if (entity.has_polyline()) {
        append_trimmed_brim_leaf(storage, entity, trim_area, out);
        return;
    }

    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        append_trimmed_brim_tree(storage, entity.child(child_idx), trim_area, out);
}

Slic3r::ExPolygons brim_subject_from_extrusion(const ExtrusionEntity &extrusion)
{
    /*
    Auxiliary brim layers need an area subject, while the trimmed result is an
    extrusion tree. Reuse the native cover computation here because the trim
    plugin is built into the host and the API has no cover-by-width helper yet.
    */
    const Slic3r::ExtrusionEntity *native =
        reinterpret_cast<const Slic3r::ExtrusionEntity *>(extrusion.handle());
    Slic3r::Polygons coverage = native->polygons_covered_by_width(float(SCALED_EPSILON));
    return coverage.empty() ? Slic3r::ExPolygons{} : Slic3r::union_ex(coverage);
}

bool publish_brim(storage_handle *storage,
                  orchestrator_handle *orchestrator,
                  const Print &print,
                  const Object &object,
                  coord_t height,
                  coord_t print_z,
                  coord_t slice_z,
                  StoredExtrusionEntity &brim,
                  std::optional<uint32_t> owner_instance)
{
    if (storage == nullptr || brim.empty())
        return false;

    Slic3r::ExPolygons subject = brim_subject_from_extrusion(brim.readonly());
    if (subject.empty())
        return false;

    const ExPolygonCollection subject_view(reinterpret_cast<const expolygon_collection_handle *>(&subject));

    return publish_adhesion_extrusion_to_auxiliary_layer(storage,
                                                         orchestrator,
                                                         print,
                                                         object,
                                                         subject_view,
                                                         height,
                                                         print_z,
                                                         slice_z,
                                                         RAW_LAYER_ADHESION_KIND_BRIM,
                                                         0,
                                                         brim.mutable_view(), owner_instance);
}

bool trim_print_brim(storage_handle *storage,
                     orchestrator_handle *orchestrator,
                     const Print &print,
                     const Object &object,
                     const Layer &reference_layer,
                     const ExtrusionEntity &brim,
                     const ExtrusionEntity &skirt)
{
    if (brim.empty() || skirt.empty())
        return false;

    StoredExPolygonCollection trim_area = skirt_trim_area(storage, skirt);
    if (trim_area.empty())
        return false;

    StoredExtrusionEntity trimmed(storage);
    trimmed.disable_sort().disable_reverse();
    append_trimmed_brim_tree(storage, brim, trim_area.readonly(), trimmed);
    trimmed.disable_sort().disable_reverse();

    const coord_t height = reference_layer.height();
    const coord_t print_z = reference_layer.print_z();
    const coord_t slice_z = reference_layer.slice_z();
    remove_auxiliary_layers(object, layer_is_brim_adhesion);
    if (!trimmed.empty() && !publish_brim(storage, orchestrator, print, object, height, print_z, slice_z, trimmed))
        throw std::runtime_error("Default brim/skirt trim could not publish brim.");
    return true;
}

void trim_brim_against_skirt(const plugin_run_context *run_ctx, orchestrator_handle *orchestrator)
{
    const run_ctx_skirt_brim *ctx = plugin_ctx_as_skirt_brim(run_ctx);
    if (ctx == nullptr || ctx->print == nullptr)
        return;

    const SkirtBrimStep step(ctx);
    const Print print = step.print();
    if (!print.config().get(k_trim_enabled_key).get_bool())
        return;
    if (print.config().get("draft_shield").get_int() == 0)
        return;

    storage_handle *storage = run_ctx->plugin_storage;
    const Object auxiliary_object = print.auxiliary_object();
    StoredExtrusionEntity print_brim =
        collect_auxiliary_layer_extrusions(storage, auxiliary_object, layer_is_brim_adhesion);
    StoredExtrusionEntity print_skirt =
        collect_auxiliary_layer_extrusions(storage, auxiliary_object, layer_is_normal_skirt_adhesion);
    if (!print_brim.empty() && !print_skirt.empty() && auxiliary_object.auxiliary_layer_count() > 0) {
        const Layer reference_layer = auxiliary_object.auxiliary_layer(0);
        trim_print_brim(storage, orchestrator, print, auxiliary_object, reference_layer, print_brim.readonly(), print_skirt.readonly());
    }

    for (uint32_t object_idx = 0; object_idx < print.object_count(); ++object_idx) {
        const Object object = print.object(object_idx);
        if (object.layer_count() == 0)
            continue;
        bool has_skirt = !print_skirt.empty();
        for (uint32_t index = 0; index < object.auxiliary_layer_count(); ++index)
            has_skirt = has_skirt || layer_is_normal_skirt_adhesion(object.auxiliary_layer(index));
        if (!has_skirt)
            continue;
        // Publication inserts equal-Z layers before existing ones: snapshot borrowed source identities, not their changing indices.
        std::vector<Layer> source_layers;
        for (uint32_t index = 0; index < object.auxiliary_layer_count(); ++index) {
            const Layer layer = object.auxiliary_layer(index);
            if (layer_is_brim_adhesion(layer))
                source_layers.push_back(layer);
        }
        for (const Layer &layer : source_layers) {
            StoredExtrusionEntity brim(storage);
            append_auxiliary_layer_extrusions(storage, layer, brim);
            for (const std::pair<uint32_t, c_point> &projection : layer_instance_projections(object, layer)) {
                const uint32_t owner = projection.first;
                const c_point shift = projection.second;
                StoredExtrusionEntity local_skirt(storage);
                for (uint32_t skirt_index = 0; skirt_index < object.auxiliary_layer_count(); ++skirt_index) {
                    const Layer skirt_layer = object.auxiliary_layer(skirt_index);
                    if (!layer_is_normal_skirt_adhesion(skirt_layer))
                        continue;
                    for (const std::pair<uint32_t, c_point> &skirt_projection : layer_instance_projections(object, skirt_layer))
                        if (skirt_projection.first == owner)
                            append_auxiliary_layer_extrusions(storage, skirt_layer, local_skirt);
                }
                if (local_skirt.empty() && !print_skirt.empty()) {
                    local_skirt = StoredExtrusionEntity(storage, print_skirt.readonly());
                    translate_trim_tree(local_skirt.mutable_view(), c_point{-shift.x, -shift.y});
                }
                if (local_skirt.empty()) {
                    StoredExtrusionEntity copy(storage, brim.readonly());
                    if (!publish_brim(storage, orchestrator, print, object,
                            layer.height(), layer.print_z(), layer.slice_z(), copy, owner))
                        throw std::runtime_error("Default brim/skirt trim could not preserve instance brim.");
                    continue;
                }
                StoredExPolygonCollection mask = skirt_trim_area(storage, local_skirt.readonly());
                StoredExtrusionEntity trimmed(storage);
                trimmed.disable_sort().disable_reverse();
                append_trimmed_brim_tree(storage, brim.readonly(), mask.readonly(), trimmed);
                if (!trimmed.empty() && !publish_brim(storage, orchestrator, print, object,
                        layer.height(), layer.print_z(), layer.slice_z(), trimmed, owner))
                    throw std::runtime_error("Default brim/skirt trim could not publish instance brim.");
            }
            object.remove_auxiliary_layer(layer);
        }
    }
}

void translate_trim_tree(MutableExtrusionEntity entity, c_point shift)
{
    if (entity.has_polyline()) {
        if (!entity.translate(shift))
            throw std::runtime_error("Cannot translate skirt trim geometry.");
        return;
    }
    for (uint32_t index = 0; index < entity.child_count(); ++index)
        translate_trim_tree(entity.child_mutable(index), shift);
}

class DefaultBrimSkirtTrim : public PluginBase
{
public:
    static DefaultBrimSkirtTrim &instance(orchestrator_handle *orchestrator)
    {
        static DefaultBrimSkirtTrim instance(orchestrator);
        return instance;
    }

    explicit DefaultBrimSkirtTrim(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

private:
    const char *id_impl() const noexcept override { return "skirt_brim.brim_skirt_trim.default"; }
    const char *name_impl() const noexcept override { return "Default brim/skirt trim"; }
    const char *description_impl() const noexcept override
    {
        return "Post-processes brim after the final skirt geometry is known.";
    }
    const char *exclusive_group_impl() const noexcept override { return k_group_id; }
    slicing_step_t step_impl() const noexcept override { return STEP_SKIRT_BRIM; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    int32_t priority_impl() const noexcept override { return 200; }
    const char *progress_message_format_impl() const noexcept override { return "Trimming brim against skirt"; }

    int32_t defined_config_keys(const char **keys) const noexcept override
    {
        if (keys != nullptr)
            keys[0] = k_defined_config_keys[0];
        return int32_t(std::size(k_defined_config_keys));
    }

    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        if (keys == nullptr)
            return int32_t(std::size(k_used_config_keys));
        std::copy(std::begin(k_used_config_keys), std::end(k_used_config_keys), keys);
        return int32_t(std::size(k_used_config_keys));
    }

    void inilialize_impl(storage_handle *) const override
    {
        raw_config_option_def def = raw_config_option_def_init();
        def.opt_key = k_trim_enabled_key;
        def.type = RAW_CO_BOOL;
        def.container_type = RAW_CONTAINER_TYPE_PROJECT;
        def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
        def.printer_technology = RAW_PT_FFF;
        def.label = "Trim brim by skirt";
        def.full_label = "Trim brim by final skirt";
        def.category = RAW_OPTION_CATEGORY_SKIRT_BRIM;
        def.invalidates_step = STEP_SKIRT_BRIM;
        def.tooltip =
            "Trim brim paths that overlap the final skirt or draft shield band. "
            "This is useful when a close draft shield would otherwise be printed over the brim.";
        def.mode = RAW_CONFIG_OPTION_MODE_ADV_EXP | RAW_CONFIG_OPTION_MODE_SUSI;
        def.default_serialized_value = "0";
        orchestrator_create_option_def(m_orchestrator, &def);

        orchestrator_add_ui_fragment(
            m_orchestrator,
            "print.ui",
            k_trim_enabled_key,
            "page:Skirt & Brim\n"
            "group:Brim\n"
            "setting:insert$aftersetting$brim_per_object:brim_skirt_trim\n",
            0);
    }

    void run_impl(const plugin_run_context *run_ctx) const override
    {
        throw_if_cancelled(run_ctx);
        trim_brim_against_skirt(run_ctx, m_orchestrator);
    }
};

} // namespace

void register_default_brim_skirt_trim_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(orchestrator, DefaultBrimSkirtTrim::instance(orchestrator).c_instance());
    const plugin_instance member = DefaultBrimSkirtTrim::instance(orchestrator).c_instance();
    raw_exclusive_group_def definition = raw_exclusive_group_def_init();
    definition.exclusive_group = member.vt->get_exclusive_group(member.ctx);
    definition.label = "Brim/skirt trim";
    definition.tooltip = "Selects the plugin that resolves interactions between generated brim and skirt geometry.";
    DefaultBrimSkirtTrim::instance(orchestrator).register_exclusive_group(STEP_SKIRT_BRIM, definition);
}

}}} // namespace slic3r_api::SkirtBrim::DefaultBrimSkirtTrimPlugin
