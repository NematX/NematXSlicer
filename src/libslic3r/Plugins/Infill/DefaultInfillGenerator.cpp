///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "DefaultInfillGenerator.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"
#include "libslic3r/Api/plugin/cpp/GuiRuleHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/properties/BridgeProperties.hpp"
#include "libslic3r/Api/host/InfillBridgeFlow.hpp"
#include "libslic3r/Plugins/Surface/BridgePreparation.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionProperty.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintRegion.hpp"
#include "libslic3r/Surface.hpp"
#include "libslic3r/libslic3r.h"

/*
Default infill generation
==========================

Generate extrusion paths for each fill surface using its selected infill pattern.
This is the default STEP_INFILL plugin, run once per PrintObject after earlier
steps have classified its fill surfaces. It does not use one fixed algorithm:
each surface selects a pattern plugin through its configured pattern id.

setup_run_impl() performs a discovery pass over the object: it visits layers,
islands, region islands, and their fill surfaces. For each usable surface,
make_surface_recipe() resolves its flow, density, pattern, connection, bridge
data, and overlap area; the optional recipe callback may change those values.
Prepared bridges are validated after that complete modifier chain, before their
recipe can be cached. Their generated attributes regain native circular dimensions
only when they still exactly match the pattern's transported flow.
Surfaces of one region island that end up with identical raw parameters are
grouped, and every group is reported to the host through
report_pattern_assignments() so the used pattern plugins can be initialized
once for the print. The recipes are kept in m_object_caches until run().

run_impl() replays the cached groups instead of recomputing the recipes. Each
surface still results in one pattern call, and the generated subtree is kept
only when the pattern and density are valid. It receives an
ExtrusionPropertyInfill containing the source surface id, the raw fill recipe
and, when the recipe requests an enforced full fill volume, the free area the
surface expects. The plugin publishes that subtree through
append_region_island_extrusion() and a STEP_POST_INFILL plugin later turns the
area and the recipe into a flow correction. Gap fill is not generated here; it
is deliberately left for the post-infill step.

Representative direct calls, not loop nesting or a timeline:

    DefaultInfillGenerator::setup_run_impl()
        |-- make_surface_recipe()
        |   |-- resolve_pattern_id() [host callback]
        |   |-- modify_surface_recipe() [optional host callback]
        |   `-- BridgePreparation::validate_recipe() [after all modifiers]
        |-- params_key() [group equal effective recipes]
        `-- report_pattern_assignments() [host callback]

    DefaultInfillGenerator::run_impl()
        |-- make_surface_recipe() [only for an uncached direct run]
        `-- generate_surface_from_recipe()
            |-- generate_pattern() [host callback]
            |-- restore_prepared_bridge_dimensions() [matching transport only]
            `-- append_region_island_extrusion() [host callback]

Discovery visits layers, islands, region groups and surfaces, skipping empty,
void or invalid recipes. It accumulates progress totals and reports each pattern.
Normal execution reuses that cache; an uncached direct run rebuilds recipes.
Temporary pattern output uses a generic owned Children root. A usable result
receives its source-surface identity, recipe values and optional target area
before host publication. Empty or failed results are discarded without publishing
partial output. Prepared bridge dimensions are restored only for matching
transport, independently of this root's representation.

The generator works through the public step context and callbacks. It reads
the existing print data but leaves ownership and direct publication of the
result to the host API.
*/

namespace slic3r_api { namespace Infill { namespace DefaultInfillGeneratorPlugin {

// One surface kept in a work group, with the free area and the area to fill
// computed once during the discovery pass.
struct DefaultInfillSurfaceWork
{
    const Slic3r::Surface *surface = nullptr;
    Slic3r::ExPolygons no_overlap_areas;
    double area_to_fill_mm2 = 0.;
};

// Surfaces of one region island that share the same raw pattern parameters.
// They are filled by separate pattern calls but initialized as one unit.
struct DefaultInfillWorkGroup
{
    const Slic3r::Layer *layer = nullptr;
    const Slic3r::LayerSliceIsland *island = nullptr;
    const Slic3r::LayerRegionIsland *region_island = nullptr;
    const Slic3r::LayerRegion *region = nullptr;
    infill_pattern_runtime_id pattern_id = INFILL_PATTERN_RUNTIME_ID_INVALID;
    raw_infill_pattern_params params = {};
    std::vector<DefaultInfillSurfaceWork> surfaces;
};

// Recipes discovered by setup_run() for one object. The pattern plugins receive
// pointers into this cache, so it must stay alive until run() for the object.
struct DefaultInfillObjectWorkCache
{
    std::vector<DefaultInfillWorkGroup> groups;
};

namespace {

// Restore native circular dimensions only for attributes matching the pattern's
// scaled transport exactly. Other physical models retain their incoming attributes.
void restore_prepared_bridge_dimensions(Slic3r::ExtrusionEntity &output, const c_flow &transport,
                                       const Slic3r::Flow &native, const plugin_run_context *run_ctx);

const char *k_default_infill_generator_id = "infill.generator.default";
const char *k_no_dependencies[] = { nullptr };

// Generate one surface and publish its recipe-bearing subtree unless the pattern fails or produces no geometry.
// Callbacks borrow the local output for each call; cancellation checks may throw.
void generate_surface_from_recipe(const run_ctx_generate_infill &ctx,
                                  const Slic3r::Layer &layer,
                                  const Slic3r::LayerSliceIsland &island,
                                  const Slic3r::LayerRegionIsland &region_island,
                                  const Slic3r::LayerRegion &region,
                                  const Slic3r::Surface &surface,
                                  infill_pattern_runtime_id pattern_id,
                                  const raw_infill_pattern_params &params,
                                  const Slic3r::ExPolygons &no_overlap_areas,
                                  double area_to_fill_mm2,
                                  const plugin_run_context *run_ctx);

// The generator owns the five infill pattern lists. Pattern plugins may extend
// them through orchestrator_modify_option_def(); every value is a plugin id.
const char *k_fill_pattern_key = "fill_pattern";
const char *k_top_fill_pattern_key = "top_fill_pattern";
const char *k_bottom_fill_pattern_key = "bottom_fill_pattern";
const char *k_solid_fill_pattern_key = "solid_fill_pattern";
const char *k_bridge_fill_pattern_key = "bridge_fill_pattern";

const raw_used_config_key k_used_config_keys[] = {
    { k_fill_pattern_key, RAW_CO_ENUM, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT },
    { k_top_fill_pattern_key, RAW_CO_ENUM, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT },
    { k_bottom_fill_pattern_key, RAW_CO_ENUM, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT },
    { k_solid_fill_pattern_key, RAW_CO_ENUM, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT },
    { k_bridge_fill_pattern_key, RAW_CO_ENUM, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT }
};
const char *k_defined_config_keys[] = {
    k_fill_pattern_key,
    k_top_fill_pattern_key,
    k_bottom_fill_pattern_key,
    k_solid_fill_pattern_key,
    k_bridge_fill_pattern_key
};

void create_pattern_option(orchestrator_handle *orchestrator,
                           const char *key,
                           const char *label,
                           const char *full_label,
                           const char *tooltip,
                           const char *default_id,
                           const key_value_string_pair_t *entries,
                           uint32_t entry_count,
                           raw_config_option_mode mode,
                           const char *cli = nullptr,
                           const char *alias = nullptr)
{
    raw_config_option_def def = raw_config_option_def_init();
    def.opt_key = key;
    def.type = RAW_CO_ENUM;
    def.gui_type = RAW_GUI_TYPE_SELECT_CLOSE;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.category = RAW_OPTION_CATEGORY_INFILL;
    def.invalidates_step = STEP_INFILL;
    def.mode = mode;
    def.label = label;
    def.full_label = full_label;
    def.tooltip = tooltip;
    def.cli = cli;
    if (alias != nullptr)
        def.aliases = { &alias, 1 };
    def.contains_plugin_id = 1;
    def.default_serialized_value = default_id;
    def.enum_def.value_label_pairs = { entries, entry_count };
    orchestrator_create_option_def(orchestrator, &def);
}

void create_infill_pattern_options(orchestrator_handle *orchestrator)
{
    // Base lists of pattern plugin ids. Patterns moved to their own plugins
    // (gyroid, honeycomb, 3dhoneycomb, hilbertcurve, archimedeanchords,
    // octagramspiral) contribute themselves through
    // orchestrator_modify_option_def(). The legacy "(filled)" variants are
    // gone: gap fill is owned by infill.post_process.gap_fill.
    static const key_value_string_pair_t k_sparse_patterns[] = {
        { "rectilinear",         "Rectilinear" },
        { "alignedrectilinear",  "Aligned Rectilinear" },
        { "monotonic",           "Monotonic" },
        { "grid",                "Grid" },
        { "triangles",           "Triangles" },
        { "stars",               "Stars" },
        { "cubic",               "Cubic" },
        { "line",                "Line" },
        { "concentric",          "Concentric" },
        { "scatteredrectilinear","Scattered Rectilinear" },
        { "adaptivecubic",       "Adaptive Cubic" },
        { "supportcubic",        "Support Cubic" },
        { "lightning",           "Lightning" },
    };
    static const key_value_string_pair_t k_top_patterns[] = {
        { "rectilinear",         "Rectilinear" },
        { "monotonic",           "Monotonic" },
        { "alignedrectilinear",  "Aligned Rectilinear" },
        { "concentric",          "Concentric" },
        { "sawtooth",            "Sawtooth" },
        { "smooth",              "Ironing" },
    };
    static const key_value_string_pair_t k_bottom_patterns[] = {
        { "rectilinear",         "Rectilinear" },
        { "monotonic",           "Monotonic" },
        { "alignedrectilinear",  "Aligned Rectilinear" },
        { "concentric",          "Concentric" },
        { "smooth",              "Ironing" },
    };
    static const key_value_string_pair_t k_solid_patterns[] = {
        { "ensuring",            "Ensuring" },
        { "rectilinear",         "Rectilinear" },
        { "monotonic",           "Monotonic" },
        { "alignedrectilinear",  "Aligned Rectilinear" },
        { "concentric",          "Concentric" },
        { "smooth",              "Ironing" },
    };
    static const key_value_string_pair_t k_bridge_patterns[] = {
        { "rectilinear",         "Rectilinear" },
        { "monotonic",           "Monotonic" },
    };

    create_pattern_option(orchestrator, k_fill_pattern_key, "Pattern", "Sparse fill pattern",
        "Fill pattern for general low-density infill."
        "\nIf you want an 'aligned' pattern, set 90° to the fill angle increment setting.",
        "stars", k_sparse_patterns,
        uint32_t(sizeof(k_sparse_patterns) / sizeof(k_sparse_patterns[0])),
        RAW_CONFIG_OPTION_MODE_SIM_ADV_EXP);
    create_pattern_option(orchestrator, k_top_fill_pattern_key, "Top", "Top fill Pattern",
        "Fill pattern for top infill. This only affects the top visible layer, and not its adjacent solid shells."
        "\nIf you want an 'aligned' pattern, set 90° to the fill angle increment setting.",
        "monotonic", k_top_patterns,
        uint32_t(sizeof(k_top_patterns) / sizeof(k_top_patterns[0])),
        RAW_CONFIG_OPTION_MODE_SIM_ADV_EXP,
        "top-fill-pattern|external-fill-pattern=s", "external_fill_pattern");
    create_pattern_option(orchestrator, k_bottom_fill_pattern_key, "Bottom", "Bottom fill pattern",
        "Fill pattern for bottom infill. This only affects the bottom visible layer, and not its adjacent solid shells."
        "\nIf you want an 'aligned' pattern, set 90° to the fill angle increment setting.",
        "monotonic", k_bottom_patterns,
        uint32_t(sizeof(k_bottom_patterns) / sizeof(k_bottom_patterns[0])),
        RAW_CONFIG_OPTION_MODE_ADV_EXP,
        "bottom-fill-pattern|external-fill-pattern=s", "external_fill_pattern");
    create_pattern_option(orchestrator, k_solid_fill_pattern_key, "Solid fill pattern", "Solid fill pattern",
        "Fill pattern for solid (internal) infill. This only affects the solid not-visible layers."
        " You should use rectilinear in most cases. You can try ironing for translucent material.",
        "ensuring", k_solid_patterns,
        uint32_t(sizeof(k_solid_patterns) / sizeof(k_solid_patterns[0])),
        RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI);
    create_pattern_option(orchestrator, k_bridge_fill_pattern_key, "Bridging fill pattern", "Bridging fill pattern",
        "Fill pattern for bridges and internal bridge infill.",
        "rectilinear", k_bridge_patterns,
        uint32_t(sizeof(k_bridge_patterns) / sizeof(k_bridge_patterns[0])),
        RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI);
}

void register_infill_pattern_rules(orchestrator_handle *orchestrator)
{
    using slic3r_api::all_of;
    using slic3r_api::any_of;
    using slic3r_api::int_equals;
    using slic3r_api::non_zero;
    using slic3r_api::true_;

    const auto add_rule = [orchestrator](const char *key, const slic3r_api::GuiCondition &condition) {
        slic3r_api::GuiRule::activate(key)
            .when(condition)
            .scope(RAW_PRESET_TYPE_FFF_PRINT)
            .add_to(orchestrator);
    };

    // Same conditions as the legacy FFF print rules: each pattern list is
    // selectable only when the matching surface kind is printed.
    const slic3r_api::GuiCondition has_top = any_of({
        non_zero("top_solid_layers"),
        all_of({ non_zero("perimeters"), true_("spiral_vase") }),
        int_equals("solid_infill_every_layers", 1)
    });
    const slic3r_api::GuiCondition has_bottom = any_of({
        non_zero("bottom_solid_layers"),
        int_equals("solid_infill_every_layers", 1)
    });
    const slic3r_api::GuiCondition has_solid = any_of({
        non_zero("top_solid_layers"),
        all_of({ non_zero("perimeters"), true_("spiral_vase") }),
        int_equals("solid_infill_every_layers", 1),
        non_zero("bottom_solid_layers"),
        all_of({ non_zero("fill_density"), non_zero("solid_infill_every_layers") }),
        all_of({ non_zero("fill_density"), non_zero("solid_infill_below_area") })
    });

    add_rule(k_fill_pattern_key, non_zero("fill_density"));
    add_rule(k_top_fill_pattern_key, has_top);
    add_rule(k_bottom_fill_pattern_key, has_bottom);
    add_rule(k_solid_fill_pattern_key, has_solid);
    add_rule(k_bridge_fill_pattern_key, has_solid);
}


struct SurfaceInfillRecipe
{
    std::string pattern_plugin_id;
    raw_infill_pattern_params raw = {};
    Slic3r::ExPolygons no_overlap_areas;
    // Free-of-perimeter-overlap area this surface expects, or 0 when no volume
    // correction was requested. It is not part of raw, so grouping stays
    // byte-identical.
    double area_to_fill_mm2 = 0.;
};

c_flow to_c_flow(const Slic3r::Flow &flow)
{
    c_flow out = {};
    out.width = flow.scaled_width();
    out.spacing = flow.scaled_spacing();
    out.height = flow.scaled_height();
    out.nozzle_diameter = Slic3r::scale_i(flow.nozzle_diameter());
    out.is_bridge = flow.bridge() ? 1 : 0;
    out.spacing_ratio = flow.spacing_ratio();
    out.mm3_per_mm = flow.mm3_per_mm();
    return out;
}

std::string serialized_pattern_id(const Slic3r::PrintRegionConfig &region_config, const char *key)
{
    // Read the option generically so built-in enum options and plugin-defined
    // pattern id options follow the same path.
    const Slic3r::ConfigOption *option = region_config.option(key);
    if (option == nullptr)
        return std::string();
    std::string value = option->serialize();
    if (!value.empty() && value.front() == '!')
        value.erase(value.begin());
    return value;
}

float compute_fill_angle_for_layer(const Slic3r::PrintRegionConfig &region_config, size_t layer_id)
{
    // Pattern plugins receive the final angle in radians. Keeping the template
    // and increment logic here avoids duplicating config interpretation in
    // every pattern implementation.
    float angle_degrees = 0.f;
    if (!region_config.fill_angle_template.empty()) {
        const size_t idx = layer_id % region_config.fill_angle_template.size();
        angle_degrees = region_config.fill_angle_template.get_at(idx);
    } else {
        angle_degrees = region_config.fill_angle.value;
    }
    angle_degrees += region_config.fill_angle_increment.value * layer_id;
    return float(Slic3r::Geometry::deg2rad(angle_degrees));
}

Slic3r::FlowRole flow_role_for_surface(const Slic3r::Surface &surface)
{
    if (surface.get_property<BridgeSurfaceInfo>() != nullptr)
        return Slic3r::frSolidInfill;
    if (surface.has_pos_top())
        return Slic3r::frTopSolidInfill;
    if (surface.has_fill_solid())
        return Slic3r::frSolidInfill;
    return Slic3r::frInfill;
}

raw_extrusion_role raw_role_for_surface(const Slic3r::Surface &surface, bool bridge)
{
    if (bridge)
        return surface.has_pos_bottom() ? RAW_EXTRUSION_ROLE_BRIDGE_INFILL :
                                          RAW_EXTRUSION_ROLE_INTERNAL_BRIDGE_INFILL;
    if (surface.has_fill_solid())
        return surface.has_pos_top() ? RAW_EXTRUSION_ROLE_TOP_SOLID_INFILL :
                                       RAW_EXTRUSION_ROLE_SOLID_INFILL;
    return RAW_EXTRUSION_ROLE_INTERNAL_INFILL;
}

std::string pattern_for_surface(const Slic3r::Surface &surface,
                                bool bridge,
                                const Slic3r::PrintRegionConfig &region_config)
{
    if (bridge)
        return serialized_pattern_id(region_config, "bridge_fill_pattern");
    if (surface.has_pos_top())
        return serialized_pattern_id(region_config, "top_fill_pattern");
    if (surface.has_pos_bottom())
        return serialized_pattern_id(region_config, "bottom_fill_pattern");
    if (surface.has_fill_solid())
        return serialized_pattern_id(region_config, "solid_fill_pattern");
    return serialized_pattern_id(region_config, "fill_pattern");
}

Slic3r::InfillConnection connection_for_surface(const Slic3r::Surface &surface,
                                                bool bridge,
                                                const Slic3r::PrintRegionConfig &region_config)
{
    if (bridge)
        return region_config.infill_connection_bridge.value;
    if (surface.has_pos_top())
        return region_config.infill_connection_top.value;
    if (surface.has_pos_bottom())
        return region_config.infill_connection_bottom.value;
    if (surface.has_fill_solid())
        return region_config.infill_connection_solid.value;
    return region_config.infill_connection.value;
}

Slic3r::Flow normal_flow_for_surface(const Slic3r::Layer &layer,
                                     const Slic3r::LayerRegion &region,
                                     Slic3r::FlowRole extrusion_role,
                                     const Slic3r::Surface &surface)
{
    const float height = surface.scaled_thickness() == -1 ?
        float(layer.unscaled_height()) :
        float(surface.unscaled_thickness());
    return region.region().flow(*layer.object(), extrusion_role, height, layer.id());
}

double perimeter_spacing_for_overlap(const Slic3r::LayerRegion &region,
                                     const Slic3r::PrintRegionConfig &region_config)
{
    if (region_config.perimeters == 1)
        return region.flow(Slic3r::frExternalPerimeter).spacing();
    if (region_config.only_one_perimeter_top)
        return std::min(region.flow(Slic3r::frPerimeter).spacing(),
                        region.flow(Slic3r::frExternalPerimeter).spacing());
    return region.flow(Slic3r::frPerimeter).spacing();
}

Slic3r::ExPolygons make_no_overlap_areas(const Slic3r::LayerSliceIsland &island,
                                         const Slic3r::Surface &surface,
                                         const Slic3r::PrintRegionConfig &region_config,
                                         double overlap,
                                         const plugin_run_context *run_ctx)
{
    if (region_config.perimeters <= 0 || overlap == 0.)
        return Slic3r::ExPolygons{surface.expolygon};

    // Pattern extrusion may encroach into perimeters, but volume accounting and
    // bridge placement still need the strictly free area. The island owns that
    // free area because region-level free areas can overlap after offsetting.
    throw_if_cancelled(run_ctx);
    Slic3r::ExPolygons result =
        Slic3r::intersection_ex(island.infill_free_areas(), Slic3r::ExPolygons{surface.expolygon});
    throw_if_cancelled(run_ctx);
    return result;
}

SurfaceInfillRecipe make_surface_recipe(const run_ctx_generate_infill &ctx,
                                        const Slic3r::Print &print,
                                        const Slic3r::PrintObject &object,
                                        const Slic3r::Layer &layer,
                                        const Slic3r::LayerSliceIsland &island,
                                        const Slic3r::LayerRegionIsland &region_island,
                                        const Slic3r::LayerRegion &region,
                                        const Slic3r::Surface &surface,
                                        const plugin_run_context *run_ctx)
{
    const Slic3r::PrintRegionConfig &region_config = region.region().config();
    const bool bridge = layer.id() > 0 && surface.has_mod_bridge();
    const Slic3r::FlowRole extrusion_role = flow_role_for_surface(surface);
    const std::string pattern_id = pattern_for_surface(surface, bridge, region_config);
    const Slic3r::Flow flow = bridge ?
        Slic3r::infill_bridge_flow(layer, region, extrusion_role) :
        normal_flow_for_surface(layer, region, extrusion_role, surface);

    SurfaceInfillRecipe recipe;
    // Grouping relies on the full raw parameter bytes, so padding must be
    // deterministic. Zero the struct before assigning the individual members.
    std::memset(&recipe.raw, 0, sizeof(recipe.raw));
    recipe.pattern_plugin_id = pattern_id;
    recipe.raw.surface_type = static_cast<raw_surface_type>(surface.surface_type);
    recipe.raw.extrusion_role = raw_role_for_surface(surface, bridge);
    recipe.raw.flow = to_c_flow(flow);
    recipe.raw.density = surface.has_fill_solid() || bridge ? 1.f : float(region_config.fill_density) / 100.f;
    if (surface.has_mod_overBridge())
        recipe.raw.density = float(region_config.over_bridge_flow_ratio.get_effective_value(1));
    recipe.raw.flow_mult = surface.has_pos_top() && surface.get_property<BridgeSurfaceInfo>() == nullptr ?
        float(region_config.fill_top_flow_ratio.get_effective_value(1)) :
        1.f;
    recipe.raw.connection = static_cast<c_infill_connection>(connection_for_surface(surface, bridge, region_config));

    // Gap fill is now a post-infill responsibility. Pattern plugins should
    // generate their normal fill only and leave narrow residuals for the later
    // post-process plugin, so C_INFILL_FLAG_ADD_GAP_FILL and
    // C_INFILL_FLAG_GAP_FILL_ENABLED stay clear.
    uint16_t infill_flags = 0;
    if (pattern_id == "monotonic")
        infill_flags |= C_INFILL_FLAG_MONOTONIC;
    if (pattern_id == "sawtooth")
        infill_flags |= C_INFILL_FLAG_DONT_ADJUST;
    if (region_config.enforce_full_fill_volume.get_bool())
        infill_flags |= C_INFILL_FLAG_FILL_EXACTLY;
    if ((region_config.perimeter_generator == Slic3r::PerimeterGeneratorType::Arachne &&
         pattern_id == "concentric") ||
        pattern_id == "ensuring")
        infill_flags |= C_INFILL_FLAG_USE_ARACHNE;
    if (region_config.fill_angle_cross)
        infill_flags |= C_INFILL_FLAG_CAN_ANGLE_CROSS;
    // A bridge gets no gap fill, so its line phase alone decides its coverage; lines along the border are welcome there,
    // they rest on the perimeters. This only places fixed-width lines: variable widths would fill a bridge better.
    // A prepared bridge gets its pitch and phase from the bridge recipe pass (BridgeRows.hpp); the pattern keeps that phase.
    if (bridge)
        infill_flags |= C_INFILL_FLAG_BEST_COVERAGE;
    recipe.raw.infill_flags = infill_flags;
    recipe.raw.extruder = region.region().extruder(extrusion_role, object);
    recipe.raw.priority = int32_t(surface.priority);
    recipe.raw.spacing = bridge || surface.has_fill_solid() ?
        flow.spacing() :
        region.region().flow(object, Slic3r::frInfill, float(layer.unscaled_height()), layer.id()).spacing();
    recipe.raw.angle = compute_fill_angle_for_layer(region_config, layer.id());
    if (object.config().fill_angle_follow_model)
        recipe.raw.angle += float(object.trafo().rotation().eulerAngles(0, 1, 2).z());
    recipe.raw.bridge_angle = surface.bridge_angle;
    recipe.raw.bridge_type = int32_t(static_cast<uint8_t>(region_config.bridge_type.value));
    recipe.raw.layer_height = layer.unscaled_height();
    recipe.raw.z = layer.unscaled_print_z();
    recipe.raw.layer_id = uint32_t(layer.id() - object.layer(0).id());
    recipe.raw.bridge_offset = -1;
    recipe.raw.fill_resolution = std::min(flow.scaled_width() / 16,
        std::max(SCALED_EPSILON, Slic3r::scale_i(print.config().resolution_internal.value)));
    recipe.raw.anchor_length = 1000.f;
    recipe.raw.anchor_length_max = 1000.f;
    const Slic3r::BoundingBox &object_bounding_box = object.bounding_box();
    recipe.raw.object_bounding_box.min = { object_bounding_box.min.x(), object_bounding_box.min.y() };
    recipe.raw.object_bounding_box.max = { object_bounding_box.max.x(), object_bounding_box.max.y() };

    if (!surface.has_fill_solid() && !bridge) {
        recipe.raw.anchor_length = float(region_config.infill_anchor);
        if (region_config.infill_anchor.percent)
            recipe.raw.anchor_length = float(recipe.raw.anchor_length * 0.01 * recipe.raw.spacing);
        recipe.raw.anchor_length_max = float(region_config.infill_anchor_max);
        if (region_config.infill_anchor_max.percent)
            recipe.raw.anchor_length_max = float(recipe.raw.anchor_length_max * 0.01 * recipe.raw.spacing);
        recipe.raw.anchor_length = std::min(recipe.raw.anchor_length, recipe.raw.anchor_length_max);
        if (region_config.fill_aligned_z)
            recipe.raw.max_sparse_infill_spacing = Slic3r::unscaled(object.get_sparse_max_spacing());
    } else if (bridge) {
        recipe.raw.anchor_length = 0.f;
        recipe.raw.anchor_length_max = 0.f;
    }

    const double perimeter_spacing = perimeter_spacing_for_overlap(region, region_config);
    recipe.raw.overlap = region_config.perimeters > 0 ?
        region_config.infill_overlap.get_effective_value((perimeter_spacing + recipe.raw.spacing) / 2.) :
        0.;
    const int extruder_index = recipe.raw.extruder == 0 ? 0 : int(recipe.raw.extruder - 1);
    recipe.raw.loop_clipping =
        Slic3r::scale_i(print.config().get_computed_value("seam_gap", extruder_index) * flow.nozzle_diameter());
    recipe.raw.link_max_length = (!flow.bridge() && recipe.raw.density > .8f) ?
        Slic3r::scale_i(3. * recipe.raw.spacing) :
        0;
    recipe.no_overlap_areas = make_no_overlap_areas(island, surface, region_config, recipe.raw.overlap, run_ctx);

    // Config stores the selected pattern as a stable plugin id string. The
    // infill ABI uses a compact runtime id so recipe modifiers can switch
    // patterns without editing caller-owned text buffers.
    throw_if_cancelled(run_ctx);
    recipe.raw.pattern_id = ctx.resolve_pattern_id != nullptr ?
        ctx.resolve_pattern_id(&ctx, recipe.pattern_plugin_id.c_str()) :
        INFILL_PATTERN_RUNTIME_ID_INVALID;
    throw_if_cancelled(run_ctx);
    if (ctx.modify_surface_recipe != nullptr) {
        throw_if_cancelled(run_ctx);
        ctx.modify_surface_recipe(&ctx,
                                  reinterpret_cast<const layer_handle *>(&layer),
                                  reinterpret_cast<const layer_island_handle *>(&island),
                                  reinterpret_cast<const layer_region_island_handle *>(&region_island),
                                  reinterpret_cast<const layer_region_handle *>(&region),
                                  reinterpret_cast<const surface_handle *>(&surface),
                                  reinterpret_cast<const expolygon_collection_handle *>(&recipe.no_overlap_areas),
                                  &recipe.raw);
        throw_if_cancelled(run_ctx);
    }

    // A late modifier cannot silently replace the prepared bridge's directional contract.
    BridgePreparation::validate_recipe(reinterpret_cast<const surface_handle *>(&surface),recipe.raw,
        ctx.pattern_plugin_id==nullptr ? nullptr : ctx.pattern_plugin_id(&ctx,recipe.raw.pattern_id),
        reinterpret_cast<const layer_region_handle *>(&region));

    // Enforce full fill volume: measure the free area this surface expects, so
    // a STEP_POST_INFILL plugin can compute the volume target and correct the
    // generated extrusion. Sparse surfaces are never corrected, even at 100%
    // density, so the gate requires a solid surface on top of the legacy
    // FillParams check (FillBase.cpp:437): !dont_adjust && full_infill() &&
    // !flow.bridge() && fill_exactly.
    recipe.area_to_fill_mm2 = 0;
    if ((recipe.raw.infill_flags & C_INFILL_FLAG_FILL_EXACTLY) != 0 &&
        (recipe.raw.infill_flags & C_INFILL_FLAG_DONT_ADJUST) == 0 && !bridge &&
        surface.has_fill_solid() &&
        recipe.raw.density > 0.9999f && recipe.raw.density < 1.0001f) {
        // An empty free area means the whole surface is available, like the
        // legacy fallback in compute_unscaled_volume_to_fill().
        if (recipe.no_overlap_areas.empty())
            recipe.area_to_fill_mm2 = Slic3r::unscaled(Slic3r::unscaled(surface.area()));
        else
            for (const Slic3r::ExPolygon &expoly : recipe.no_overlap_areas)
                recipe.area_to_fill_mm2 += Slic3r::unscaled(Slic3r::unscaled(expoly.area()));
    }

    return recipe;
}

const Slic3r::LayerRegion *primary_region(const Slic3r::LayerRegionIsland &region_island)
{
    if (region_island.regions().empty())
        return nullptr;
    return *region_island.regions().begin();
}

bool surface_needs_infill(const Slic3r::Surface &surface)
{
    return !surface.empty() && !surface.has_fill_void();
}

// Byte-exact key used to group surfaces that share one raw recipe. Padding is
// deterministic because make_surface_recipe() zeroes the struct first.
std::string params_key(const raw_infill_pattern_params &params)
{
    return std::string(reinterpret_cast<const char *>(&params), sizeof(params));
}

void generate_surface_from_recipe(const run_ctx_generate_infill &ctx,
                                  const Slic3r::Layer &layer,
                                  const Slic3r::LayerSliceIsland &island,
                                  const Slic3r::LayerRegionIsland &region_island,
                                  const Slic3r::LayerRegion &region,
                                  const Slic3r::Surface &surface,
                                  infill_pattern_runtime_id pattern_id,
                                  const raw_infill_pattern_params &params,
                                  const Slic3r::ExPolygons &no_overlap_areas,
                                  double area_to_fill_mm2,
                                  const plugin_run_context *run_ctx)
{
    Slic3r::ExtrusionEntity output(Slic3r::ExtrusionEntity::Children{}, true, true, false);
    output.set_can_sort_reverse(false, false);

    throw_if_cancelled(run_ctx);
    const int32_t generated = ctx.generate_pattern(&ctx,
                                                   pattern_id,
                                                   reinterpret_cast<const layer_handle *>(&layer),
                                                   reinterpret_cast<const layer_island_handle *>(&island),
                                                   reinterpret_cast<const layer_region_island_handle *>(&region_island),
                                                   reinterpret_cast<const layer_region_handle *>(&region),
                                                   reinterpret_cast<const surface_handle *>(&surface),
                                                   reinterpret_cast<const expolygon_collection_handle *>(&no_overlap_areas),
                                                   &params,
                                                   reinterpret_cast<extrusion_entity_handle *>(&output));
    throw_if_cancelled(run_ctx);
    // A successful solid pattern may leave the whole narrow surface uncovered.
    // Keep its recipe until post-infill can turn that residual into gap fill.
    if (!generated || (output.empty() && (!surface.has_fill_solid() || surface.has_mod_bridge())))
        return;

    if (surface.get_property<BridgeSurfaceInfo>() != nullptr)
        restore_prepared_bridge_dimensions(output, params.flow, Slic3r::infill_bridge_flow(layer, region, Slic3r::frSolidInfill), run_ctx);

    // Keep a direct link from the extrusion subtree to the Surface recipe that
    // produced it, together with useful data for next plugins. The property is inherited by
    // every child path, so later post-infill plugins can split or reorder paths
    // and still recover the recipe from the closest parent.
    throw_if_cancelled(run_ctx);
    Slic3r::ExtrusionPropertyInfill &infill_property =
        output.get_or_add_property<Slic3r::ExtrusionPropertyInfill>();
    infill_property.source_surface_id = surface.id();
    infill_property.area_to_fill_mm2 = area_to_fill_mm2;
    infill_property.height = params.flow.height;
    infill_property.spacing = params.flow.spacing;
    infill_property.spacing_ratio = params.flow.spacing_ratio;
    infill_property.density = params.density;
    infill_property.flow_mult = params.flow_mult;
    infill_property.surface_type = params.surface_type;
    infill_property.infill_flags = params.infill_flags;

    // The data tree is read-only from the plugin point of view, but this host
    // callback is explicitly the publication channel. The const_cast is limited
    // to building the opaque handle expected by that callback; the plugin never
    // mutates the LayerRegionIsland directly.
    throw_if_cancelled(run_ctx);
    ctx.append_region_island_extrusion(
        reinterpret_cast<layer_region_island_handle *>(const_cast<Slic3r::LayerRegionIsland *>(&region_island)),
        params.extrusion_role,
        reinterpret_cast<extrusion_entity_handle *>(&output));
}

void restore_prepared_bridge_dimensions(Slic3r::ExtrusionEntity &output, const c_flow &transport,
                                       const Slic3r::Flow &native, const plugin_run_context *run_ctx)
{
    std::vector<Slic3r::ExtrusionEntity *> pending{&output};
    while (!pending.empty()) {
        throw_if_cancelled(run_ctx);
        Slic3r::ExtrusionEntity &entity = *pending.back();
        pending.pop_back();
        if (Slic3r::ExtrusionAttributes *attributes = entity.get_property<Slic3r::ExtrusionAttributes>()) {
            if ((attributes->role == RAW_EXTRUSION_ROLE_BRIDGE_INFILL || attributes->role == RAW_EXTRUSION_ROLE_INTERNAL_BRIDGE_INFILL) && attributes->mm3_per_mm == transport.mm3_per_mm &&
                attributes->width == float(unscaled(transport.width)) && attributes->height == float(unscaled(transport.height))) {
                // Transported height uses the layer grid. Recover native circular dimensions
                // only when the attributes still match that transport, without changing flow area.
                attributes->width = native.width();
                attributes->height = native.height();
            }
        }
        // Read children through the const overload: the mutable accessor wraps
        // a leaf in a new collection and would keep extending this traversal.
        for (const Slic3r::ExtrusionEntityUPtr &child : static_cast<const Slic3r::ExtrusionEntity &>(entity).children())
            if (child)
                pending.push_back(child.get());
    }
}

} // namespace

DefaultInfillGenerator &
DefaultInfillGenerator::instance(orchestrator_handle *orch)
{
    static DefaultInfillGenerator s_instance(orch);
    return s_instance;
}

const char *DefaultInfillGenerator::id_impl() const noexcept
{
    return k_default_infill_generator_id;
}

const char *DefaultInfillGenerator::name_impl() const noexcept
{
    return "Default infill generator";
}

const char *DefaultInfillGenerator::description_impl() const noexcept
{
    return "Builds infill recipes from LayerRegionIsland surfaces and delegates pattern extrusion to active infill pattern plugins.";
}

slicing_step_t DefaultInfillGenerator::step_impl() const noexcept
{
    return STEP_INFILL;
}

const char *const *DefaultInfillGenerator::dependencies_impl() const noexcept
{
    return k_no_dependencies;
}

int32_t DefaultInfillGenerator::priority_impl() const noexcept
{
    return 0;
}

int32_t DefaultInfillGenerator::used_config_keys(raw_used_config_key *keys) const noexcept
{
    if (keys != nullptr)
        for (size_t index = 0; index < sizeof(k_used_config_keys) / sizeof(k_used_config_keys[0]); ++index)
            keys[index] = k_used_config_keys[index];
    return int32_t(sizeof(k_used_config_keys) / sizeof(k_used_config_keys[0]));
}

int32_t DefaultInfillGenerator::defined_config_keys(const char **keys) const noexcept
{
    if (keys != nullptr)
        for (size_t index = 0; index < sizeof(k_defined_config_keys) / sizeof(k_defined_config_keys[0]); ++index)
            keys[index] = k_defined_config_keys[index];
    return int32_t(sizeof(k_defined_config_keys) / sizeof(k_defined_config_keys[0]));
}

void DefaultInfillGenerator::inilialize_impl(storage_handle *) const
{
    create_infill_pattern_options(m_orchestrator);
    register_infill_pattern_rules(m_orchestrator);
}

const char *DefaultInfillGenerator::progress_message_format_impl() const noexcept
{
    return "Generating infill: %u / %u surfaces";
}

void DefaultInfillGenerator::setup_run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_generate_infill *ctx = plugin_ctx_as_generate_infill(run_ctx);
    if (ctx == nullptr || ctx->print == nullptr || ctx->object == nullptr)
        return;

    const Slic3r::Print &print = *reinterpret_cast<const Slic3r::Print *>(ctx->print);
    const Slic3r::PrintObject &object = *reinterpret_cast<const Slic3r::PrintObject *>(ctx->object);

    // Discovery pass. Surfaces of one region island are grouped by their full
    // raw parameters, so a group is exactly the set of surfaces that one
    // pattern initialization must cover. The groups are kept for run().
    std::shared_ptr<DefaultInfillObjectWorkCache> cache = std::make_shared<DefaultInfillObjectWorkCache>();
    uint32_t surface_count = 0;
    for (const Slic3r::Layer &layer : object.layers()) {
        throw_if_cancelled(run_ctx);
        for (const Slic3r::LayerSliceIsland &island : layer.islands()) {
            throw_if_cancelled(run_ctx);
            for (const Slic3r::LayerRegionIsland &region_island : island.regions_islands()) {
                throw_if_cancelled(run_ctx);
                const Slic3r::LayerRegion *region = primary_region(region_island);
                if (region == nullptr)
                    continue;

                std::map<std::string, size_t> group_index_by_key;
                for (const Slic3r::Surface &surface : region_island.fill_surfaces()) {
                    throw_if_cancelled(run_ctx);
                    if (!surface_needs_infill(surface))
                        continue;

                    SurfaceInfillRecipe recipe =
                        make_surface_recipe(*ctx, print, object, layer, island, region_island, *region, surface, run_ctx);
                    if (recipe.raw.pattern_id == INFILL_PATTERN_RUNTIME_ID_INVALID || recipe.raw.density <= 0.f)
                        continue;

                    ++surface_count;
                    const std::string key = params_key(recipe.raw);
                    const std::map<std::string, size_t>::const_iterator found = group_index_by_key.find(key);
                    size_t group_index;
                    if (found == group_index_by_key.end()) {
                        group_index = cache->groups.size();
                        group_index_by_key.emplace(key, group_index);
                        cache->groups.emplace_back();
                        DefaultInfillWorkGroup &group = cache->groups.back();
                        group.layer = &layer;
                        group.island = &island;
                        group.region_island = &region_island;
                        group.region = region;
                        group.pattern_id = recipe.raw.pattern_id;
                        group.params = recipe.raw;
                    } else {
                        group_index = found->second;
                    }

                    DefaultInfillSurfaceWork work;
                    work.surface = &surface;
                    work.no_overlap_areas = std::move(recipe.no_overlap_areas);
                    work.area_to_fill_mm2 = recipe.area_to_fill_mm2;
                    cache->groups[group_index].surfaces.emplace_back(std::move(work));
                }
            }
        }
    }

    progress().add_max(surface_count);

    // Report the groups per pattern plugin. The host copies everything, but
    // the params pointers must stay valid until run() for this object, which is
    // guaranteed by storing the cache in m_object_caches.
    if (ctx->report_pattern_assignments != nullptr) {
        std::map<infill_pattern_runtime_id, std::vector<raw_infill_pattern_assignment>> assignments_by_pattern;
        std::vector<std::vector<const surface_handle *>> surface_lists;
        surface_lists.reserve(cache->groups.size());
        for (const DefaultInfillWorkGroup &group : cache->groups) {
            std::vector<const surface_handle *> surfaces;
            surfaces.reserve(group.surfaces.size());
            for (const DefaultInfillSurfaceWork &work : group.surfaces)
                surfaces.emplace_back(reinterpret_cast<const surface_handle *>(work.surface));
            surface_lists.emplace_back(std::move(surfaces));

            raw_infill_pattern_assignment assignment = {};
            assignment.layer_island = reinterpret_cast<const layer_island_handle *>(group.island);
            assignment.region_island = reinterpret_cast<const layer_region_island_handle *>(group.region_island);
            assignment.params = &group.params;
            assignment.surfaces = surface_lists.back().data();
            assignment.surface_count = uint32_t(surface_lists.back().size());
            assignments_by_pattern[group.pattern_id].emplace_back(assignment);
        }

        for (const std::pair<const infill_pattern_runtime_id, std::vector<raw_infill_pattern_assignment>> &entry :
             assignments_by_pattern) {
            if (entry.second.empty())
                continue;
            ctx->report_pattern_assignments(ctx, entry.first, entry.second.data(), uint32_t(entry.second.size()));
        }
    }

    m_object_caches[&object] = std::move(cache);
}

void DefaultInfillGenerator::run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_generate_infill *ctx = plugin_ctx_as_generate_infill(run_ctx);
    assert(ctx != nullptr);
    if (ctx == nullptr || ctx->print == nullptr || ctx->object == nullptr ||
        ctx->generate_pattern == nullptr || ctx->append_region_island_extrusion == nullptr) {
        return;
    }

    const Slic3r::Print &print = *reinterpret_cast<const Slic3r::Print *>(ctx->print);
    const Slic3r::PrintObject &object = *reinterpret_cast<const Slic3r::PrintObject *>(ctx->object);

    const std::map<const Slic3r::PrintObject *, std::shared_ptr<DefaultInfillObjectWorkCache>>::iterator cached =
        m_object_caches.find(&object);
    if (cached != m_object_caches.end() && cached->second) {
        for (const DefaultInfillWorkGroup &group : cached->second->groups) {
            for (const DefaultInfillSurfaceWork &work : group.surfaces) {
                throw_if_cancelled(run_ctx);
                generate_surface_from_recipe(*ctx, *group.layer, *group.island, *group.region_island, *group.region,
                                             *work.surface, group.pattern_id, group.params, work.no_overlap_areas,
                                             work.area_to_fill_mm2, run_ctx);
                progress().increment();
            }
        }
        // The recipes are no longer needed once the object has been filled.
        m_object_caches.erase(cached);
        return;
    }

    // Defensive fallback for a generator run without a prior discovery pass:
    // rebuild each recipe on the fly.
    for (const Slic3r::Layer &layer : object.layers()) {
        throw_if_cancelled(run_ctx);
        for (const Slic3r::LayerSliceIsland &island : layer.islands()) {
            throw_if_cancelled(run_ctx);
            for (const Slic3r::LayerRegionIsland &region_island : island.regions_islands()) {
                throw_if_cancelled(run_ctx);
                const Slic3r::LayerRegion *region = primary_region(region_island);
                if (region == nullptr)
                    continue;

                for (const Slic3r::Surface &surface : region_island.fill_surfaces()) {
                    throw_if_cancelled(run_ctx);
                    if (!surface_needs_infill(surface))
                        continue;

                    SurfaceInfillRecipe recipe =
                        make_surface_recipe(*ctx, print, object, layer, island, region_island, *region, surface, run_ctx);
                    progress().increment();
                    if (recipe.raw.pattern_id == INFILL_PATTERN_RUNTIME_ID_INVALID || recipe.raw.density <= 0.f)
                        continue;

                    generate_surface_from_recipe(*ctx, layer, island, region_island, *region, surface,
                                                 recipe.raw.pattern_id, recipe.raw, recipe.no_overlap_areas,
                                                 recipe.area_to_fill_mm2, run_ctx);
                }
            }
        }
    }
}

void register_default_infill_generator_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, DefaultInfillGenerator::instance(orch).c_instance());
}

}}} // namespace slic3r_api::Infill::DefaultInfillGeneratorPlugin
