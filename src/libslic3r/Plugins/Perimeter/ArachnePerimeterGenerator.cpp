///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
ArachnePerimeterGenerator.cpp
=============================

Implementation of the Arachne perimeter generator declared in
ArachnePerimeterGenerator.hpp.

The file holds the plugin metadata, the owned option definition and activation
rule, the Arachne-to-host conversion helpers, the run callbacks and the
registration entry point. run() converts host nodes into Arachne wall passes and
publishes the closed and open extrusion entities back into the node property
collections.
*/

#include "ArachnePerimeterGenerator.hpp"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <iterator>
#include <limits>
#include <utility>
#include <vector>

#include "libslic3r/Arachne/WallToolPaths.hpp"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_perimeter.h"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/GuiRuleHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/RegionSettingsViews.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionGeometryUnits.hpp"
#include "libslic3r/ExtrusionProperty.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Geometry/MedialAxis.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintRegion.hpp"

/*
Arachne perimeter generation
============================

This plugin provides the Arachne implementation of STEP_PERIMETER. The host
invokes it for one LayerSliceIsland and supplies the callback
run_region_group(), which owns the traversal of one region group's perimeter
tree and calls generate_node() for each node that needs to be produced.

One perimeter tree uses the flows, perimeter count, extruder, loop direction
and Arachne wall settings of a single region, and the host publishes each
region group in a LayerRegionIsland keyed by its extruder. Regions of the
island that disagree on these settings therefore must not share a group.
RegionSettings, read through the public LayerIsland view of the island, cuts
the island slice into the largest areas whose regions agree on them. Each
connected piece of such an area becomes the root of one region group, whose
generation state comes from the group's first region. When all regions agree,
the island stays one group rooted at its whole slice.

generate_node() converts one node area into variable-width Arachne wall paths.
It applies the configured perimeter count, selects external or internal flow,
assigns perimeter and loop properties, and stores generic owned nodes for
closed contours or reversible open sequences. It also computes the inner
areas and inner fill areas that the following perimeter and surface stages
will use. Invalid or empty geometry is returned without creating extrusion.

The normal call flow is:

    register_arachne_perimeter_generator_plugin()
    `-- ArachnePerimeterGenerator::instance()
        `-- orchestrator_register_plugin()

    ArachnePerimeterGenerator::setup_run_impl()
    `-- reserve one progress unit for the island

    ArachnePerimeterGenerator::run_impl()
    `-- validate and unwrap the perimeter context
        |-- RegionSettings::segregate(): areas whose regions agree on the settings
        `-- for each area and each connected piece of it
            |-- build ArachneGeneratorState from the group's first region
            |   `-- resolve layer, print, flows, and requested perimeter count
            `-- run_region_group(group regions, piece, &generate_node)
                `-- generate_node() for each perimeter tree node
                    |-- build Arachne::WallToolPaths from the node area
                    |-- make_arachne_extrusions()
                    |   |-- convert variable-width lines to paths
                    |   `-- append generic closed contours or open sequences
                    |       `-- make_path_sequence(): transfer attributed leaves and set parent permissions
                    |-- move generated extrusion into the node
                    |-- compute inner areas and fill areas
                    `-- publish both area collections to the host

Closed-loop direction follows the configured contour and hole direction,
including the optional odd-layer reversal. Loop endpoints that are only
epsilon-close are normalized before publication. The plugin is responsible
for Arachne geometry conversion, while tree traversal and final storage remain
owned by the perimeter step context.
Constructive path values retain their direct flow and perimeter metadata when
their base content moves into generic leaves. Closed contours are fixed and
rotatable; open sequences may reverse as a unit but may not rotate. Owned scratch
trees release unpublished output on cancellation or allocation failure.
*/

namespace slic3r_api { namespace Perimeter { namespace ArachnePerimeterGeneratorPlugin {

namespace {

const char *k_arachne_perimeter_generator_id = "perimeter.generator.arachne";
const char *k_no_dependencies[] = { nullptr };

const char *k_perimeters_key = "perimeters";
const char *k_perimeter_reverse_key = "perimeter_reverse";
const char *k_perimeter_extruder_key = "perimeter_extruder";
const char *k_external_perimeter_width_key = "external_perimeter_extrusion_width";
const char *k_perimeter_width_key = "perimeter_extrusion_width";
const char *k_external_perimeter_spacing_key = "external_perimeter_extrusion_spacing";
const char *k_external_perimeter_odd_layers_key = "external_perimeter_extrusion_change_odd_layers";
const char *k_external_perimeter_overlap_key = "external_perimeter_overlap";
const char *k_perimeter_spacing_key = "perimeter_extrusion_spacing";
const char *k_perimeter_odd_layers_key = "perimeter_extrusion_change_odd_layers";
const char *k_perimeter_overlap_key = "perimeter_overlap";
const char *k_perimeter_direction_key = "perimeter_direction";
const char *k_min_bead_width_key = "min_bead_width";
const char *k_min_feature_size_key = "min_feature_size";
const char *k_wall_distribution_count_key = "wall_distribution_count";
const char *k_wall_transition_angle_key = "wall_transition_angle";
const char *k_wall_transition_filter_deviation_key = "wall_transition_filter_deviation";
const char *k_wall_transition_length_key = "wall_transition_length";

// used_config_keys is the plugin declaration: the host activates these settings
// while this implementation is selected and invalidates its caches when they
// change. Arachne reads the wall count and the reversal directly, and splits
// islands by the extruder and the widths, like the classic generator, which
// declares the same keys. The other flow settings, the loop direction and the
// Arachne wall settings are compared but not declared: a selector rule of
// this generator would enable them even where their core activation rules
// disable them.
const raw_used_config_key k_used_config_keys[] = {
    { k_perimeters_key, RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { k_perimeter_reverse_key, RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { k_perimeter_extruder_key, RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { k_external_perimeter_width_key, RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { k_perimeter_width_key, RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE }
};

const char *const k_defined_config_keys[] = {
    k_perimeter_reverse_key
};

// Transfer constructive path values into generic owned leaves with explicit parent permissions.
Slic3r::ExtrusionEntity make_path_sequence(Slic3r::ExtrusionPaths &&paths, bool can_reverse, bool can_rotate);

// Append one reversible fixed-start sequence, retaining each constructive leaf's permissions.
void append_open_paths(Slic3r::ExtrusionEntity &dst, Slic3r::ExtrusionPaths &&paths);

// Convert and annotate one Arachne line, normalizing closed endpoints before ownership enters dst.
void append_arachne_line(Slic3r::ExtrusionEntity &dst,
                         const Slic3r::Arachne::ExtrusionLine &line,
                         size_t biggest_inset_idx, size_t inset_offset,
                         const Slic3r::Layer &layer,
                         const Slic3r::PrintRegionConfig &region_config,
                         const Slic3r::PrintConfig &print_config,
                         const Slic3r::Flow &external_flow, const Slic3r::Flow &internal_flow,
                         const plugin_run_context *run_ctx);

// Build one owned generic tree in Arachne's path order, checking cancellation between lines.
Slic3r::ExtrusionEntity make_arachne_extrusions(
    const std::vector<Slic3r::Arachne::VariableWidthLines> &perimeters, size_t inset_offset,
    const Slic3r::Layer &layer, const Slic3r::PrintRegionConfig &region_config,
    const Slic3r::PrintConfig &print_config,
    const Slic3r::Flow &external_flow, const Slic3r::Flow &internal_flow,
    const plugin_run_context *run_ctx);

void initialize_arachne_perimeter_settings(orchestrator_handle *orchestrator)
{
    /*
    Option definitions owned by this module. Each entry fixes the option type,
    override scope, default and GUI metadata; defined keys are published in the
    print presets, so profiles read and write them by name like any built-in
    setting. Called once from inilialize_impl().
    */
    raw_config_option_def def = raw_config_option_def_init();

    // ---- perimeter_reverse | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "perimeter_reverse";
    def.type = RAW_CO_BOOL;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_PERIMETER;
    def.category = RAW_OPTION_CATEGORY_PERIMETER;
    def.label = "Perimeter reversal on even layers";
    def.tooltip = "On even layers, all perimeter loops are reversed (it disables the overhang reversal, so it doesn't double-reverse).That setting will likely create defects on the perimeters, so it's only useful is for materials that have some direction-dependent properties (stress lines).";
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "0";
    orchestrator_create_option_def(orchestrator, &def);
}

void add_arachne_perimeter_gui_rules(orchestrator_handle *orchestrator)
{
    // Perimeter reversal needs the Arachne generator and a non-zero perimeter count.
    GuiRule::activate({ "perimeter_reverse" })
        .when(non_zero("perimeters"))
        .scope(RAW_PRESET_TYPE_FFF_PRINT)
        .add_to(orchestrator);
}
const Slic3r::Layer *to_layer(const layer_handle *handle)
{
    return reinterpret_cast<const Slic3r::Layer *>(handle);
}

const Slic3r::Print *to_print(const print_handle *handle)
{
    return reinterpret_cast<const Slic3r::Print *>(handle);
}

const Slic3r::LayerRegion *to_layer_region(const layer_region_handle *handle)
{
    return reinterpret_cast<const Slic3r::LayerRegion *>(handle);
}

Slic3r::Flow perimeter_flow(const Slic3r::LayerRegion &region, const bool external)
{
    return region.flow(external ? Slic3r::frExternalPerimeter : Slic3r::frPerimeter);
}

size_t max_inset_idx(const std::vector<Slic3r::Arachne::VariableWidthLines> &perimeters,
                     const plugin_run_context *run_ctx)
{
    size_t out = 0;
    for (const Slic3r::Arachne::VariableWidthLines &perimeter : perimeters) {
        throw_if_cancelled(run_ctx);
        for (const Slic3r::Arachne::ExtrusionLine &line : perimeter)
            out = std::max(out, line.inset_idx);
    }
    return out;
}

Slic3r::ExtrusionLoopRole loop_role_for_line(const Slic3r::Arachne::ExtrusionLine &line,
                                             const size_t biggest_inset_idx)
{
    Slic3r::ExtrusionLoopRole loop_role = Slic3r::elrDefault;
    if (line.inset_idx == biggest_inset_idx)
        loop_role = Slic3r::ExtrusionLoopRole(loop_role | Slic3r::elrInternal | Slic3r::elrFirstLoop);
    if (!line.is_contour())
        loop_role = Slic3r::ExtrusionLoopRole(loop_role | Slic3r::elrHole);
    return loop_role;
}

bool line_is_closed(const Slic3r::Arachne::ExtrusionLine &line)
{
    return line.is_closed ||
           (!line.junctions.empty() && line.junctions.front().p == line.junctions.back().p);
}

Slic3r::ExtrusionPaths variable_width_paths(const Slic3r::Arachne::ExtrusionLine &line,
                                            const Slic3r::ExtrusionRole role,
                                            const Slic3r::Flow &flow,
                                            const Slic3r::PrintConfig &print_config,
                                            const plugin_run_context *run_ctx)
{
    if (line.size() < 2 || line.is_zero_length())
        return {};

    Slic3r::ThickPolyline thick_polyline = Slic3r::Arachne::to_thick_polyline(line);
    if (thick_polyline.points.size() < 2)
        return {};

    const Slic3r::coord_t resolution = std::max(flow.scaled_width() / 4, scale_i(print_config.resolution.value));
    const Slic3r::coord_t tolerance  = std::max<Slic3r::coord_t>(1, flow.scaled_width() / 10);
    throw_if_cancelled(run_ctx);
    Slic3r::ExtrusionPaths paths =
        Slic3r::Geometry::unsafe_variable_width(thick_polyline, role, flow, resolution, tolerance);
    throw_if_cancelled(run_ctx);
    return paths;
}

void annotate_paths(Slic3r::ExtrusionPaths &paths,
                    const size_t shell_idx,
                    const Slic3r::ExtrusionLoopRole loop_role)
{
    const int16_t shell_count = int16_t(std::min<size_t>(shell_idx, size_t(std::numeric_limits<int16_t>::max())));
    for (Slic3r::ExtrusionPath &path : paths) {
        path.get_or_add_property<EPropertyPerimeter>()
            .shell_count(shell_count)
            .perimeter_flags(uint16_t(loop_role));
    }
}

void append_open_paths(Slic3r::ExtrusionEntity &dst, Slic3r::ExtrusionPaths &&paths)
{
    if (paths.empty())
        return;

    for (size_t idx = 1; idx < paths.size(); ++idx)
        paths[idx].set_can_reverse(false);
    dst.append_child(std::make_unique<Slic3r::ExtrusionEntity>(make_path_sequence(std::move(paths), true, false)));
}

void append_arachne_line(Slic3r::ExtrusionEntity &dst,
                         const Slic3r::Arachne::ExtrusionLine &line,
                         const size_t biggest_inset_idx,
                         const size_t inset_offset,
                         const Slic3r::Layer &layer,
                         const Slic3r::PrintRegionConfig &region_config,
                         const Slic3r::PrintConfig &print_config,
                         const Slic3r::Flow &external_flow,
                         const Slic3r::Flow &internal_flow,
                         const plugin_run_context *run_ctx)
{
    if (line.size() < 2 || line.is_zero_length())
        return;

    const size_t absolute_inset_idx = inset_offset + line.inset_idx;
    const bool is_external = absolute_inset_idx == 0;
    const bool is_contour = line.is_contour();
    const bool is_closed = line_is_closed(line);
    const Slic3r::ExtrusionRole role = is_external ?
        Slic3r::ExtrusionRole::ExternalPerimeter :
        Slic3r::ExtrusionRole::Perimeter;
    const Slic3r::Flow &flow = is_external ? external_flow : internal_flow;
    Slic3r::ExtrusionLoopRole loop_role = loop_role_for_line(line, biggest_inset_idx);
    Slic3r::ExtrusionPaths paths = variable_width_paths(line, role, flow, print_config, run_ctx);
    if (paths.empty())
        return;

    annotate_paths(paths, absolute_inset_idx, loop_role);

    if (is_closed && paths.back().last_point().coincides_with_epsilon(paths.front().first_point())) {
        Slic3r::ExtrusionEntity loop = make_path_sequence(std::move(paths), false, true);
        // STEP_ORDERING validates the direct metadata of terminal loops. The
        // path annotations above remain useful to path consumers, but the loop
        // itself must retain the same shell and perimeter classification.
        Slic3r::ExtrusionPropertyLoopRole &loop_property =
            loop.get_or_add_property<Slic3r::ExtrusionPropertyLoopRole>();
        loop_property.perimeter_idx = int16_t(std::min<size_t>(
            absolute_inset_idx, size_t(std::numeric_limits<int16_t>::max())));
        loop_property.set_perimeter_role(loop_role);
        const bool ccw_contour = region_config.perimeter_direction.value == Slic3r::PerimeterDirection::pdCCW_CW ||
                                 region_config.perimeter_direction.value == Slic3r::PerimeterDirection::pdCCW_CCW;
        const bool ccw_hole = region_config.perimeter_direction.value == Slic3r::PerimeterDirection::pdCW_CCW ||
                              region_config.perimeter_direction.value == Slic3r::PerimeterDirection::pdCCW_CCW;
        const Slic3r::ConfigOption *perimeter_reverse_opt = region_config.option("perimeter_reverse");
        const bool need_ccw = ((perimeter_reverse_opt != nullptr && perimeter_reverse_opt->get_bool() && layer.id() % 2 == 1) ==
                               (is_contour ? ccw_contour : ccw_hole));
        if (need_ccw != !(Slic3r::extrusion_geometry_area(loop) > 0.))
            loop.reverse();

        // Arachne may close a loop with points that are epsilon-close but not
        // bit-identical. Normalize the stored endpoint before later code
        // relies on exact loop closure.
        if (loop.child_count() != 0)
            loop.child(0).polyline_ref().set_front(loop.child(loop.child_count() - 1).polyline_ref().back());
        dst.append_child(std::make_unique<Slic3r::ExtrusionEntity>(std::move(loop)));
        return;
    }

    append_open_paths(dst, std::move(paths));
}

Slic3r::ExtrusionEntity make_arachne_extrusions(
    const std::vector<Slic3r::Arachne::VariableWidthLines> &perimeters,
    const size_t inset_offset,
    const Slic3r::Layer &layer,
    const Slic3r::PrintRegionConfig &region_config,
    const Slic3r::PrintConfig &print_config,
    const Slic3r::Flow &external_flow,
    const Slic3r::Flow &internal_flow,
    const plugin_run_context *run_ctx)
{
    Slic3r::ExtrusionEntity extrusion(Slic3r::ExtrusionEntity::Children(), true, true, false);
    const size_t biggest_inset_idx = max_inset_idx(perimeters, run_ctx);
    for (const Slic3r::Arachne::VariableWidthLines &perimeter : perimeters) {
        throw_if_cancelled(run_ctx);
        for (const Slic3r::Arachne::ExtrusionLine &line : perimeter)
            append_arachne_line(extrusion, line, biggest_inset_idx, inset_offset, layer, region_config, print_config,
                                external_flow, internal_flow, run_ctx);
    }
    return extrusion;
}

Slic3r::ExtrusionEntity make_path_sequence(Slic3r::ExtrusionPaths &&paths, bool can_reverse, bool can_rotate)
{
    Slic3r::ExtrusionEntity sequence(Slic3r::ExtrusionEntity::Children(), false, can_reverse, true);
    sequence.set_can_rotate(can_rotate);
    for (Slic3r::ExtrusionPath &path : paths)
        sequence.append_child(std::make_unique<Slic3r::ExtrusionEntity>(std::move(static_cast<Slic3r::ExtrusionEntity &>(path))));
    return sequence;
}

struct ArachneGeneratorState
{
    const Slic3r::Layer *layer = nullptr;
    const Slic3r::Print *print = nullptr;
    const Slic3r::LayerRegion *region = nullptr;
    Slic3r::Flow external_flow;
    Slic3r::Flow internal_flow;
    size_t perimeter_count = 0;
};

const Slic3r::ExPolygon *node_area(const perimeter_node &node)
{
    return reinterpret_cast<const Slic3r::ExPolygon *>(node.area);
}

Slic3r::ExPolygons *to_expolygons(expolygon_collection_handle *handle)
{
    return reinterpret_cast<Slic3r::ExPolygons *>(handle);
}

int32_t generate_node(void *generator_context,
                      perimeter_generation_context *context,
                      perimeter_node *node,
                      expolygon_collection_handle *inner_areas_out,
                      expolygon_collection_handle *inner_fill_areas_out)
{
    const ArachneGeneratorState *state = reinterpret_cast<const ArachneGeneratorState *>(generator_context);
    if (state == nullptr || state->layer == nullptr || state->print == nullptr || state->region == nullptr ||
        context == nullptr || context->run_ctx == nullptr ||
        node == nullptr || node->extrusions == nullptr || inner_areas_out == nullptr ||
        inner_fill_areas_out == nullptr)
        return 0;

    throw_if_cancelled(context->run_ctx);

    const Slic3r::ExPolygon *area = node_area(*node);
    if (area == nullptr || area->empty())
        return 1;

    if (state->perimeter_count > 0)
        node->perimeter_needed = std::max<uint32_t>(node->perimeter_needed, uint32_t(state->perimeter_count));
    const bool is_external = node->perimeter_idx == 0;
    const Slic3r::Flow &outer_flow = is_external ? state->external_flow : state->internal_flow;
    const Slic3r::Flow &inner_flow = state->internal_flow;
    const Slic3r::PrintRegionConfig &region_config = state->region->region().config();
    const Slic3r::PrintConfig &print_config = state->print->config();

    Slic3r::ExPolygons fill_no_overlap = Slic3r::ExPolygons{ *area };
    if (node->perimeter_needed > 0) {
        Slic3r::Polygons outlines = Slic3r::to_polygons(*area);
        throw_if_cancelled(context->run_ctx);
        Slic3r::Arachne::WallToolPaths wall_tool_paths(outlines,
                                                       outer_flow.scaled_spacing(),
                                                       outer_flow.scaled_width(),
                                                       inner_flow.scaled_spacing(),
                                                       inner_flow.scaled_width(),
                                                       1,
                                                       Slic3r::coord_t(0),
                                                       state->layer->unscaled_height(),
                                                       region_config,
                                                       print_config);
        throw_if_cancelled(context->run_ctx);
        const std::vector<Slic3r::Arachne::VariableWidthLines> &perimeters =
            wall_tool_paths.getToolPaths();
        throw_if_cancelled(context->run_ctx);
        Slic3r::ExtrusionEntity extrusion =
            make_arachne_extrusions(perimeters, node->perimeter_idx, *state->layer, region_config, print_config,
                                    outer_flow, inner_flow, context->run_ctx);
        throw_if_cancelled(context->run_ctx);
        extrusion_move_from(node->extrusions, reinterpret_cast<extrusion_entity_handle *>(&extrusion));

        throw_if_cancelled(context->run_ctx);
        fill_no_overlap = Slic3r::union_ex(wall_tool_paths.getInnerContour());
        throw_if_cancelled(context->run_ctx);
        if (fill_no_overlap.empty())
            fill_no_overlap = Slic3r::ExPolygons{ *area };
    }

    throw_if_cancelled(context->run_ctx);
    Slic3r::ExPolygons fill_areas = Slic3r::offset_ex(
        fill_no_overlap, 0.25 * double(inner_flow.scaled_spacing()));
    throw_if_cancelled(context->run_ctx);
    fill_no_overlap = Slic3r::ensure_valid(std::move(fill_no_overlap));

    throw_if_cancelled(context->run_ctx);
    *to_expolygons(inner_areas_out) = std::move(fill_no_overlap);
    *to_expolygons(inner_fill_areas_out) = std::move(fill_areas);
    return 1;
}

} // namespace

ArachnePerimeterGenerator &
ArachnePerimeterGenerator::instance(orchestrator_handle *orch)
{
    static ArachnePerimeterGenerator s_instance(orch);
    return s_instance;
}

const char *ArachnePerimeterGenerator::id_impl() const noexcept
{
    return k_arachne_perimeter_generator_id;
}

slicing_step_t ArachnePerimeterGenerator::step_impl() const noexcept
{
    return STEP_PERIMETER;
}

const char *const *ArachnePerimeterGenerator::dependencies_impl() const noexcept
{
    return k_no_dependencies;
}

int32_t ArachnePerimeterGenerator::priority_impl() const noexcept
{
    return 0;
}

int32_t ArachnePerimeterGenerator::used_config_keys(raw_used_config_key *keys) const noexcept
{
    if (keys != nullptr)
        for (uint32_t idx = 0; idx < sizeof(k_used_config_keys) / sizeof(k_used_config_keys[0]); ++idx)
            keys[idx] = k_used_config_keys[idx];
    return int32_t(sizeof(k_used_config_keys) / sizeof(k_used_config_keys[0]));
}

int32_t ArachnePerimeterGenerator::defined_config_keys(const char **keys) const noexcept
{
    if (keys != nullptr)
        for (size_t idx = 0; idx < std::size(k_defined_config_keys); ++idx)
            keys[idx] = k_defined_config_keys[idx];
    return int32_t(std::size(k_defined_config_keys));
}

void ArachnePerimeterGenerator::inilialize_impl(storage_handle *) const
{
    initialize_arachne_perimeter_settings(m_orchestrator);
    add_arachne_perimeter_gui_rules(m_orchestrator);
}

const char *ArachnePerimeterGenerator::progress_message_format_impl() const noexcept
{
    return "Arachne perimeter generator: %u / %u islands";
}

void ArachnePerimeterGenerator::setup_run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_generate_perimeter *ctx = plugin_ctx_as_generate_perimeter(run_ctx);
    if (ctx != nullptr && ctx->island != nullptr)
        progress().add_max(1);
}

void ArachnePerimeterGenerator::run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_generate_perimeter *ctx = plugin_ctx_as_generate_perimeter(run_ctx);
    if (ctx == nullptr || ctx->island == nullptr || ctx->layer == nullptr || ctx->print == nullptr ||
        ctx->run_region_group == nullptr)
        return;

    throw_if_cancelled(run_ctx);

    const LayerIsland island(ctx->island);
    const Slic3r::Layer *layer = to_layer(ctx->layer);
    const Slic3r::Print *print = to_print(ctx->print);
    if (island.region_count() == 0 || layer == nullptr || print == nullptr)
        return;

    // The flows read the widths, spacings, odd-layer changes and overlaps of
    // both perimeter roles, the extruder gives their nozzle, the loops follow
    // the direction and reversal settings, and WallToolPaths reads the wall
    // settings. Regions that agree on all of them share one tree; an
    // accept-all clip keeps the whole slice when every region agrees.
    RegionSettings settings(run_ctx->plugin_storage,
                            island,
                            {{ k_perimeters_key,
                               k_perimeter_extruder_key,
                               k_external_perimeter_width_key,
                               k_external_perimeter_spacing_key,
                               k_external_perimeter_odd_layers_key,
                               k_external_perimeter_overlap_key,
                               k_perimeter_width_key,
                               k_perimeter_spacing_key,
                               k_perimeter_odd_layers_key,
                               k_perimeter_overlap_key,
                               k_perimeter_direction_key,
                               k_perimeter_reverse_key,
                               k_min_bead_width_key,
                               k_min_feature_size_key,
                               k_wall_distribution_count_key,
                               k_wall_transition_angle_key,
                               k_wall_transition_filter_deviation_key,
                               k_wall_transition_length_key }});
    settings.segregate(island.slice());

    for (const auto &[values, clip] : settings.get_areas(k_perimeters_key)) {
        const std::vector<LayerRegion> &regions = settings.get_regions(k_perimeters_key, values);
        if (regions.empty())
            continue;

        std::vector<const layer_region_handle *> handles;
        handles.reserve(regions.size());
        for (const LayerRegion &group_region : regions)
            handles.push_back(group_region.handle());

        const Slic3r::LayerRegion *region = to_layer_region(regions.front().handle());
        ArachneGeneratorState state;
        state.layer = layer;
        state.print = print;
        state.region = region;
        state.external_flow = perimeter_flow(*region, true);
        state.internal_flow = perimeter_flow(*region, false);
        state.perimeter_count = region->region().config().perimeters.value <= 0 ?
            size_t(0) :
            size_t(region->region().config().perimeters.value);

        StoredExPolygonCollection root_areas = clip.intersections(island.slice());
        for (ExPolygon root_area : root_areas) {
            throw_if_cancelled(run_ctx);
            ctx->run_region_group(ctx,
                                  handles.data(),
                                  uint32_t(handles.size()),
                                  root_area.handle(),
                                  &state,
                                  &generate_node);
            throw_if_cancelled(run_ctx);
        }
    }

    progress().increment();
}

void register_arachne_perimeter_generator_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, ArachnePerimeterGenerator::instance(orch).c_instance());
}

}}} // namespace slic3r_api::Perimeter::ArachnePerimeterGeneratorPlugin

#ifdef ARACHNE_PERIMETER_GENERATOR_PLUGIN_DLL
extern "C" void register_plugin(orchestrator_handle *orch)
{
    slic3r_api::Perimeter::ArachnePerimeterGeneratorPlugin::register_arachne_perimeter_generator_plugin(orch);
}
#endif // ARACHNE_PERIMETER_GENERATOR_PLUGIN_DLL
