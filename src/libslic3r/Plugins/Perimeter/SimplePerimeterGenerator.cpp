///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "SimplePerimeterGenerator.hpp"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_perimeter.h"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PerimeterStepViews.hpp"

/*
Simple perimeter generation
===========================

This plugin provides the fixed-width implementation of STEP_PERIMETER. The
host invokes it for one LayerIsland and supplies a callback that traverses the
region group and calls generate_node() for each perimeter node. The plugin
uses the first region's external- and internal-perimeter flows and requested
perimeter count as the shared generation parameters for that island. The first
loop gets the external role and flow, the following loops the internal ones.

generate_node() first applies the requested perimeter count to the node. If no
additional perimeter is needed, it publishes the node area and fill area
unchanged. Otherwise, make_perimeter_extrusion() offsets the node area, creates
closed contour and hole loops, and stores them as reversible extrusion
subtrees. The callback then computes the inner perimeter area and the slightly
larger fill/anchor area for the following stages.

The normal call flow is:

    register_simple_perimeter_generator_plugin()
    `-- SimplePerimeterGenerator::instance()
        `-- orchestrator_register_plugin()

    SimplePerimeterGenerator::setup_run_impl()
    `-- reserve one progress unit for the island

    SimplePerimeterGenerator::run_impl()
    `-- validate the perimeter context and collect island regions
        |-- external_perimeter_flow(island)
        |-- internal_perimeter_flow(island)
        |-- perimeter_count(island)
        `-- run_region_group(..., &generate_node)
            `-- generate_node() for each perimeter node
                |-- apply the requested perimeter count
                |-- no more perimeters needed?
                |   `-- publish the existing inner and fill areas
                `-- otherwise
                    |-- make_perimeter_extrusion()
                    |   |-- offset the node area
                    |   |-- create contour and hole loops
                    |   `-- attach perimeter and extrusion attributes
                    |-- move the extrusion into the node
                    |-- compute inner perimeter area
                    `-- compute fill/anchor area

Only the perimeter geometry conversion is implemented here. Region traversal,
node ownership, and publication of the resulting area collections remain the
responsibility of the perimeter step context.
*/

namespace slic3r_api { namespace Perimeter { namespace SimplePerimeterGeneratorPlugin {

namespace {

// Copy a closed polygon into a storage-owned loop with shell flags on its wrapper
// and the selected external/internal role and flow on its child path.
void append_perimeter_loop(StoredExtrusionEntity &dst, const Polygon &polygon, const c_flow &flow,
                           raw_extrusion_role role, uint16_t perimeter_idx, uint16_t perimeter_flags);
// Offset one node and return its owned contour/hole loops. Cancellation and
// allocation errors propagate; the borrowed area is not changed.
StoredExtrusionEntity make_perimeter_extrusion(storage_handle *storage, const ExPolygon &area,
    const c_flow &flow, raw_extrusion_role role, uint16_t perimeter_idx, double line_offset,
    const plugin_run_context *run_ctx);
// Return the first region's internal flow, or an empty flow for a regionless island.
c_flow internal_perimeter_flow(const LayerIsland &island);
// Publish one shell using external flow at depth zero and internal flow below it.
// Return zero for an incomplete context; geometry failures reach the plugin runner.
int32_t generate_node(void *generator_context, perimeter_generation_context *context, perimeter_node *node,
                     expolygon_collection_handle *inner_areas_out, expolygon_collection_handle *inner_fill_areas_out);

const char *k_simple_perimeter_generator_id = "perimeter.generator.simple";
const char *k_no_dependencies[] = { nullptr };

const char *k_perimeters_key = "perimeters";

// used_config_keys is the plugin declaration: the host activates these settings
// while this implementation is selected and invalidates its caches when they
// change. The simple generator reads the wall count directly; the other
// generator settings are shared with the core or with the other
// implementations.
const raw_used_config_key k_used_config_keys[] = {
    { k_perimeters_key, RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE }
};

constexpr uint16_t k_perimeter_flags_loop = uint16_t(C_EXTRUSION_PERIMETER_FLAG_LOOP);
constexpr uint16_t k_perimeter_flags_hole = uint16_t(C_EXTRUSION_PERIMETER_FLAG_LOOP | C_EXTRUSION_PERIMETER_FLAG_HOLE);

StoredExPolygonCollection offset_area(storage_handle *storage,
                                      const ExPolygon &area,
                                      double delta,
                                      const plugin_run_context *run_ctx)
{
    ClipperOperand subject(storage, area);
    throw_if_cancelled(run_ctx);
    StoredExPolygonCollection out = clipper_offset(subject, delta).to_expolygon_collection();
    throw_if_cancelled(run_ctx);
    return out;
}

void append_perimeter_loop(StoredExtrusionEntity &dst,
                           const Polygon &polygon,
                           const c_flow &flow,
                           raw_extrusion_role role,
                           uint16_t perimeter_idx,
                           uint16_t perimeter_flags)
{
    if (!polygon.valid_polygon() || polygon.empty())
        return;

    std::vector<c_point> points = polygon.points();
    if (points.empty())
        return;
    points.push_back(points.front());

    StoredExtrusionEntity path(dst.storage(), points);
    EPropertyAttributes &attributes = path.get_or_add(EPropertyAttributes::key);
    attributes.extrusion_role(role)
        .mm3_per_mm(flow.mm3_per_mm)
        .width(float(unscaled(flow.width)))
        .height(float(unscaled(flow.height)));

    StoredExtrusionEntity loop(dst.storage());
    loop.get_or_add(EPropertyPerimeter::key).shell_count(perimeter_idx).perimeter_flags(perimeter_flags);
    loop.append_child_move(path.mutable_view());
    loop.set_flags(RAW_EXTRUSION_FLAG_REVERSIBLE);
    dst.append_child_move(loop.mutable_view());
}

StoredExtrusionEntity make_perimeter_extrusion(storage_handle *storage,
                                               const ExPolygon &area,
                                               const c_flow &flow,
                                               raw_extrusion_role role,
                                               uint16_t perimeter_idx,
                                               double line_offset,
                                               const plugin_run_context *run_ctx)
{
    StoredExtrusionEntity extrusion(storage);
    extrusion.disable_reverse().disable_sort();

    StoredExPolygonCollection loops = offset_area(storage, area, line_offset, run_ctx);
    throw_if_cancelled(run_ctx);
    for (ExPolygon loop : loops) {
        append_perimeter_loop(extrusion, loop.contour(), flow, role, perimeter_idx, k_perimeter_flags_loop);
        for (Polygon hole : loop.holes())
            append_perimeter_loop(extrusion, hole, flow, role, perimeter_idx, k_perimeter_flags_hole);
    }
    return extrusion;
}

struct SimpleGeneratorState
{
    c_flow flow = {};          // external perimeter flow, used by the first loop
    c_flow internal_flow = {}; // used by the following loops
    uint32_t perimeter_count = 1;
};

int32_t generate_node(void *generator_context,
                      perimeter_generation_context *context,
                      perimeter_node *node,
                      expolygon_collection_handle *inner_areas_out,
                      expolygon_collection_handle *inner_fill_areas_out)
{
    if (generator_context == nullptr || context == nullptr || context->run_ctx == nullptr ||
        context->run_ctx->plugin_storage == nullptr || node == nullptr ||
        inner_areas_out == nullptr || inner_fill_areas_out == nullptr)
        return 0;

    const SimpleGeneratorState &state = *reinterpret_cast<const SimpleGeneratorState *>(generator_context);
    storage_handle *storage = context->run_ctx->plugin_storage;
    PerimeterNodeView node_view(node);
    throw_if_cancelled(context->run_ctx);

    if (state.perimeter_count > 0)
        node_view.set_perimeter_needed(std::max(node_view.perimeter_needed(), state.perimeter_count));

    if (!node_view.needs_more_perimeters()) {
        StoredExPolygonCollection inner_areas(storage);
        inner_areas.push_back(node_view.area());
        expolygons_move(inner_areas_out, inner_areas.mutable_handle());

        StoredExPolygonCollection inner_fill_areas(storage);
        inner_fill_areas.push_back(node_view.fill_area());
        expolygons_move(inner_fill_areas_out, inner_fill_areas.mutable_handle());
        return 1;
    }

    const bool first_perimeter = node_view.perimeter_idx() == 0;
    // The first loop is the external perimeter; the following ones are internal perimeters.
    const c_flow &loop_flow = first_perimeter ? state.flow : state.internal_flow;
    const raw_extrusion_role loop_role = first_perimeter ? RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER :
                                                           RAW_EXTRUSION_ROLE_INTERNAL_PERIMETER;
    const double line_offset = first_perimeter ? -0.5 * double(state.flow.width) :
                                                 -0.5 * double(state.internal_flow.spacing);
    const uint16_t perimeter_idx = uint16_t(std::min<uint32_t>(node_view.perimeter_idx(), UINT16_MAX));
    StoredExtrusionEntity extrusion = make_perimeter_extrusion(
        storage, node_view.area(), loop_flow, loop_role, perimeter_idx, line_offset, context->run_ctx);
    throw_if_cancelled(context->run_ctx);
    extrusion_move_from(node->extrusions, extrusion.mutable_handle());

    const double inner_offset = first_perimeter ? -0.5 * double(state.flow.width + state.internal_flow.spacing) :
                                                  -double(state.internal_flow.spacing);
    StoredExPolygonCollection inner_areas =
        offset_area(storage, node_view.area(), inner_offset, context->run_ctx);
    throw_if_cancelled(context->run_ctx);
    expolygons_move(inner_areas_out, inner_areas.mutable_handle());

    // The fill/anchor area is slightly larger than the next perimeter area,
    // matching the old "perimeter spacing minus 25%" anchoring convention.
    const double fill_offset = inner_offset + 0.25 * double(state.internal_flow.spacing);
    StoredExPolygonCollection inner_fill_areas =
        offset_area(storage, node_view.area(), fill_offset, context->run_ctx);
    throw_if_cancelled(context->run_ctx);
    expolygons_move(inner_fill_areas_out, inner_fill_areas.mutable_handle());

    return 1;
}

c_flow external_perimeter_flow(const LayerIsland &island)
{
    return island.region_count() > 0 ? island.region(0).flow(RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER) : c_flow{};
}

c_flow internal_perimeter_flow(const LayerIsland &island)
{
    return island.region_count() > 0 ? island.region(0).flow(RAW_EXTRUSION_ROLE_INTERNAL_PERIMETER) : c_flow{};
}

uint32_t perimeter_count(const LayerIsland &island)
{
    if (island.region_count() == 0)
        return 0;
    const int32_t count = island.region(0).print_region().config().get("perimeters").get_int();
    return count <= 0 ? 0 : uint32_t(count);
}

} // namespace

SimplePerimeterGenerator &
SimplePerimeterGenerator::instance(orchestrator_handle *orch)
{
    static SimplePerimeterGenerator s_instance(orch);
    return s_instance;
}

const char *SimplePerimeterGenerator::id_impl() const noexcept
{
    return k_simple_perimeter_generator_id;
}

slicing_step_t SimplePerimeterGenerator::step_impl() const noexcept
{
    return STEP_PERIMETER;
}

const char *const *SimplePerimeterGenerator::dependencies_impl() const noexcept
{
    return k_no_dependencies;
}

int32_t SimplePerimeterGenerator::priority_impl() const noexcept
{
    return 0;
}

int32_t SimplePerimeterGenerator::used_config_keys(raw_used_config_key *keys) const noexcept
{
    if (keys != nullptr)
        for (uint32_t idx = 0; idx < sizeof(k_used_config_keys) / sizeof(k_used_config_keys[0]); ++idx)
            keys[idx] = k_used_config_keys[idx];
    return int32_t(sizeof(k_used_config_keys) / sizeof(k_used_config_keys[0]));
}

const char *SimplePerimeterGenerator::progress_message_format_impl() const noexcept
{
    return "Simple perimeter generator: %u / %u islands";
}

void SimplePerimeterGenerator::setup_run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_generate_perimeter *ctx = plugin_ctx_as_generate_perimeter(run_ctx);
    if (ctx != nullptr && ctx->island != nullptr)
        progress().add_max(1);
}

void SimplePerimeterGenerator::run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_generate_perimeter *ctx = plugin_ctx_as_generate_perimeter(run_ctx);
    if (ctx == nullptr || ctx->island == nullptr || ctx->run_region_group == nullptr)
        return;

    throw_if_cancelled(run_ctx);

    const LayerIsland island(ctx->island);
    if (island.region_count() == 0)
        return;

    std::vector<const layer_region_handle *> regions;
    regions.reserve(island.region_count());
    for (uint32_t region_idx = 0; region_idx < island.region_count(); ++region_idx)
        regions.push_back(island.region(region_idx).handle());

    SimpleGeneratorState state;
    state.flow = external_perimeter_flow(island);
    state.internal_flow = internal_perimeter_flow(island);
    state.perimeter_count = perimeter_count(island);
    throw_if_cancelled(run_ctx);
    ctx->run_region_group(ctx,
                          regions.empty() ? nullptr : regions.data(),
                          uint32_t(regions.size()),
                          island.slice().handle(),
                          &state,
                          &generate_node);
    throw_if_cancelled(run_ctx);

    progress().increment();
}

void register_simple_perimeter_generator_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, SimplePerimeterGenerator::instance(orch).c_instance());
}

}}} // namespace slic3r_api::Perimeter::SimplePerimeterGeneratorPlugin

#ifdef SIMPLE_PERIMETER_GENERATOR_PLUGIN_DLL
extern "C" void register_plugin(orchestrator_handle *orch)
{
    slic3r_api::Perimeter::SimplePerimeterGeneratorPlugin::register_simple_perimeter_generator_plugin(orch);
}
#endif // SIMPLE_PERIMETER_GENERATOR_PLUGIN_DLL
