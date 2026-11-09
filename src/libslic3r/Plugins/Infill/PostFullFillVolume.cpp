///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "PostFullFillVolume.hpp"

#include <cassert>
#include <cmath>

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_post_infill.h"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionTreeVisitors.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/ParallelFor.hpp"
#include "libslic3r/Api/plugin/cpp/properties/ExtrusionProperties.hpp"

/*
Post-infill full-fill volume correction
=======================================

The legacy Fill::fill_surface_extrusion() adjusted the flow of an enforced full
fill surface (FillBase.cpp:437-461): it measured the volume the generated paths
would extrude, compared it with the volume the surface expects, and multiplied
mm3_per_mm and width by the clamped ratio. This plugin performs the same
correction after infill generation, so pattern plugins only produce geometry.

STEP_INFILL stamps the raw fill recipe on the root subtree of every surface
(c_extrusion_property_infill): the free area to fill, the flow height, spacing
and spacing ratio, the density, the flow multiplier and the infill flags. From
those values this pass rebuilds both sides of the legacy comparison:

- the target is area_to_fill_mm2 * height * flow_mult;
- the measured volume is the sum of length * mm3_per_mm over the leaves, and it
  is normalized by spacing / width_eff when the spacing ratio is not 1
  (FillBase.cpp:314-318), where
  width_eff = spacing - height * (1 - 0.25 * PI) * (1 - spacing_ratio).

The normalization keeps a reduced solid_infill_overlap an intentional void
instead of refilling it, exactly like the legacy code, and it is a no-op with
the default 100% overlap.

This pass walks the infill bucket of each LayerRegionIsland and, for every direct
child carrying a positive area to fill, measures the subtree volume and scales
the flow attributes of the whole subtree. Running it twice is a no-op while the
ratio stays inside the failsafe bounds; when the first run had to clamp, a second
run on the same tree converges toward the target instead. A normal re-slice
rebuilds the tree before this step, so the correction is applied once per
extrusion in practice.

The normal call flow is:

    register_post_full_fill_volume_plugin()
    `-- PostFullFillVolume::instance()
        `-- orchestrator_register_plugin()
            `-- PostFullFillVolume::run_impl()
                |-- walk object -> layers -> islands -> region islands
                |-- borrow the infill bucket of each region island
                |-- measure every stamped subtree and scale its flow attributes
                `-- leave unstamped subtrees unchanged
*/

namespace slic3r_api { namespace Infill { namespace PostFullFillVolumePlugin {
namespace {

const char *k_post_full_fill_volume_id = "infill.post_process.full_fill_volume";
const char *k_no_dependencies[] = { nullptr };

// Same failsafe bounds as the legacy correction (FillBase.cpp:412-413). They
// live here, not in the ABI: the recipe only describes the surface, the plugin
// owns the policy that decides how far the flow may move to reach the target.
const double k_min_flow_mult = 0.8;
const double k_max_flow_mult = 1.3;

// Ratios this close to 1 are floating point noise, not a real correction.
const double k_flow_mult_epsilon = 1e-6;

// Sum the material volume the leaves of one subtree would extrude.
class SubtreeVolumeVisitor : public ExtrusionTreeVisitor<>
{
public:
    double volume_mm3 = 0.;

    void visit_leaf(MutableExtrusionEntity entity) override
    {
        const EPropertyAttributes *attributes = current_property(EPropertyAttributes::key);
        if (attributes == nullptr || !entity.has_polyline())
            return;

        const c_extrusion_property_attributes &raw = *attributes;
        volume_mm3 += unscaled(entity.local_length()) * raw.mm3_per_mm;
    }
};

// Scale the flow of every leaf that carries its own attributes, like the legacy
// ExtrusionModifyFlow visitor did (ExtrusionEntityVisitors.cpp:653-660). The
// height stays untouched: the correction changes how much material is laid
// down, not the layer thickness.
//
// Every attribute carried by the subtree is scaled, not only the leaves: a leaf
// that inherits its flow from an ancestor then sees the scaled ancestor value,
// a leaf with its own attributes is scaled directly, and a subtree root can
// carry the flow for all of its children. Nothing is materialized on the
// leaves, so a later edit of an ancestor still applies to every descendant.
// enter_node() runs for every entity of the traversal, leaves included.
class ScaleFlowAttributesVisitor : public ExtrusionTreeVisitor<>
{
public:
    explicit ScaleFlowAttributesVisitor(double mult) : m_mult(mult) {}

    void enter_node(MutableExtrusionEntity entity) override
    {
        EPropertyAttributes *attributes = entity.get_mutable(EPropertyAttributes::key);
        if (attributes == nullptr)
            return;

        c_extrusion_property_attributes &raw = *attributes;
        raw.mm3_per_mm *= m_mult;
        raw.width = float(raw.width * m_mult);
    }

private:
    double m_mult = 1.;
};

void correct_stamped_subtrees(MutableExtrusionEntity root, const plugin_run_context *run_ctx)
{
    for (uint32_t child_idx = 0; child_idx < root.child_count(); ++child_idx) {
        if ((child_idx & 63u) == 0)
            throw_if_cancelled(run_ctx);

        // The stamp lives on the direct child of the bucket because the host
        // keeps property-carrying collections as semantic subtree roots. Only
        // that outermost target is consumed: a nested subtree carrying its own
        // stamp is ignored on purpose. Gap fill is published inside the source
        // surface subtree without its own stamp, so this pass measures fill and
        // gap fill together like the legacy accounting did (FillBase.cpp:312).
        MutableExtrusionEntity child = root.child_mutable(child_idx);
        EPropertyInfill *infill = child.get_mutable(EPropertyInfill::key);
        if (infill == nullptr || infill->area_to_fill_mm2() <= 0.)
            continue;

        SubtreeVolumeVisitor measure;
        measure.traverse(child);
        if (measure.volume_mm3 <= 0.)
            continue;

        // Rebuild the legacy comparison from the stamped recipe. The target is
        // the volume of the free area, and the measured volume is normalized
        // from the extruded width to the real spacing when the recipe reduces
        // the line overlap on purpose (FillBase.cpp:314-318).
        const double height_mm = unscaled(infill->height());
        const double spacing_mm = unscaled(infill->spacing());
        double measured_mm3 = measure.volume_mm3;
        if (infill->spacing_ratio() != 1.f && spacing_mm > 0.) {
            const double width_eff_mm = spacing_mm - height_mm * (1. - 0.25 * PI) *
                                                          (1. - double(infill->spacing_ratio()));
            if (width_eff_mm > 0.)
                measured_mm3 *= spacing_mm / width_eff_mm;
        }

        const double target_mm3 = infill->area_to_fill_mm2() * height_mm * double(infill->flow_mult());
        double mult = target_mm3 / measured_mm3;
        if (mult > k_max_flow_mult)
            mult = k_max_flow_mult;
        if (mult < k_min_flow_mult)
            mult = k_min_flow_mult;
        if (std::abs(mult - 1.) <= k_flow_mult_epsilon)
            continue;

        ScaleFlowAttributesVisitor scale(mult);
        scale.traverse(child);
    }
}

void correct_region_island(const run_ctx_post_infill_generation &ctx,
                           const LayerRegionIsland &region_island,
                           const plugin_run_context *run_ctx)
{
    // Every infill role resolves to the same bucket, so one lookup covers the
    // sparse, solid, top-solid and bridge subtrees of this region island.
    if (!region_island.has_extrusion(RAW_EXTRUSION_ROLE_INTERNAL_INFILL))
        return;

    extrusion_entity_handle *root_handle =
        ctx.get_region_island_mutable_extrusion(region_island.handle(), RAW_EXTRUSION_ROLE_INTERNAL_INFILL);
    if (root_handle == nullptr)
        return;

    correct_stamped_subtrees(MutableExtrusionEntity(root_handle), run_ctx);
}

// Worker of the per-layer parallel loop. Layers own disjoint extrusion trees,
// so the correction can run once per layer; scratch_storage stays unused
// because this pass only mutates existing flow attributes.
void correct_layer(uint32_t layer_idx,
                   storage_handle * /*scratch_storage*/,
                   const run_ctx_post_infill_generation *ctx,
                   const plugin_run_context *run_ctx)
{
    const Layer layer = Object(ctx->object).layer(layer_idx);
    for (uint32_t island_idx = 0; island_idx < layer.island_count(); ++island_idx) {
        if ((island_idx & 63u) == 0)
            throw_if_cancelled(run_ctx);
        const LayerIsland island = layer.island(island_idx);
        for (uint32_t region_island_idx = 0; region_island_idx < island.region_island_count(); ++region_island_idx) {
            if ((region_island_idx & 63u) == 0)
                throw_if_cancelled(run_ctx);
            correct_region_island(*ctx, island.region_island(region_island_idx), run_ctx);
        }
    }
}

} // namespace

PostFullFillVolume &
PostFullFillVolume::instance(orchestrator_handle *orch)
{
    static PostFullFillVolume s_instance(orch);
    return s_instance;
}

const char *PostFullFillVolume::id_impl() const noexcept
{
    return k_post_full_fill_volume_id;
}

const char *PostFullFillVolume::name_impl() const noexcept
{
    return "Post-infill full fill volume";
}

const char *PostFullFillVolume::description_impl() const noexcept
{
    return "Corrects the flow of enforced full fill surfaces from the raw fill recipe stamped by the infill generator.";
}

slicing_step_t PostFullFillVolume::step_impl() const noexcept
{
    return STEP_POST_INFILL;
}

const char *const *PostFullFillVolume::dependencies_impl() const noexcept
{
    return k_no_dependencies;
}

int32_t PostFullFillVolume::priority_impl() const noexcept
{
    // Runs after gap fill (priority 0) so the measurement includes gap volume
    // in the stamped subtree, exactly like the legacy accounting
    // (FillBase.cpp:312), and after
    // dense_infill.post_infill_order (priority 10) so the pass order never
    // depends on plugin registration order.
    return 11;
}

const char *PostFullFillVolume::progress_message_format_impl() const noexcept
{
    return "Correcting full fill volume: %u / %u layers";
}

void PostFullFillVolume::setup_run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_post_infill_generation *ctx = plugin_ctx_as_post_infill_generation(run_ctx);
    if (ctx == nullptr || ctx->object == nullptr)
        return;
    progress().add_max(Object(ctx->object).layer_count());
}

void PostFullFillVolume::run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_post_infill_generation *ctx = plugin_ctx_as_post_infill_generation(run_ctx);
    assert(ctx != nullptr);
    if (ctx == nullptr || ctx->object == nullptr || ctx->get_region_island_mutable_extrusion == nullptr)
        return;

    const uint32_t layer_count = Object(ctx->object).layer_count();
    parallel_for_storage_with_progress(0,
                                       layer_count,
                                       run_ctx,
                                       &progress(),
                                       correct_layer,
                                       ctx,
                                       run_ctx);
}

void register_post_full_fill_volume_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, PostFullFillVolume::instance(orch).c_instance());
}

}}} // namespace slic3r_api::Infill::PostFullFillVolumePlugin
