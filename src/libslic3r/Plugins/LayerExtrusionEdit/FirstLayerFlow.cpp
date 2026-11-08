///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
First-layer flow
================

Extrusion flow is stored in mm3/mm on each extrusion leaf. On the first layer
of an object (the layer whose bottom is at Z = 0), this plugin multiplies that
flow by two ratios, like the legacy G-code writer did:
- first_layer_flow_ratio, a region setting: a path that crosses regions with
  different ratios is split at their border, and each piece gets its region's
  ratio;
- filament_first_layer_flow_ratio, read for the extruder of the tool group.

Only deposited material is scaled: travel, wipe, retraction, wipe tower and
milling paths, and paths already expressed in E per mm, are left untouched.
Skirt, brim and support paths lie outside the region masks, so they are not
split: they use the default region configuration of their object, the print's
defaults with the object's own region overrides. For the print-level skirt and
brim, that object is the print's auxiliary object, so they use the print's defaults.

The plugin runs before the default speed plugin, so autospeed and volumetric
limits use the final flow. Each run only edits its own layer group; every
storage-backed object it creates is local to the run. The shared
plugin_storage locks its own allocations and must never be cleared here.

    run_impl()
    |-- build_regional_settings_partition(): borrow configurations and copy masks
    |-- project_regional_process_values(): combine regions with equal ratios
    `-- apply_tree(): inherit attributes and edit only deposited flow
        |-- deposit_value(): choose one value without duplicating a deposit
        `-- split_leaf_by_areas(): preserve geometry while assigning regional ratios
*/

#include "FirstLayerFlow.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "ExtrusionProcessParameterHelpers.hpp"
#include "RegionalProcessParameterHelpers.hpp"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/StringHelpers.hpp"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace FirstLayerFlowPlugin {
namespace {

using namespace ProcessParameterHelpers;
using namespace RegionalProcessParameterHelpers;

constexpr char k_plugin_id[] = "layer_extrusion_edit.first_layer_flow";
constexpr char k_ratio_key[] = "first_layer_flow_ratio";
constexpr char k_filament_ratio_key[] = "filament_first_layer_flow_ratio";

const char *const k_no_dependencies[] = { nullptr };
const char *const k_defined_config_keys[] = { k_ratio_key };
const raw_used_config_key k_used_config_keys[] = {
    { k_ratio_key, RAW_CO_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { k_filament_ratio_key, RAW_CO_VECTOR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE }
};

struct TreeRatios;

// Reject a ratio that would remove all material or is not a number. The option
// accepts 0% for compatibility with old profiles, but a zero flow cannot print.
double checked_ratio(const char *key, double ratio);
// Convert the borrowed regional percent setting to a positive factor.
float region_ratio(const Config &region_config);
// Identify deposited roles whose flow is not managed by a separate service.
bool is_deposit_role(raw_extrusion_role role);
// Require the fragments to cover the source within the splitter's rounding budget.
void check_split_coverage(double source_length, const std::vector<ExtrusionAreaFragment> &fragments);
// Copy inherited attributes to a leaf and change only its mm3/mm flow.
void set_leaf_flow(MutableExtrusionEntity leaf, const EPropertyAttributes &effective, double flow);
// Scale borrowed tree leaves in place. Exterior roles use object defaults;
// deposits select one regional value, while ordinary paths may split. Geometry,
// ratio and cancellation errors propagate without retaining any borrowed view.
void apply_tree(MutableExtrusionEntity entity, std::optional<EPropertyAttributes> attributes,
                const TreeRatios &ratios, const plugin_run_context *run_ctx);

double checked_ratio(const char *key, double ratio)
{
    if (!std::isfinite(ratio) || ratio <= 0.)
        throw std::runtime_error(std::string(key) + " must be greater than 0% on the first layer; found " +
                                 to_string_nozero(ratio * 100., 15) + "%.");
    return ratio;
}

float region_ratio(const Config &region_config)
{
    return float(checked_ratio(k_ratio_key, region_config.percent_or_default(k_ratio_key, 1.)));
}

bool is_deposit_role(raw_extrusion_role role)
{
    if (role == RAW_EXTRUSION_ROLE_NONE ||
        RAW_EXTRUSION_ROLE_IS_TRAVEL(role) ||
        RAW_EXTRUSION_ROLE_IS_WIPE(role) ||
        RAW_EXTRUSION_ROLE_IS_RETRACT(role) ||
        RAW_EXTRUSION_ROLE_IS_UNRETRACT(role) ||
        RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_WIPE_TOWER) ||
        RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_MILL))
        return false;
    return RAW_EXTRUSION_ROLE_IS_PERIMETER(role) ||
           RAW_EXTRUSION_ROLE_IS_INFILL(role) ||
           RAW_EXTRUSION_ROLE_IS_SUPPORT(role) ||
           RAW_EXTRUSION_ROLE_IS_SKIRT(role) ||
           role == RAW_EXTRUSION_ROLE_GAP_FILL;
}

// Flow ratios of one extrusion source on the first layer: the regional
// partition of first_layer_flow_ratio, and the ratio of the paths printed
// outside the region masks (skirt, brim, support), from the object's default region.
struct SourceRatios
{
    RegionalProcessPartition partition;
    double outside_ratio = 1.;
};

// Everything a leaf needs to compute its new flow.
struct TreeRatios
{
    const SourceRatios &source;
    std::vector<ExPolygonCollection> area_views;
    double filament_ratio = 1.;
};

void check_split_coverage(double source_length, const std::vector<ExtrusionAreaFragment> &fragments)
{
    double fragment_length = 0.;
    for (const ExtrusionAreaFragment &fragment : fragments)
        fragment_length += double(fragment.entity.local_length());
    const double tolerance = double(fragments.size()) * double(SCALED_EPSILON) + source_length * 1e-7;
    if (std::abs(source_length - fragment_length) > tolerance)
        throw std::runtime_error("First-layer flow: the region areas do not cover a printed path.");
}

void set_leaf_flow(MutableExtrusionEntity leaf, const EPropertyAttributes &effective, double flow)
{
    EPropertyAttributes &direct = leaf.get_or_add(EPropertyAttributes::key);
    direct = effective;
    direct.mm3_per_mm(flow);
}

void apply_tree(MutableExtrusionEntity entity,
                std::optional<EPropertyAttributes> attributes,
                const TreeRatios &ratios,
                const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    if (const EPropertyAttributes *direct = entity.get(EPropertyAttributes::key))
        attributes = *direct;

    if (entity.child_count() > 0) {
        for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
            apply_tree(entity.child_mutable(child_idx), attributes, ratios, run_ctx);
        return;
    }

    if (entity.point_count() == 0)
        return;
    if (!attributes)
        throw std::runtime_error("First-layer flow: a printed path has no extrusion attributes.");
    const raw_extrusion_role role = attributes->extrusion_role();
    // height == -2 marks a path whose E value is already given per mm.
    if (static_cast<const c_extrusion_property_attributes &>(*attributes).height == -2.f ||
        !is_deposit_role(role))
        return;

    const double initial_flow = attributes->mm3_per_mm();
    if (!std::isfinite(initial_flow) || initial_flow <= 0.)
        throw std::runtime_error("First-layer flow: a printed path has no positive flow.");

    const RegionalProcessPartition &partition = ratios.source.partition;
    const bool outside_regions = RAW_EXTRUSION_ROLE_IS_SKIRT(role) || RAW_EXTRUSION_ROLE_IS_SUPPORT(role);
    if (!partition.requires_split() || outside_regions) {
        const double region = outside_regions ? ratios.source.outside_ratio :
                                                double(partition.uniform_value);
        const double factor = region * ratios.filament_ratio;
        if (factor != 1.)
            set_leaf_flow(entity, *attributes, initial_flow * factor);
        return;
    }

    if (entity.is_deposit()) {
        const double factor = double(deposit_value(entity.readonly(), partition)) * ratios.filament_ratio;
        if (factor != 1.)
            set_leaf_flow(entity, *attributes, initial_flow * factor);
        return;
    }

    // The splitter replaces this leaf by its fragments; they are handled here,
    // not by the recursion, so each piece is scaled exactly once.
    const double source_length = double(entity.local_length());
    const std::vector<ExtrusionAreaFragment> fragments = entity.split_leaf_by_areas(ratios.area_views);
    check_split_coverage(source_length, fragments);
    for (const ExtrusionAreaFragment &fragment : fragments) {
        if (fragment.area_index >= partition.values.size())
            throw std::runtime_error("First-layer flow: the splitter returned an invalid area index.");
        const double factor = double(partition.values[fragment.area_index]) * ratios.filament_ratio;
        if (factor != 1.)
            set_leaf_flow(fragment.entity, *attributes, initial_flow * factor);
    }
}

class FirstLayerFlow final : public PluginBase
{
public:
    // Borrow the provider instance bound to the application's orchestrator.
    static FirstLayerFlow &instance(orchestrator_handle *orch)
    {
        static FirstLayerFlow singleton(orch);
        return singleton;
    }

private:
    // Borrow the orchestrator; geometry and partitions belong to each run.
    explicit FirstLayerFlow(orchestrator_handle *orch) : PluginBase(orch) {}

    // Return the stable provider identifier.
    const char *id_impl() const noexcept override { return k_plugin_id; }
    // Return borrowed display text.
    const char *name_impl() const noexcept override { return "First layer flow"; }
    // Describe the two independent flow factors to the settings catalogue.
    const char *description_impl() const noexcept override
    {
        return "Scales deposited flow on the first layer by the region and filament first-layer flow ratios.";
    }
    // Edit ordered layer groups before final process limits are resolved.
    slicing_step_t step_impl() const noexcept override { return STEP_LAYER_EXTRUSION_EDIT; }
    // Return the empty dependency list; priority establishes editor order.
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    // Before the default speed plugin (priority 0), which reads the final flow.
    int32_t priority_impl() const noexcept override { return -10; }

    // Copy declared inputs, or return their count when keys is null.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        if (keys != nullptr)
            std::copy(std::begin(k_used_config_keys), std::end(k_used_config_keys), keys);
        return int32_t(std::size(k_used_config_keys));
    }

    // Copy the owned regional key, or return its count when keys is null.
    int32_t defined_config_keys(const char **keys) const noexcept override
    {
        if (keys != nullptr)
            std::copy(std::begin(k_defined_config_keys), std::end(k_defined_config_keys), keys);
        return int32_t(std::size(k_defined_config_keys));
    }

    // Define the regional ratio once; registration failures remain visible.
    void inilialize_impl(storage_handle *) const override
    {
        raw_config_option_def def = raw_config_option_def_init();
        def.opt_key = k_ratio_key;
        def.type = RAW_CO_PERCENT;
        def.container_type = RAW_CONTAINER_TYPE_REGION;
        def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
        def.printer_technology = RAW_PT_FFF;
        def.invalidates_step = STEP_LAYER_EXTRUSION_EDIT;
        def.category = RAW_OPTION_CATEGORY_WIDTH;
        def.label = "First layer";
        def.full_label = "First layer flow ratio";
        def.tooltip = "You can increase this to over-extrude on the first layer if there is not enough plastic because your bed isn't levelled.\nNote: DON'T USE THIS if your only problem is bed leveling, LEVEL YOUR BED! Use this setting only as last resort after all calibrations failed.";
        def.sidetext = "%";
        def.has_min = 1;
        def.min_value = 0;
        def.precision = 6;
        def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
        def.default_serialized_value = "100%";
        const option_def_error_code result = orchestrator_create_option_def(m_orchestrator, &def);
        if (result != OPTION_DEF_ERROR_OK)
            throw std::runtime_error(std::string("Cannot define ") + k_ratio_key +
                                     " (option definition error " + std::to_string(int(result)) + ").");
    }

    // Resolve first-layer sources into run-local partitions and mutate their
    // plan flow. Both regional and exterior factors must be neutral to skip a tree.
    void run_impl(const plugin_run_context *run_ctx) const override
    {
        throw_if_cancelled(run_ctx);
        const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
        if (ctx == nullptr || ctx->print == nullptr || ctx->plan == nullptr ||
            ctx->layer_group == nullptr || run_ctx->plugin_storage == nullptr)
            throw std::runtime_error("First-layer flow received an invalid layer-group callback context.");

        storage_handle *storage = run_ctx->plugin_storage;
        const Print print(ctx->print);
        PrintingPlan plan(ctx->plan);
        PrintingLayerGroup layer_group(ctx->layer_group);
        // One entry per source region island and object instance, reused by
        // every extrusion of that source in this layer group.
        std::map<RegionalSourceKey, SourceRatios> sources;

        for (uint32_t tool_idx = 0; tool_idx < layer_group.tool_group_count(); ++tool_idx) {
            throw_if_cancelled(run_ctx);
            PrintingToolGroup tool_group = layer_group.tool_group(tool_idx);
            const extruder_index_t extruder_id = tool_group.extruder_id();
            const double filament_ratio = extruder_id == EXTRUDER_ID_UNKNOWN ? 1. :
                checked_ratio(k_filament_ratio_key,
                              print.config().vector_percent_or_default(k_filament_ratio_key, uint32_t(extruder_id), 1.));

            for (uint32_t extrusion_idx = 0; extrusion_idx < tool_group.extrusion_count(); ++extrusion_idx) {
                throw_if_cancelled(run_ctx);
                PrintingExtrusion extrusion = tool_group.extrusion(extrusion_idx);
                // Region-less service paths, and paths without a source layer,
                // cannot be placed on a layer; the speed plugin reports the latter.
                const LayerRegionIsland region_island = extrusion.region_island();
                if (!region_island.valid() || region_island.region_count() == 0)
                    continue;
                const Layer layer = region_island.region(0).layer();
                if (!layer.valid() || layer.bottom_z() > 0)
                    continue;

                const RegionalSourceKey key{ region_island.handle(), printing_plan_object_instance_idx(plan, extrusion) };
                auto found = sources.find(key);
                if (found == sources.end()) {
                    SourceRatios ratios;
                    const RegionalSettingsPartition settings = build_regional_settings_partition(
                        storage, region_island, { k_ratio_key }, plan.object_instance_shift(extrusion));
                    ratios.partition = project_regional_process_values(storage, settings, region_ratio);
                    // Not region_island.region(0): the source regions are ordered by
                    // address, so that choice could change from one slicing to the next.
                    ratios.outside_ratio = region_ratio(layer.object().default_region_config(print));
                    found = sources.emplace(key, std::move(ratios)).first;
                }

                const SourceRatios &source = found->second;
                if (!source.partition.requires_split() &&
                    double(source.partition.uniform_value) * filament_ratio == 1. &&
                    source.outside_ratio * filament_ratio == 1.)
                    continue;
                TreeRatios ratios{ source, {}, filament_ratio };
                for (const StoredExPolygonCollection &area : source.partition.areas)
                    ratios.area_views.push_back(area.readonly());
                apply_tree(extrusion.mutable_root(), std::nullopt, ratios, run_ctx);
            }
        }
    }
};

} // namespace

void register_first_layer_flow_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, FirstLayerFlow::instance(orch).c_instance());
}

}}} // namespace slic3r_api::LayerExtrusionEdit::FirstLayerFlowPlugin
