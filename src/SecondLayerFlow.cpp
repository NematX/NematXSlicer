///|/ Copyright (c) NematXSlicer 2026 Durand Remi @supermerill
///|/
///|/ NematXSlicer is released under the terms of the AGPLv3 or higher

// Adjust deposited volume on the second printed layer.
// Extrusion flow is stored in mm3/mm. For example, an initial value of 0.04 mm3/mm
// with second_layer_flow_ratio set to 80% produces 0.032 mm3/mm. This plugin runs
// before DefaultSpeed so that speed calculation uses the adjusted extrusion flow.
//
// Terms used by the public API: PrintingLayerGroup contains a layer's extrusions
// grouped by tool; PrintingExtrusion is a printable extrusion and its entity tree;
// tree leaves contain path points; a region provides settings and an island provides
// layer geometry. A leaf inherits attributes from its parent unless it overrides them.
// Scaled coordinates are the integer geometry coordinates used by the slicer and
// share the coordinate frame used for instance offsets.
//
// Registration and execution: the host constructs the plugin, reads its metadata,
// calls inilialize_impl to define the option, then calls run_impl for each layer group.
//
// run_impl skips extrusions without a source (synthetic work, not region paths) and those
// whose source layer is not the second printed layer. The host guarantees that every other
// source is valid, with all its regions on the same layer. On the second layer, a run-local
// cache keeps the ratio factors of each source region island and print instance, like the
// host's first-layer flow plugin: only the regions of that region island are considered.
// Paths crossing regions with different ratios are split before each fragment receives its
// region's factor; skirt, brim and support use the object's default factor.
//
// The host may run several layer groups at the same time. Each callback only edits its
// own layer group, and its factors, cloned masks, settings, and views are local to it.
// plugin_storage is shared but locks its own allocations, so callbacks need no extra
// lock; never clear it here, other callbacks may still own geometry in it.
//
// Region masks are in object coordinates while printing paths include the instance offset,
// so each mask is cloned and its contours and holes are translated without rebuilding or
// unioning the polygons.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iterator>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_plugin.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/RegionSettingsViews.hpp"
#include "libslic3r/Api/plugin/cpp/StringHelpers.hpp"

namespace nematx_second_layer_flow {
namespace {

using namespace slic3r_api;

constexpr char k_plugin_id[] = "nematx_second_layer_flow";
constexpr char k_ratio_key[] = "second_layer_flow_ratio";
constexpr char k_option_tooltip[] =
    "Compensate on the second layer for intentional first-layer over-extrusion for adhesion: use the "
    "inverse ratio (125% on the first layer -> 80% on the second). Scales deposited flow in mm3/mm "
    "before speed calculation. Minimum: 1%.";

const char *const k_no_dependencies[] = { nullptr };
const char *const k_defined_config_keys[] = { k_ratio_key };
const raw_used_config_key k_used_config_keys[] = {
    { k_ratio_key, RAW_CO_PERCENT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT }
};

// Validate the normalized factor (for example, 80% becomes 0.8) before using it.
// Zero, negative, or non-finite values reject the export. Earlier leaves may already
// have been changed because processing the full export is not transactional.
double checked_factor(double normalized_factor)
{
    const double percent = normalized_factor * 100.0;
    if (!std::isfinite(normalized_factor) || normalized_factor <= 0.0)
        throw std::runtime_error(std::string(k_ratio_key) + " must be finite and greater than 0%; found " +
                                 to_string_nozero(percent, 15) + "%. The export was rejected; previously processed "
                                 "extrusion paths may already have been modified.");

    return normalized_factor;
}

// Compute a leaf's new flow and reject invalid inputs or results.
// This check precedes the current leaf's write but does not undo earlier writes.
double checked_product(double initial_flow, double factor)
{
    const double result = initial_flow * factor;
    if (!std::isfinite(initial_flow) || initial_flow <= 0.0 ||
        !std::isfinite(result) || result <= 0.0)
        throw std::runtime_error(std::string(k_ratio_key) + " produces an invalid mm3_per_mm value: " +
                                 to_string_nozero(initial_flow, 15) + " * " + to_string_nozero(factor, 15) + " = " +
                                 to_string_nozero(result, 15) + ". The export was rejected; previously processed "
                                 "extrusion paths may already have been modified.");
    return result;
}

// Select deposited paths that use this setting. Travel, wipe, retract, unretract,
// wipe-tower, and milling paths are excluded. Purge-tower flow is managed separately.
bool is_deposit_role(raw_extrusion_role role)
{
    if (role == RAW_EXTRUSION_ROLE_NONE ||
        RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_TRAVEL) ||
        RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_WIPE) ||
        RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_RETRACT) ||
        RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_UNRETRACT) ||
        RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_WIPE_TOWER) ||
        RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_MILL))
        return false;

    return RAW_EXTRUSION_ROLE_IS_PERIMETER(role) ||
           RAW_EXTRUSION_ROLE_IS_INFILL(role) ||
           RAW_EXTRUSION_ROLE_IS_SUPPORT(role) ||
           RAW_EXTRUSION_ROLE_IS_SKIRT(role) ||
           role == RAW_EXTRUSION_ROLE_GAP_FILL;
}

// Exclude empty leaves, leaves without effective attributes, paths already using the
// public E/mm convention (height == -2), and paths whose role is not a deposition.
bool is_flow_scaling_eligible_leaf(const MutableExtrusionEntity &entity,
                                   const std::optional<EPropertyAttributes> &attributes)
{
    if (entity.point_count() == 0 || !attributes)
        return false;

    // -2 is the public convention for paths whose E value is already expressed per mm.
    const auto &payload = static_cast<const c_extrusion_property_attributes &>(*attributes);
    if (payload.height == -2.f)
        return false;
    return is_deposit_role(attributes->extrusion_role());
}

// Resolve the effective attribute set: direct attributes override the parent's set;
// otherwise, the parent's set continues to child entities.
std::optional<EPropertyAttributes> effective_attributes(
    const MutableExtrusionEntity &entity,
    std::optional<EPropertyAttributes> inherited)
{
    if (const EPropertyAttributes *direct = entity.get(EPropertyAttributes::key))
        inherited = *direct;
    return inherited;
}

// Identify the second printed layer through its layer links: it has positive bottom Z
// and its valid lower layer starts at Z == 0.
bool is_second_layer(const Layer &layer)
{
    if (!layer.valid() || layer.bottom_z() <= 0)
        return false;
    const Layer lower = layer.lower_layer();
    return lower.valid() && lower.bottom_z() == 0;
}

// Cache key for one layer callback: the source region island and print instance.
// The catalog ID distinguishes instances with different offsets.
struct SourceKey
{
    const layer_region_island_handle *region_island = nullptr;
    uint32_t printing_catalog_id = UINT32_MAX;

    // Keep std::map ordering stable by comparing the handle, then the instance ID.
    bool operator<(const SourceKey &rhs) const
    {
        const std::less<const layer_region_island_handle *> less_handle;
        if (region_island != rhs.region_island)
            return less_handle(region_island, rhs.region_island);
        return printing_catalog_id < rhs.printing_catalog_id;
    }
};

// Flow factors of one extrusion source on the second layer. When its regions have
// different ratios, areas holds one mask per ratio (in printing-plan coordinates) and
// factors the matching factor; otherwise areas is empty and uniform_factor applies.
// outside_factor is used for skirt, brim and support, printed outside the masks.
struct SourceFactors
{
    double uniform_factor = 1.0;
    std::vector<StoredExPolygonCollection> areas;
    std::vector<double> factors;
    double outside_factor = 1.0;

    bool requires_split() const { return !areas.empty(); }
};

// Find the physical island that owns a region island. The plugin API has no parent
// link, so scan the islands of the source layer.
LayerIsland owning_layer_island(const LayerRegionIsland &region_island, const Layer &layer)
{
    for (uint32_t island_idx = 0; island_idx < layer.island_count(); ++island_idx) {
        const LayerIsland island = layer.island(island_idx);
        for (uint32_t idx = 0; idx < island.region_island_count(); ++idx)
            if (island.region_island(idx).handle() == region_island.handle())
                return island;
    }
    throw std::runtime_error("Printing extrusion source LayerRegionIsland has no owning LayerIsland.");
}

// Region masks are in object coordinates while printing paths include the instance
// offset: clone the mask and translate its contours and holes in place, without
// rebuilding or unioning the polygons.
StoredExPolygonCollection translated_area(storage_handle *storage, const ExPolygonCollection &source, c_point shift)
{
    StoredExPolygonCollection translated = source.clone(storage);
    if (shift.x == 0 && shift.y == 0)
        return translated;
    for (uint32_t expolygon_idx = 0; expolygon_idx < translated.size(); ++expolygon_idx) {
        ::expolygon_handle *expolygon = ::expolygons_at(translated.mutable_handle(), expolygon_idx);
        ::multipoint_translate(::polygon_as_multipoint(::expolygon_contour(expolygon)),
                               double(shift.x), double(shift.y));
        for (uint32_t hole_idx = 0; hole_idx < ::expolygon_hole_size(expolygon); ++hole_idx)
            ::multipoint_translate(::polygon_as_multipoint(::expolygon_hole_at(expolygon, hole_idx)),
                                   double(shift.x), double(shift.y));
    }
    return translated;
}

// Build the factors of one source from the regions of its region island only. A single
// region, or regions sharing one ratio, give a uniform factor. Otherwise RegionSettings
// groups the regions by ratio and clips them to the owning island.
// Skirt, brim and support use the default region configuration of their object: the
// print's defaults with the object's own region overrides (for the print-level skirt and
// brim, the auxiliary object, so the print's defaults). Not region_island.region(0): the
// source regions are ordered by address, so that choice could change between runs.
SourceFactors build_source_factors(storage_handle *storage,
                                   const Print &print,
                                   const PrintingPlan &printing_plan,
                                   const PrintingExtrusion &extrusion,
                                   const LayerRegionIsland &region_island,
                                   const Layer &source_layer)
{
    SourceFactors out;
    out.outside_factor = checked_factor(
        source_layer.object().default_region_config(print).percent_or_default(k_ratio_key, 1.0));
    const Config first_config = region_island.region(0).print_region().config();
    if (region_island.region_count() == 1) {
        out.uniform_factor = checked_factor(first_config.percent_or_default(k_ratio_key, 1.0));
        return out;
    }

    RegionSettings settings(storage, first_config, {{k_ratio_key}});
    for (uint32_t region_idx = 0; region_idx < region_island.region_count(); ++region_idx)
        settings.add_region(region_island.region(region_idx));
    settings.segregate(owning_layer_island(region_island, source_layer).slice());
    const RegionSettings::AreaMap &areas = settings.get_areas(k_ratio_key);
    if (areas.size() == 1) {
        out.uniform_factor = checked_factor(areas.begin()->first.get_float(k_ratio_key));
        return out;
    }

    const c_point shift = printing_plan.object_instance_shift(extrusion);
    for (const auto &[value, clip] : areas) {
        if (clip.is_accept_all())
            throw std::runtime_error("Regional flow partition has no concrete source area.");
        out.factors.push_back(checked_factor(value.get_float(k_ratio_key)));
        out.areas.push_back(translated_area(storage, clip.expolygons(), shift));
    }
    return out;
}
// Check that the splitter's fragments preserve the source path length within tolerance.
// This does not prove exact geometric coverage: a gap and overlap of equal length could cancel.
// The splitter drops pieces and snaps ends shorter than SCALED_EPSILON at each cut, so allow
// one SCALED_EPSILON per fragment plus a small relative term for accumulated rounding.
void validate_split_coverage(double source_length,
                             const std::vector<ExtrusionAreaFragment> &fragments)
{
    double fragment_length = 0.0;
    for (const ExtrusionAreaFragment &fragment : fragments)
        fragment_length += double(fragment.entity.local_length());

    const double tolerance = double(fragments.size()) * double(SCALED_EPSILON) +
                             std::abs(source_length) * 1e-7;
    if (!std::isfinite(source_length) || !std::isfinite(fragment_length) ||
        std::abs(source_length - fragment_length) > tolerance)
        throw std::runtime_error("Regional areas do not cover an entire printable path. "
                                 "The export was rejected to avoid dropping or duplicating geometry.");
}

// Materialize the effective attributes before changing flow. Preserve the full attribute
// set, including inherited properties, and update only mm3_per_mm.
void set_leaf_flow(MutableExtrusionEntity entity,
                   const EPropertyAttributes &effective,
                   double final_flow)
{
    EPropertyAttributes &direct = entity.get_or_add(EPropertyAttributes::key);
    direct = effective;
    direct.mm3_per_mm(final_flow);
}

// Traverse the entity tree while propagating effective attributes. A uniform source applies
// one factor; otherwise each eligible leaf is split once, except skirt, brim and support,
// which use the object's default factor. A 100% factor leaves attributes untouched.
// Validation is local, so a later error does not restore earlier leaves.
void apply_tree(MutableExtrusionEntity entity,
                const std::optional<EPropertyAttributes> &parent_attributes,
                const SourceFactors &factors,
                const std::vector<ExPolygonCollection> &views,
                const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    const std::optional<EPropertyAttributes> attributes =
        effective_attributes(entity, parent_attributes);

    if (entity.child_count() > 0) {
        for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
            throw_if_cancelled(run_ctx);
            apply_tree(entity.child_mutable(child_idx), attributes, factors, views, run_ctx);
        }
        return;
    }

    if (entity.point_count() > 0 && !attributes)
        throw std::runtime_error("Printable extrusion leaf has no effective EPropertyAttributes; "
                                 "the flow plugin cannot identify its movement role.");
    if (!is_flow_scaling_eligible_leaf(entity, attributes))
        return;

    const double initial_flow = attributes->mm3_per_mm();
    if (!factors.requires_split()) {
        const double final_flow = checked_product(initial_flow, factors.uniform_factor);
        if (factors.uniform_factor != 1.0)
            set_leaf_flow(entity, *attributes, final_flow);
        return;
    }

    // Skirt, brim and support lie outside the region masks: splitting them would leave
    // uncovered pieces. They take the object's default factor instead.
    const raw_extrusion_role role = attributes->extrusion_role();
    if (RAW_EXTRUSION_ROLE_IS_SKIRT(role) || RAW_EXTRUSION_ROLE_IS_SUPPORT(role)) {
        const double final_flow = checked_product(initial_flow, factors.outside_factor);
        if (factors.outside_factor != 1.0)
            set_leaf_flow(entity, *attributes, final_flow);
        return;
    }

    // The public splitter replaces this leaf and returns borrowed handles to its fragments.
    // Keep the views and factors alive through splitting and processing. Handle the new
    // children in this loop so this traversal does not visit them recursively a second time.
    const double source_length = double(entity.local_length());
    const std::vector<ExtrusionAreaFragment> fragments = entity.split_leaf_by_areas(views);
    validate_split_coverage(source_length, fragments);

    // Validate and process each fragment in order. This local validation is not transactional:
    // if a later fragment fails, earlier fragments remain processed.
    for (const ExtrusionAreaFragment &fragment : fragments) {
        throw_if_cancelled(run_ctx);
        if (fragment.area_index >= factors.factors.size())
            throw std::runtime_error("Regional flow splitter returned an invalid area index.");
        const double factor = factors.factors[fragment.area_index];
        const double final_flow = checked_product(initial_flow, factor);
        // At 100%, preserve the original direct or inherited attributes without writing them.
        if (factor != 1.0)
            set_leaf_flow(fragment.entity, *attributes, final_flow);
    }
}

class SecondLayerFlow final : public PluginBase
{
public:
    // Initialize the PluginBase adapter; the host calls the plugin callbacks afterward.
    explicit SecondLayerFlow(orchestrator_handle *orchestrator)
        : PluginBase(orchestrator) {}

private:
    // Return the stable ABI ID used by the host to identify this plugin.
    const char *id_impl() const noexcept override { return k_plugin_id; }
    // Return the label shown in the plugin selector.
    const char *name_impl() const noexcept override { return "Second layer flow"; }
    // Describe the affected layer and path types in the plugin selector.
    const char *description_impl() const noexcept override
    {
        return "Scales deposited volume on the second printed layer selected by source-layer links.";
    }
    // Run during layer extrusion editing, before DefaultSpeed.
    slicing_step_t step_impl() const noexcept override { return STEP_LAYER_EXTRUSION_EDIT; }
    // Declare that this plugin has no plugin dependencies.
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    // Set the execution order within this pipeline step.
    int32_t priority_impl() const noexcept override { return -10; }

    // Return the number of consumed settings and optionally copy their type and source.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        if (keys != nullptr)
            std::copy(std::begin(k_used_config_keys), std::end(k_used_config_keys), keys);
        return int32_t(std::size(k_used_config_keys));
    }

    // Return the number of settings defined by this plugin and optionally fill the output array.
    int32_t defined_config_keys(const char **keys) const noexcept override
    {
        if (keys != nullptr)
            std::copy(std::begin(k_defined_config_keys), std::end(k_defined_config_keys), keys);
        return int32_t(std::size(k_defined_config_keys));
    }

    // The host calls this initialization callback after registration, not from the constructor.
    // Define the regional FFF percentage setting with a 100% default; execution also requires
    // the value to be strictly positive.
    void inilialize_impl(storage_handle *) const override
    {
        raw_config_option_def def = raw_config_option_def_init();
        def.opt_key = k_ratio_key;
        def.type = RAW_CO_PERCENT;
        def.category = RAW_OPTION_CATEGORY_WIDTH;
        def.level = RAW_OPTION_LEVEL_EXPERT;
        def.container_type = RAW_CONTAINER_TYPE_REGION;
        def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
        def.printer_technology = RAW_PT_FFF;
        def.invalidates_step = STEP_LAYER_EXTRUSION_EDIT;
        def.label = "Second layer";
        def.full_label = "Second layer flow ratio";
        def.tooltip = k_option_tooltip;
        def.sidetext = "%";
        def.has_min = 1;
        def.min_value = 1.0;
        def.precision = 6;
        def.mode = RAW_CONFIG_OPTION_MODE_EXPERT;
        def.default_serialized_value = "100%";

        const option_def_error_code result = orchestrator_create_option_def(m_orchestrator, &def);
        if (result != OPTION_DEF_ERROR_OK)
            throw std::runtime_error(std::string("Cannot define ") + k_ratio_key +
                                     " (option definition error " + std::to_string(int(result)) + ").");
    }

    // Process one layer group supplied by the host. Every storage-backed object created
    // here is local to this call and destroyed before it returns.
    void run_impl(const plugin_run_context *run_ctx) const override
    {
        throw_if_cancelled(run_ctx);

        const run_ctx_layer_extrusion_edition *ctx =
            plugin_ctx_as_layer_extrusion_edition(run_ctx);
        if (ctx == nullptr || ctx->print == nullptr || ctx->plan == nullptr ||
            ctx->layer_group == nullptr || run_ctx->plugin_storage == nullptr)
            throw std::runtime_error("Second-layer flow received an invalid layer-group callback context.");

        const Print print(ctx->print);
        PrintingPlan printing_plan(ctx->plan);
        PrintingLayerGroup layer_group(ctx->layer_group);
        // Cache one set of factors per source region island and print instance, reused by
        // every extrusion of that source in this layer group.
        std::map<SourceKey, SourceFactors> sources;
        for (uint32_t tool_idx = 0; tool_idx < layer_group.tool_group_count(); ++tool_idx) {
            throw_if_cancelled(run_ctx);
            PrintingToolGroup tool_group = layer_group.tool_group(tool_idx);
            for (uint32_t extrusion_idx = 0;
                 extrusion_idx < tool_group.extrusion_count(); ++extrusion_idx) {
                throw_if_cancelled(run_ctx);
                PrintingExtrusion extrusion = tool_group.extrusion(extrusion_idx);
                // An extrusion without a source is synthetic work, not a region path.
                // Any other source is valid and all its regions share one layer.
                const LayerRegionIsland region_island = extrusion.region_island();
                if (!region_island.valid())
                    continue;
                const Layer source_layer = region_island.region(0).layer();
                if (!is_second_layer(source_layer))
                    continue;

                const SourceKey key{ region_island.handle(), extrusion.printing_catalog_id() };
                auto found = sources.find(key);
                if (found == sources.end())
                    found = sources.emplace(key, build_source_factors(run_ctx->plugin_storage, print,
                        printing_plan, extrusion, region_island, source_layer)).first;
                // A uniform 100% ratio needs no flow changes: skip the tree walk.
                const SourceFactors &factors = found->second;
                if (!factors.requires_split() && factors.uniform_factor == 1.0)
                    continue;

                // Build borrowed views once for this extrusion; the cache keeps their
                // geometry alive while the same view vector is shared through recursion.
                std::vector<ExPolygonCollection> views;
                views.reserve(factors.areas.size());
                for (const StoredExPolygonCollection &area : factors.areas)
                    views.push_back(area.readonly());
                apply_tree(extrusion.mutable_root(), std::nullopt, factors, views, run_ctx);
            }
        }
    }
};

} // namespace anonymous
} // namespace nematx_second_layer_flow

#ifdef NEMATX_SECOND_LAYER_FLOW_DLL
// Host ABI entry point. Register a static instance so it stays alive for every callback.
// Registration does not call inilialize_impl; the host reads metadata and initializes it afterward.
extern "C" SLIC3R_PLUGIN_API void register_plugin(orchestrator_handle *orchestrator)
{
    static nematx_second_layer_flow::SecondLayerFlow plugin(orchestrator);
    orchestrator_register_plugin(orchestrator, plugin.c_instance());
}
#endif

// Keep this header last so its ABI registration includes every public API header above.
#include "libslic3r/Api/plugin/c/slic3r_plugin_register_version.h"
