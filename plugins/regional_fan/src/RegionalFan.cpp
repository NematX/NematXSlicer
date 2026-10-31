///|/ Copyright (c) 2026 NematX
///|/
///|/ Released under the GNU Affero General Public License, version 3 or later.

/*
Regional fan speed overrides
============================

Four region settings, disabled by default, force the fan speed of the paths
printed in a region (usually through a modifier):
- print_fan_speed: every deposited path of the region;
- print_perimeters_fan_speed: internal and external perimeters (not overhangs
  or thin walls);
- print_solid_infill_fan_speed: solid and top solid infill, ironing;
- print_bridge_fan_speed: bridge and internal bridge infill.
A role-specific setting wins over print_fan_speed. An enabled 0% stops the fan;
a disabled setting leaves the path to the standard fan policies.

Each value becomes an Override fan request on the deposited leaf. The plugin
runs at LAYER_EXTRUSION_EDIT_PRIORITY_FAN_REGIONAL (18), before the host's
standard fan (20), which only fills the leaves left without a request. The
later policies (layer-time cooling, first layers, dynamic overhangs) never
change an Override. An existing Override, or a request in the mode of another
plugin, is kept; an Automatic request is replaced.

Paths crossing regions with different requests are split at the region
boundaries, as in the second-layer flow plugin: a run-local cache keeps the
requests of each source region island and print instance, with one mask per
value translated by the instance offset. Skirt, brim and support use the default
region configuration of their object, without splitting. Extrusions without a
source region island (synthetic services) are left untouched.

The leaves are those of the host fan policies: empty leaves (events), travel,
wipe tower and milling paths are excluded, as are paths whose E is replaced by a
raw EXTRUSION command, such as the zero-flow connectors of the around-holes
infill: they keep the fan state of their neighbours.

A split copies the direct properties of the leaf into every fragment. This is
right for inherited values (attributes, speeds, metadata of other plugins) but
would repeat a one-shot event: splitting a leaf that carries a custom G-code, a
special command, a heater or an extrusion-axis event rejects the export.
*/

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iterator>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "RegionalFanTree.hpp"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_plugin.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/RegionSettingsViews.hpp"

namespace nematx_regional_fan {
namespace {

using namespace slic3r_api;

constexpr const char *k_plugin_id = "nematx_regional_fan";

// Indexed by Setting.
const char *const k_keys[SettingCount] = {
    "print_fan_speed", "print_perimeters_fan_speed", "print_solid_infill_fan_speed", "print_bridge_fan_speed"
};
// Legacy labels and tooltips.
const char *const k_labels[SettingCount] = {
    "Fan speed override", "Fan speed override for all perimeters",
    "Fan speed override for top and solid infill", "Fan speed override for bridges"
};
const char *const k_tooltips[SettingCount] = {
    "Override the fan speed (to be used in modifiers)."
    "\nIf disabled, uses the fan speed as specified for the current filament."
    "\nSet to 0 to disable the fan.",
    "Override the fan speed (to be used in modifiers) for internal and external perimeters."
    "\nIt override the print_fan_speed general fan override."
    "\nIf disabled, uses the fan speed as specified for the current filament."
    "\nSet to 0 to disable the fan.",
    "Override the fan speed (to be used in modifiers) for solid and top infill."
    "\nIt override the print_fan_speed general fan override."
    "\nIf disabled, uses the fan speed as specified for the current filament."
    "\nSet to 0 to disable the fan.",
    "Override the fan speed (to be used in modifiers) for bridge infill."
    "\nIt override the print_fan_speed general fan override."
    "\nIf disabled, uses the fan speed as specified for the current filament."
    "\nSet to 0 to disable the fan."
};

const char *const k_no_dependencies[] = { nullptr };
const raw_used_config_key k_used_config_keys[] = {
    { k_keys[General], RAW_CO_PERCENT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT },
    { k_keys[Perimeters], RAW_CO_PERCENT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT },
    { k_keys[SolidInfill], RAW_CO_PERCENT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT },
    { k_keys[Bridge], RAW_CO_PERCENT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT }
};

// Build the requests from a reader returning the normalized value of an enabled
// setting (percent settings are read as ratios: 70% -> 0.7), or a negative value.
Requests read_requests(const std::function<double(const char *)> &ratio)
{
    Requests out;
    for (int setting = 0; setting < SettingCount; ++setting) {
        const double value = ratio(k_keys[setting]);
        if (value < 0.)
            continue;
        const double percent = value * 100.;
        if (!std::isfinite(percent) || percent > 100.)
            throw std::runtime_error(std::string(k_keys[setting]) + " must be between 0% and 100%.");
        out.percent[setting] = float(percent);
    }
    return out;
}

Requests read_requests(const Config &config)
{
    return read_requests([&config](const char *key) { return config.percent_or_default(key, -1.); });
}

Requests read_requests(const RegionSettingsValue &value)
{
    return read_requests([&value](const char *key) { return value.is_enabled(key) ? value.get_float(key) : -1.; });
}

// Cache key for one layer callback: the source region island and print instance.
// The catalog ID distinguishes instances with different offsets.
struct SourceKey
{
    const layer_region_island_handle *region_island = nullptr;
    uint32_t printing_catalog_id = UINT32_MAX;

    bool operator<(const SourceKey &rhs) const
    {
        const std::less<const layer_region_island_handle *> less_handle;
        if (region_island != rhs.region_island)
            return less_handle(region_island, rhs.region_island);
        return printing_catalog_id < rhs.printing_catalog_id;
    }
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
// offset: clone the mask and translate its contours and holes in place.
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

// Read the requests of one source from the regions of its region island only.
// Regions with the same requests need no partition. Otherwise RegionSettings
// groups the regions by setting values and clips them to the owning island.
// Skirt, brim and support use the default region configuration of their object
// (for the print-level skirt and brim, the auxiliary object: the print's values).
SourceRequests build_source_requests(storage_handle *storage, const Print &print, const PrintingPlan &plan,
                                     const PrintingExtrusion &extrusion, const LayerRegionIsland &region_island)
{
    const Layer layer = region_island.region(0).layer();
    SourceRequests out;
    out.outside = read_requests(layer.object().default_region_config(print));
    const Config first_config = region_island.region(0).print_region().config();
    out.uniform = read_requests(first_config);
    bool uniform = true;
    for (uint32_t region_idx = 1; uniform && region_idx < region_island.region_count(); ++region_idx)
        uniform = read_requests(region_island.region(region_idx).print_region().config()) == out.uniform;
    if (uniform)
        return out;

    RegionSettings settings(storage, first_config,
        {{k_keys[General], k_keys[Perimeters], k_keys[SolidInfill], k_keys[Bridge]}});
    for (uint32_t region_idx = 0; region_idx < region_island.region_count(); ++region_idx)
        settings.add_region(region_island.region(region_idx));
    settings.segregate(owning_layer_island(region_island, layer).slice());
    const RegionSettings::AreaMap &areas = settings.get_areas(k_keys[General]);
    if (areas.size() == 1) {
        out.uniform = read_requests(areas.begin()->first);
        return out;
    }
    const c_point shift = plan.object_instance_shift(extrusion);
    for (const auto &[value, clip] : areas) {
        if (clip.is_accept_all())
            throw std::runtime_error("Regional fan partition has no concrete source area.");
        out.requests.push_back(read_requests(value));
        out.areas.push_back(translated_area(storage, clip.expolygons(), shift));
    }
    return out;
}

class RegionalFan final : public PluginBase
{
public:
    explicit RegionalFan(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

private:
    const char *id_impl() const noexcept override { return k_plugin_id; }
    const char *name_impl() const noexcept override { return "Regional fan speed overrides"; }
    const char *description_impl() const noexcept override
    {
        return "Forces the fan speed of the paths of a region, for all of them or per role.";
    }
    slicing_step_t step_impl() const noexcept override { return STEP_LAYER_EXTRUSION_EDIT; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    int32_t priority_impl() const noexcept override { return LAYER_EXTRUSION_EDIT_PRIORITY_FAN_REGIONAL; }

    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        if (keys != nullptr)
            std::copy(std::begin(k_used_config_keys), std::end(k_used_config_keys), keys);
        return int32_t(std::size(k_used_config_keys));
    }
    int32_t defined_config_keys(const char **keys) const noexcept override
    {
        if (keys != nullptr)
            std::copy(std::begin(k_keys), std::end(k_keys), keys);
        return int32_t(std::size(k_keys));
    }

    // Disableable percentages, disabled at 0% by default, as in the legacy slicer.
    void inilialize_impl(storage_handle *) const override
    {
        for (int setting = 0; setting < SettingCount; ++setting) {
            raw_config_option_def def = raw_config_option_def_init();
            def.opt_key = k_keys[setting];
            def.type = RAW_CO_PERCENT;
            def.category = RAW_OPTION_CATEGORY_FILAMENT;
            def.level = RAW_OPTION_LEVEL_EXPERT;
            def.container_type = RAW_CONTAINER_TYPE_REGION;
            def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
            def.printer_technology = RAW_PT_FFF;
            def.invalidates_step = STEP_LAYER_EXTRUSION_EDIT;
            def.label = def.full_label = k_labels[setting];
            def.tooltip = k_tooltips[setting];
            def.sidetext = "%";
            def.can_be_disabled = 1;
            def.has_min = 1;
            def.min_value = 0.;
            def.has_max = 1;
            def.max_value = 100.;
            def.mode = raw_config_option_mode(RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI);
            def.default_serialized_value = "!0%";
            const option_def_error_code result = orchestrator_create_option_def(m_orchestrator, &def);
            if (result != OPTION_DEF_ERROR_OK)
                throw std::runtime_error(std::string("Cannot define ") + k_keys[setting] +
                                         " (option definition error " + std::to_string(int(result)) + ").");
        }
    }

    // Process one layer group. Every storage-backed object created here is local
    // to this call; the host may run several layer groups at the same time.
    void run_impl(const plugin_run_context *run_ctx) const override
    {
        throw_if_cancelled(run_ctx);
        const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
        if (ctx == nullptr || ctx->print == nullptr || ctx->plan == nullptr || ctx->layer_group == nullptr ||
            run_ctx->plugin_storage == nullptr)
            throw std::runtime_error("Regional fan received an invalid layer-group callback context.");

        const Print print(ctx->print);
        PrintingPlan plan(ctx->plan);
        PrintingLayerGroup layer_group(ctx->layer_group);
        std::map<SourceKey, SourceRequests> sources;
        for (uint32_t tool_idx = 0; tool_idx < layer_group.tool_group_count(); ++tool_idx) {
            PrintingToolGroup tool_group = layer_group.tool_group(tool_idx);
            for (uint32_t extrusion_idx = 0; extrusion_idx < tool_group.extrusion_count(); ++extrusion_idx) {
                throw_if_cancelled(run_ctx);
                PrintingExtrusion extrusion = tool_group.extrusion(extrusion_idx);
                const LayerRegionIsland region_island = extrusion.region_island();
                if (!region_island.valid())
                    continue;
                const SourceKey key{ region_island.handle(), extrusion.printing_catalog_id() };
                auto found = sources.find(key);
                if (found == sources.end())
                    found = sources.emplace(key, build_source_requests(run_ctx->plugin_storage, print, plan,
                                                                       extrusion, region_island)).first;
                const SourceRequests &source = found->second;
                if (!source.any())
                    continue;
                // The cache keeps the masks alive while these borrowed views are used.
                std::vector<ExPolygonCollection> views;
                views.reserve(source.areas.size());
                for (const StoredExPolygonCollection &area : source.areas)
                    views.push_back(area.readonly());
                apply_tree(extrusion.mutable_root(), std::nullopt, EPropertyFan{}, source, views, run_ctx);
            }
        }
    }
};

} // namespace
} // namespace nematx_regional_fan

// Host ABI entry point. Register a static instance so it stays alive for every callback.
extern "C" SLIC3R_PLUGIN_API void register_plugin(orchestrator_handle *orchestrator)
{
    static nematx_regional_fan::RegionalFan plugin(orchestrator);
    orchestrator_register_plugin(orchestrator, plugin.c_instance());
}

// Keep this header last so its ABI registration includes every public API header above.
#include "libslic3r/Api/plugin/c/slic3r_plugin_register_version.h"
