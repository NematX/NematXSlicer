///|/ Copyright (c) 2026 NematX
///|/
///|/ Released under the GNU Affero General Public License, version 3 or later.

/*
Smoothed temperature changes
============================

The region setting print_temperature_smooth_change (N, default 0) spreads a
change of the general nozzle temperature of an object instance over N layers
instead of applying it at once. The ramp arithmetic is in TemperatureRamp.hpp:
a request restarts the ramp from the value printed on the previous layer, the
request layer takes the first step and the target is reached on the N-th layer.

SmoothTemperature is a member of the host's layer_extrusion_edit.temperature
exclusive group, preferred to the host provider when both are active. It
derives from the host LayerTemperature (DefaultTemperature.hpp): the base setup
plans every heater request in print order, this plugin then rewrites the nozzle
setpoints of those decisions before any layer run, and the inherited run
appends each layer's decisions to the plan. The standalone package compiles the
two host sources of that planner into its own library, because the host library
exports only the C API.

General temperature and overrides. Each instance has one ramp per tool, which
follows its general temperature layer by layer. When all regional roots of the
instance and tool on a layer ask for the same temperature, it is general: object
settings, height ranges, or a modifier covering the whole instance on that
layer. Otherwise, the general temperature comes from the object settings and the
height range of the layer, and the roots asking for another temperature, such
as those of a partial modifier, are overrides: they are written as they are and
never move the ramp. The base standby requests of ooze prevention and its
second-layer requests for idle tools are kept as they are.

The pass reads the plan and edits only the cached decisions, layer by layer in
plan order:
- read every regional root of the layer: instance, tool and temperature;
- update the ramp of each instance and tool printed on the layer: created at
  once on its first layer, restarted from its current value when its general
  temperature changes, otherwise advanced by one layer. A tool's ramp waits on
  the layers where it does not print the instance;
- walk the requests in firmware order (layer requests, then for each visit the
  incoming setpoint, the roots and the standby) while tracking the last setpoint
  of each tool. The incoming setpoint after a tool change takes the value of the
  visit's first regional root and keeps the base wait; a root writes its value
  only when it differs from the last setpoint. No wait is added.

With N <= 1 everywhere, every value is the base target and the output is the
host provider's. In SEMM the base decisions are kept: the wipe tower resets the
shared heater state there.

Call tree:
    register_plugin
    SmoothTemperature::inilialize_impl            defines the setting
    SmoothTemperature::setup_impl
        LayerTemperature::setup_impl              base heater decisions
        smooth_decisions                          each layer group in plan order
            collect_roots                         regional roots and their instances
            general_target                        general temperature and N of one instance
                layer_range_config                height range containing the layer
            InstanceRamp::created / request / advance
            rewrite the requests in firmware order
    LayerTemperature::run_impl                    inherited: appends the decisions
*/

#include <algorithm>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "TemperatureRamp.hpp"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_plugin.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/VolumeViews.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/DefaultTemperature.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/ExtrusionProcessParameterHelpers.hpp"

namespace nematx_smooth_temperature {
namespace {

using namespace slic3r_api;
using namespace slic3r_api::LayerExtrusionEdit::DefaultTemperaturePlugin;
namespace ProcessParameterHelpers = slic3r_api::LayerExtrusionEdit::ProcessParameterHelpers;
using ProcessParameterHelpers::ExtrusionSettingsContext;

constexpr const char *k_plugin_id = "nematx_smooth_temperature";
constexpr const char *k_layers_key = "print_temperature_smooth_change";

// Ramp of one object instance on one tool: plan catalog id of the instance, then tool.
using RampKey = std::pair<uint32_t, extruder_index_t>;

// One regional root of a layer, in visit order.
struct RootTarget
{
    // Visit containing the root.
    uint32_t tool_idx = 0;
    // Root index in its visit.
    uint32_t extrusion_idx = 0;
    // Instance and tool printing the root.
    RampKey key;
    // Temperature asked by the root's region.
    int16_t target = 0;
};

// Roots of one instance and tool on one layer.
struct InstanceLayer
{
    // Settings of the first root: print, object and region configurations, first-layer status.
    ExtrusionSettingsContext context;
    // Source layer of the first root.
    Layer layer;
    // Distinct temperatures asked by the roots.
    std::set<int16_t> targets;
    // General temperature of the instance on this layer, when one is configured.
    std::optional<int16_t> general;
};

// Rewrite the nozzle setpoints of the base decisions along the instance ramps.
void smooth_decisions(const Print &print, PrintingPlan &plan, LayerDecisions &decisions);

// Read the regional roots of one layer, in visit order, and group them by instance and tool.
void collect_roots(const Print &print,
                   PrintingPlan &plan,
                   PrintingLayerGroup &layer,
                   std::vector<RootTarget> &roots,
                   std::map<RampKey, InstanceLayer> &instances);

// Resolve the general temperature of one instance on one tool and layer, with the ramp length N read beside it.
std::pair<std::optional<int16_t>, uint32_t> general_target(const Print &print,
                                                           extruder_index_t tool,
                                                           const InstanceLayer &instance);

// Return the configuration of the object's height range containing slice_z; a shared boundary belongs to the upper range.
std::optional<Config> layer_range_config(const Object &object, coord_t slice_z);

class SmoothTemperature final : public LayerTemperature
{
public:
    // Initialize the base planner without owning plan or print data.
    explicit SmoothTemperature(orchestrator_handle *orchestrator) : LayerTemperature(orchestrator) {}

private:
    const char *id_impl() const noexcept override { return k_plugin_id; }
    const char *name_impl() const noexcept override { return "Smoothed layer temperatures"; }
    const char *description_impl() const noexcept override
    {
        return "Spreads changes of the general nozzle temperature of each object over "
               "print_temperature_smooth_change layers.";
    }
    // Preferred to the host provider (100) in the layer temperature group.
    int32_t exclusive_group_priority_impl() const noexcept override { return 50; }

    // Copy the base planner's options, then the ramp length.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        const int32_t count = LayerTemperature::used_config_keys(keys);
        if (keys != nullptr)
            keys[count] = {k_layers_key, RAW_CO_INT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT};
        return count + 1;
    }
    int32_t defined_config_keys(const char **keys) const noexcept override
    {
        if (keys != nullptr)
            keys[0] = k_layers_key;
        return 1;
    }

    // Define the legacy region setting; a rejected definition throws.
    void inilialize_impl(storage_handle *) const override
    {
        raw_config_option_def def = raw_config_option_def_init();
        def.opt_key = k_layers_key;
        def.type = RAW_CO_INT;
        def.category = RAW_OPTION_CATEGORY_FILAMENT;
        def.level = RAW_OPTION_LEVEL_EXPERT;
        def.container_type = RAW_CONTAINER_TYPE_REGION;
        def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
        def.printer_technology = RAW_PT_FFF;
        def.invalidates_step = STEP_LAYER_EXTRUSION_EDIT;
        def.label = def.full_label = "Temperature smoothing layers";
        def.tooltip = "When the temperature changes, it gradually change over this number of layers instead of "
                      "going directly to the desired value.\nNote: does smooth first layer -> other layer transition "
                      "if set in the second layer.";
        def.sidetext = "layers";
        def.has_min = 1;
        def.min_value = 0.;
        def.mode = raw_config_option_mode(RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI);
        def.default_serialized_value = "0";
        const option_def_error_code result = orchestrator_create_option_def(m_orchestrator, &def);
        if (result != OPTION_DEF_ERROR_OK)
            throw std::runtime_error(std::string("Cannot define ") + k_layers_key + " (option definition error " +
                                     std::to_string(int(result)) + ").");
    }

    // Plan the base decisions, then smooth their nozzle setpoints outside SEMM.
    void setup_impl(const plugin_run_context *run_ctx, const uint32_t run_count) const override
    {
        LayerTemperature::setup_impl(run_ctx, run_count);
        if (!m_setup_valid)
            return;
        const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
        const Print print(ctx->print);
        if (print.config().bool_or_default("single_extruder_multi_material", false))
            return;
        PrintingPlan plan(ctx->plan);
        smooth_decisions(print, plan, m_decisions);
    }
};

void smooth_decisions(const Print &print, PrintingPlan &plan, LayerDecisions &decisions)
{
    std::map<RampKey, InstanceRamp> ramps;
    // Last setpoint of each tool in firmware order, whether kept from the base or written here.
    std::map<extruder_index_t, int16_t> written;
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        PrintingGroup group = plan.group(group_idx);
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            PrintingLayerGroup layer = group.layer_group(layer_idx);
            LayerTemperatureDecision &decision = decisions[group_idx][layer_idx];
            std::vector<RootTarget> roots;
            std::map<RampKey, InstanceLayer> instances;
            collect_roots(print, plan, layer, roots, instances);

            // Each instance and tool printed here counts one layer of its ramp.
            for (auto &[key, instance] : instances) {
                const std::pair<std::optional<int16_t>, uint32_t> general = general_target(print, key.second, instance);
                instance.general = general.first;
                if (!general.first)
                    continue;
                const std::map<RampKey, InstanceRamp>::iterator ramp = ramps.find(key);
                if (ramp == ramps.end())
                    ramps.emplace(key, InstanceRamp::created(*general.first));
                else if (ramp->second.target != *general.first)
                    ramp->second.request(*general.first, general.second);
                else
                    ramp->second.advance();
            }

            for (const HeaterRequest &request : decision.before)
                if (request.heater == HeaterKind::Tool)
                    written[request.tool] = request.temperature;
            size_t root_idx = 0;
            for (uint32_t tool_idx = 0; tool_idx < decision.visits.size(); ++tool_idx) {
                VisitTemperatureDecision &visit = decision.visits[tool_idx];
                const extruder_index_t tool = layer.tool_group(tool_idx).extruder_id();
                // A general root prints its instance's ramp value; an override keeps its own temperature.
                std::vector<std::pair<uint32_t, int16_t>> setpoints;
                for (; root_idx < roots.size() && roots[root_idx].tool_idx == tool_idx; ++root_idx) {
                    const RootTarget &root = roots[root_idx];
                    const InstanceLayer &instance = instances.at(root.key);
                    const bool general = instance.general && root.target == *instance.general;
                    setpoints.emplace_back(root.extrusion_idx, general ? ramps.at(root.key).value() : root.target);
                }
                for (HeaterRequest &request : visit.before) {
                    if (request.heater != HeaterKind::Tool)
                        continue;
                    if (request.tool == tool && !setpoints.empty())
                        request.temperature = setpoints.front().second;
                    written[request.tool] = request.temperature;
                }
                visit.roots.clear();
                for (const std::pair<uint32_t, int16_t> &setpoint : setpoints) {
                    const std::map<extruder_index_t, int16_t>::iterator last = written.find(tool);
                    if (last == written.end() || last->second != setpoint.second) {
                        visit.roots.push_back(setpoint);
                        written[tool] = setpoint.second;
                    }
                }
                for (const HeaterRequest &request : visit.after)
                    if (request.heater == HeaterKind::Tool)
                        written[request.tool] = request.temperature;
            }
        }
    }
}

void collect_roots(const Print &print,
                   PrintingPlan &plan,
                   PrintingLayerGroup &layer,
                   std::vector<RootTarget> &roots,
                   std::map<RampKey, InstanceLayer> &instances)
{
    for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
        PrintingToolGroup tool_group = layer.tool_group(tool_idx);
        const extruder_index_t tool = tool_group.extruder_id();
        for (uint32_t extrusion_idx = 0; extrusion_idx < tool_group.extrusion_count(); ++extrusion_idx) {
            PrintingExtrusion extrusion = tool_group.extrusion(extrusion_idx);
            // The base planner's roots: synthetic and region-less roots keep the current setpoint.
            if (!extrusion.region_island().valid() || ProcessParameterHelpers::is_regionless_synthetic(plan, extrusion))
                continue;
            const ExtrusionSettingsContext context = ProcessParameterHelpers::settings_context(print, tool_group, extrusion);
            const std::optional<int16_t> target = extrusion_target(context, tool);
            if (!target || *target <= 0)
                continue;
            const RampKey key{extrusion.printing_catalog_id(), tool};
            roots.push_back(RootTarget{tool_idx, extrusion_idx, key, *target});
            std::map<RampKey, InstanceLayer>::iterator instance = instances.find(key);
            if (instance == instances.end())
                instance = instances.emplace(key, InstanceLayer{context, extrusion.region_island().region(0).layer(), {},
                                                                std::nullopt}).first;
            instance->second.targets.insert(*target);
        }
    }
}

std::pair<std::optional<int16_t>, uint32_t> general_target(const Print &print,
                                                           const extruder_index_t tool,
                                                           const InstanceLayer &instance)
{
    // All roots agree: the object, its height range or a region covering the whole instance sets this temperature.
    if (instance.targets.size() == 1) {
        const int32_t layers = instance.context.region_config.int_or_default(k_layers_key, 0);
        return {*instance.targets.begin(), uint32_t(std::max(layers, 0))};
    }
    // Otherwise the object settings and the height range of the layer give it; the other roots are overrides.
    const Object object = instance.layer.object();
    const Config object_config = object.default_region_config(print);
    const std::optional<Config> range = layer_range_config(object, instance.layer.slice_z());
    const auto setting = [&object_config, &range](const char *key, const int32_t fallback) {
        return range && range->has(key) ? range->int_or_default(key, fallback) :
                                          object_config.int_or_default(key, fallback);
    };
    const bool first_layer = instance.context.first_layer;
    int32_t temperature = first_layer ? setting("print_first_layer_temperature", -1) : -1;
    if (temperature < 0)
        temperature = setting("print_temperature", -1);
    const std::optional<int16_t> general = temperature >= 0 ?
        std::optional<int16_t>(int16_t(temperature)) :
        filament_target(instance.context.print_config, tool, first_layer);
    return {general, uint32_t(std::max(setting(k_layers_key, 0), 0))};
}

std::optional<Config> layer_range_config(const Object &object, const coord_t slice_z)
{
    std::optional<Config> selected;
    for (uint32_t range_idx = 0; range_idx < object_slicing_layer_range_count(object); ++range_idx) {
        const SlicingLayerRange range = object_slicing_layer_range(object, range_idx);
        if (range.z_min() <= slice_z && slice_z <= range.z_max()) {
            selected = range.config();
            if (slice_z < range.z_max())
                break;
        }
    }
    return selected;
}

} // namespace
} // namespace nematx_smooth_temperature

// Host ABI entry point. Register a static instance so it stays alive for every callback.
extern "C" SLIC3R_PLUGIN_API void register_plugin(orchestrator_handle *orchestrator)
{
    static nematx_smooth_temperature::SmoothTemperature plugin(orchestrator);
    orchestrator_register_plugin(orchestrator, plugin.c_instance());
}

// Keep this header last so its ABI registration includes every public API header above.
#include "libslic3r/Api/plugin/c/slic3r_plugin_register_version.h"
