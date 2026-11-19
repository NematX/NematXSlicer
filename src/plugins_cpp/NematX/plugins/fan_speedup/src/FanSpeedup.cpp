///|/ Copyright (c) 2026 NematX
///|/
///|/ Released under the GNU Affero General Public License, version 3 or later.

/*
Fan anticipation and kickstart
==============================

A fan needs time to reach its speed. This plugin changes WHEN the fan requests of
the print take effect; it never changes a speed, adds a wait or touches the geometry
(the cuts keep the path, the Z, the arcs and the extruded volume).

Three host settings drive it, all disabled by default (printer, firmware category):
- fan_speedup_time: a rise of the fan speed is moved earlier by up to this many
  seconds of modeled print time (its absolute value, at least 0.01 s);
- fan_speedup_overhangs: the anticipation only applies when the path that asks for
  the rise is an overhang perimeter;
- fan_kickstart: a rise to a speed below 100% starts with 100% for
  fan_kickstart * rise / 100 seconds, then the target.
The host defines these options but needs a plugin to apply them: a non-zero
fan_speedup_time or fan_kickstart makes the export fail while this package is not
active (the "fan_temporal_plugin" group). The plugin defines no option.

The plugin runs at LAYER_EXTRUSION_EDIT_PRIORITY_FAN_TEMPORAL (40), after all the fan
policies, which have written their requests on the leaves. FanTimeline.hpp decides
the timing on a timeline of seconds; FanSpeedupTree.hpp builds that timeline from
each tool group and writes the answer in the tree. This file only reads the
settings and organizes the passes:
- setup() (serialized, read only) follows the fan target through the whole plan to
  find the target in force when each tool group starts, and keeps it in a cache that
  is cleared at every setup;
- run() edits only its own layer group: for each tool group it builds the timeline,
  plans and applies. With a zero delay and a zero kickstart nothing is walked.

Every mode is anticipated and kicked, as in the legacy slicer: a moved request keeps
the {percent, mode} pair of the request it comes from, an Override or a plugin mode
included. Nothing is anticipated across a tool group, a barrier (custom G-code,
special command, heater, fan event of a service) or the start of a layer.

Placement. A fan needs about 100 ms of precision: a change never creates a piece of
path shorter than 0.1 s (k_placement_tolerance in FanSpeedupTree.hpp), it goes to the
nearest boundary instead, and a kick shorter than that is stretched to the next place
where it can be written. See the header of FanSpeedupTree.hpp.

One warning is reported per export when a kick is dropped, or shortened by a barrier
beyond the placement tolerance: a barrier or a path that cannot be cut left no room
for its return, or a kick shorter than the tolerance ended a tool group. A change
moved within the tolerance (a return kept 0.1 s before a barrier), or to a boundary, even a
far one (a path that cannot be cut), is not reported.
*/

#include <algorithm>
#include <atomic>
#include <cmath>
#include <iterator>
#include <map>
#include <optional>
#include <stdexcept>

#include "FanSpeedupTree.hpp"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_plugin.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"

namespace nematx_fan_speedup {
namespace {

using namespace slic3r_api;

// Identifier of the plugin and keys of the host options it reads.
constexpr const char *k_plugin_id = "nematx_fan_speedup";
constexpr const char *k_time_key = "fan_speedup_time";
constexpr const char *k_overhangs_key = "fan_speedup_overhangs";
constexpr const char *k_kickstart_key = "fan_kickstart";
constexpr const char *k_travel_speed_key = "travel_speed";

// The plugin depends on no other plugin.
const char *const k_no_dependencies[] = { nullptr };
// Options read by this plugin and defined by the host. The host refuses a non-zero
// fan_speedup_time or fan_kickstart unless the selected plugin of the group lists it here.
const raw_used_config_key k_used_config_keys[] = {
    { k_time_key, RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { k_overhangs_key, RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { k_kickstart_key, RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { k_travel_speed_key, RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE }
};

// What the plugin reads from the print configuration.
struct Parameters
{
    nematx_fan_speedup::Settings settings;
    double travel_speed;

    // Both functions disabled: the plugin walks nothing and changes nothing. A value that
    // is not zero but not valid (NaN, negative kickstart) is not "disabled": it goes on to
    // the planner, which rejects it with an error.
    bool enabled() const { return settings.delay != 0. || settings.kickstart != 0.; }
};

// Reads the settings of the print: the delay is normalized (absolute value, at least
// 0.01 s when not zero) and a negative kickstart is refused by the host definition.
Parameters read_parameters(const Config &config);

Parameters read_parameters(const Config &config)
{
    Parameters parameters;
    parameters.settings.delay = normalized_delay(config.float_or_default(k_time_key, 0.));
    parameters.settings.overhangs_only = config.bool_or_default(k_overhangs_key, true);
    parameters.settings.kickstart = config.float_or_default(k_kickstart_key, 0.);
    parameters.travel_speed = config.float_or_default(k_travel_speed_key, 0.);
    return parameters;
}

class FanSpeedup final : public PluginBase
{
public:
    explicit FanSpeedup(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

private:
    const char *id_impl() const noexcept override { return k_plugin_id; }
    const char *name_impl() const noexcept override { return "Fan anticipation and kickstart"; }
    const char *description_impl() const noexcept override
    {
        return "Moves the fan speed rises earlier and kick-starts the fan, as the legacy fan_speedup_time and fan_kickstart.";
    }
    // The host options fan_speedup_time and fan_kickstart require a provider of this group.
    const char *exclusive_group_impl() const noexcept override { return "fan_temporal_plugin"; }
    slicing_step_t step_impl() const noexcept override { return STEP_LAYER_EXTRUSION_EDIT; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    int32_t priority_impl() const noexcept override { return LAYER_EXTRUSION_EDIT_PRIORITY_FAN_TEMPORAL; }

    // Host protocol: called with null to get the number of keys, then with an array of that
    // size to fill. The keys are borrowed static strings.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        if (keys != nullptr)
            std::copy(std::begin(k_used_config_keys), std::end(k_used_config_keys), keys);
        return int32_t(std::size(k_used_config_keys));
    }

    // Serialized and read only: clear the state of the previous export, then compute
    // the target in force at the start of every tool group with a pass over the plan.
    void setup_impl(const plugin_run_context *run_ctx, uint32_t) const override
    {
        m_warned.store(false);
        m_entries.clear();
        const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
        if (ctx == nullptr || ctx->print == nullptr || ctx->plan == nullptr)
            throw std::runtime_error("The fan timing received an invalid setup context.");
        if (read_parameters(Print(ctx->print).config()).enabled())
            m_entries = group_entries(ConstPrintingPlan(ctx->plan));
    }

    // Process one layer group; it edits only its own tool groups. The cache is only
    // read here, so parallel layer groups need no lock.
    void run_impl(const plugin_run_context *run_ctx) const override
    {
        throw_if_cancelled(run_ctx);
        const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
        if (ctx == nullptr || ctx->print == nullptr || ctx->layer_group == nullptr || run_ctx->plugin_storage == nullptr)
            throw std::runtime_error("The fan timing received an invalid layer-group callback context.");
        const Parameters parameters = read_parameters(Print(ctx->print).config());
        if (!parameters.enabled())
            return;

        PrintingLayerGroup layer_group(ctx->layer_group);
        for (uint32_t tool_idx = 0; tool_idx < layer_group.tool_group_count(); ++tool_idx) {
            throw_if_cancelled(run_ctx);
            PrintingToolGroup tool_group = layer_group.tool_group(tool_idx);
            const std::optional<float> entry = m_entries.at({ ctx->group_idx, ctx->layer_group_idx, tool_idx });
            if (edit_tool_group(run_ctx->plugin_storage, tool_group, parameters.settings, entry, parameters.travel_speed) &&
                !m_warned.exchange(true))
                report_warning(run_ctx,
                    "Fan timing: a fan kick was dropped, or shortened by a barrier beyond the placement tolerance, because a barrier (custom G-code, command, "
                    "heater or fan event) or a path that cannot be cut left no room for its return. A kick shorter than the "
                    "placement tolerance of 0.1 s is also dropped when it ends a tool group. Further kicks may be affected too.");
        }
    }

    // Target in force at the start of each tool group; written by setup(), read by run().
    mutable std::map<GroupKey, std::optional<float>> m_entries;
    // Set once the warning of this export has been reported.
    mutable std::atomic<bool> m_warned{ false };
};

} // namespace
} // namespace nematx_fan_speedup

// Register a static instance and copy its group description into the host before settings are built.
// The host borrows the instance for every callback; a rejected description throws.
extern "C" SLIC3R_PLUGIN_API void register_plugin(orchestrator_handle *orchestrator)
{
    static nematx_fan_speedup::FanSpeedup plugin(orchestrator);
    orchestrator_register_plugin(orchestrator, plugin.c_instance());
    raw_exclusive_group_def definition = raw_exclusive_group_def_init();
    definition.exclusive_group = "fan_temporal_plugin";
    definition.label = "Fan timing";
    definition.tooltip = "Selects the plugin that applies fan_speedup_time and fan_kickstart.";
    plugin.register_exclusive_group(STEP_LAYER_EXTRUSION_EDIT, definition);
}

// Keep this header last so its ABI registration includes every public API header above.
#include "libslic3r/Api/plugin/c/slic3r_plugin_register_version.h"
