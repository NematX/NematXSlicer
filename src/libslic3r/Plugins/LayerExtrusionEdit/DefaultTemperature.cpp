///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/


#include "DefaultTemperature.hpp"

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

#include "ExtrusionProcessParameterHelpers.hpp"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_extrusions.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_extrusion_edit.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/ExtrusionProperties.hpp"

/*
Default temperature planning
=============================

The plan-level provider adds one typed heater event for each enabled startup
request after the configured start script.  The layer-level provider first
walks visits and extrusion roots in print order, because waiting decisions
depend on the previous heater history; its parallel layer runs only materialize
the decisions recorded by that sequential setup.  Events can only be appended
at the end of a scope, so standby is appended after the last visit of tool A
and the target and wait for tool B are appended after B's TOOLCHANGE events.
In SEMM, a visit beginning with a wipe-tower root does not add a tool-change
event; after that tower change the shared heater state is made unknown before
ordinary roots are inspected.  When the default startup provider is active
and selected for its exclusive group, startup planning initializes only the
known temperature of the initial tool for layer tracking, never a target, bed,
or chamber state, so the first root always asks for its target and the firmware
removes an identical duplicate.  Regionless synthetic roots and other roots
without a region are ignored by the root loop
because settings_context needs a region; their current target is retained
instead of causing temperature oscillation.  The chamber is a startup-only
event and is not tracked here.
*/

namespace slic3r_api { namespace LayerExtrusionEdit { namespace DefaultTemperaturePlugin {
namespace {

using ProcessParameterHelpers::ExtrusionSettingsContext;

const char *const k_no_dependencies[] = {nullptr};

// Wait when the new target is at least 5 °C above the known reached temperature.
const int32_t k_wait_temperature_margin_celsius = 5;

// Stable identifier of the built-in startup-temperature provider.
const char *const k_startup_plugin_id = "extrusion_edit.startup_temperature.default";
// Exclusive group containing the startup-temperature providers.
const char *const k_startup_group_id = "extrusion_edit.startup_temperature";

enum class HeaterKind
{
    Tool,
    Bed,
    Chamber
};

struct HeaterRequest
{
    // Physical heater category receiving this operation.
    HeaterKind heater = HeaterKind::Tool;
    // Tool address, or the unknown marker for bed and chamber operations.
    extruder_index_t tool = EXTRUDER_ID_UNKNOWN;
    // Requested temperature in degrees Celsius, including zero for standby.
    int16_t temperature = 0;
    // Whether firmware must wait until the requested temperature is reached.
    bool wait = false;
};

struct StartupToolTarget
{
    // Concrete tool receiving a non-waiting startup request.
    extruder_index_t tool = EXTRUDER_ID_UNKNOWN;
    // Startup target or positive standby temperature in degrees Celsius.
    int16_t temperature = 0;
};

struct StartupHeating
{
    // All concrete tools appearing in the ordered plan, including later layers.
    std::set<extruder_index_t> used_tools;
    // First concrete tool visit in plan order, or the unknown marker for no plan.
    extruder_index_t initial_tool = EXTRUDER_ID_UNKNOWN;
    // Positive first-layer bed request, when startup heating is enabled.
    std::optional<int16_t> bed;
    // Positive initial-tool chamber request, when startup heating is enabled.
    std::optional<int16_t> chamber;
    // Positive initial-tool filament request, when startup heating is enabled.
    std::optional<int16_t> initial_tool_temperature;
    // Positive non-initial multi-nozzle requests emitted without waiting.
    std::vector<StartupToolTarget> other_tools;
};

struct TrackedHeater
{
    // Last target requested by this plan, whether or not it was waited for.
    std::optional<int16_t> target;
    // Last temperature known to have been reached by a waited request.
    std::optional<int16_t> known;
};

struct VisitTemperatureDecision
{
    // Requests appended before this visit, normally the incoming tool target.
    std::vector<HeaterRequest> before;
    // Requests appended after this visit, normally standby for its leaving tool.
    std::vector<HeaterRequest> after;
    // Root index and target for non-synthetic regional roots in visit order.
    std::vector<std::pair<uint32_t, int16_t>> roots;
};

struct LayerTemperatureDecision
{
    // Requests appended after the layer's existing before events.
    std::vector<HeaterRequest> before;
    // Per-tool-visit requests and root edits for this layer.
    std::vector<VisitTemperatureDecision> visits;
};

using LayerDecisions = std::vector<std::vector<LayerTemperatureDecision>>;

struct PreviousVisit
{
    // True after at least one concrete tool visit has been processed.
    bool valid = false;
    // Group containing the previous visit, used to attach its standby request.
    uint32_t group_idx = 0;
    // Layer containing the previous visit, used to attach its standby request.
    uint32_t layer_idx = 0;
    // Tool-group position containing the previous visit.
    uint32_t tool_idx = 0;
    // Concrete tool identifier of the previous visit.
    extruder_index_t tool = EXTRUDER_ID_UNKNOWN;
    // First-layer status of the previous visit's last regional source, or its group-layer fallback.
    bool first_layer = false;
};

// Compute startup requests and collect used tools from the ordered plan.
StartupHeating startup_heating(const Config &config, PrintingPlan &plan);

// Resolve the first regional extrusion target in one visit, with filament fallback.
std::optional<int16_t> visit_target(const Print &print,
                                    PrintingPlan &plan,
                                    PrintingToolGroup &tool_group,
                                    bool first_layer);

// Identify a SEMM visit beginning with a regionless wipe-tower root.
bool tower_visit(PrintingToolGroup &tool_group);

// Return true when startup heater requests are permitted by both policy switches.
bool startup_enabled(const Config &config);

// Resolve a configured idle temperature or standby delta for one filament target.
std::optional<int16_t> standby_target(const Config &config,
                                      extruder_index_t tool,
                                      const std::optional<int16_t> &base);

// Read regional overrides before falling back to filament temperatures.
std::optional<int16_t> extrusion_target(const ExtrusionSettingsContext &context,
                                        extruder_index_t tool);

// Read first-layer and normal filament temperatures for one concrete tool.
std::optional<int16_t> filament_target(const Config &config,
                                       extruder_index_t tool,
                                       bool first_layer);

// Convert a non-negative configuration value to the signed heater ABI value.
std::optional<int16_t> heater_temperature(int32_t value);

// Update one heater's target and known reached value after a planned request.
void apply_tracked_request(TrackedHeater &state, int16_t temperature, bool wait);

// Select the physical heater shared by one logical tool in normal mode or SEMM.
TrackedHeater &tracked_tool(std::map<extruder_index_t, TrackedHeater> &tools,
                            extruder_index_t tool,
                            bool semm);

// Store a heater operation in an owned event root and append it at the end of a fixed event phase.
void append_heater_event(storage_handle *storage,
                         PrintingScopeEvents events,
                         const HeaterRequest &request,
                         bool before);

class StartupTemperature final : public PluginBase
{
public:
    // Return the singleton startup provider registered in the current orchestrator.
    static StartupTemperature &instance(orchestrator_handle *orchestrator);

    // Initialize the generic plugin facade without owning plan or print data.
    explicit StartupTemperature(orchestrator_handle *orchestrator);

private:
    // Return the stable startup provider identifier.
    const char *id_impl() const noexcept override;
    // Return the startup provider display name.
    const char *name_impl() const noexcept override;
    // Describe the startup provider's responsibility.
    const char *description_impl() const noexcept override;
    // Select the exclusive startup-temperature group.
    const char *exclusive_group_impl() const noexcept override;
    // Label the startup-temperature group.
    const char *exclusive_group_label_impl() const noexcept override;
    // Explain the startup-temperature selection group.
    const char *exclusive_group_tooltip_impl() const noexcept override;
    // Run after start scripts in the plan-level extrusion edit step.
    slicing_step_t step_impl() const noexcept override;
    // Declare that startup temperature has no plugin dependency.
    const char *const *dependencies_impl() const noexcept override;
    // Place startup events after SettingsGCodeScripts.
    int32_t priority_impl() const noexcept override;
    // Copy every configuration option read by startup planning.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    // Startup temperature does not define a new user setting.
    int32_t defined_config_keys(const char **keys) const noexcept override;
    // Initialize without creating dynamic options.
    void inilialize_impl(storage_handle *) const override;
    // Describe plan-level startup progress.
    const char *progress_message_format_impl() const noexcept override;
    // Append startup heater events to the plan's before sequence.
    void run_impl(const plugin_run_context *run_ctx) const override;
};

class LayerTemperature final : public PluginBase
{
public:
    // Return the singleton layer provider registered in the current orchestrator.
    static LayerTemperature &instance(orchestrator_handle *orchestrator);

    // Initialize the generic plugin facade without owning plan or print data.
    explicit LayerTemperature(orchestrator_handle *orchestrator);

private:
    // Return the stable layer provider identifier.
    const char *id_impl() const noexcept override;
    // Return the layer provider display name.
    const char *name_impl() const noexcept override;
    // Describe sequential target tracking and layer event placement.
    const char *description_impl() const noexcept override;
    // Select the exclusive layer-temperature group.
    const char *exclusive_group_impl() const noexcept override;
    // Label the layer-temperature group.
    const char *exclusive_group_label_impl() const noexcept override;
    // Explain the layer-temperature selection group.
    const char *exclusive_group_tooltip_impl() const noexcept override;
    // Run in the layer extrusion edit step.
    slicing_step_t step_impl() const noexcept override;
    // Declare that layer temperature has no plugin dependency.
    const char *const *dependencies_impl() const noexcept override;
    // Run after the default fan provider and tool-change producer.
    int32_t priority_impl() const noexcept override;
    // Copy every configuration option read by layer temperature planning.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    // Layer temperature does not define a new user setting.
    int32_t defined_config_keys(const char **keys) const noexcept override;
    // Initialize without creating dynamic options.
    void inilialize_impl(storage_handle *) const override;
    // Describe layer-level temperature progress.
    const char *progress_message_format_impl() const noexcept override;
    // Record all sequential heater decisions before parallel layer runs begin.
    void setup_impl(const plugin_run_context *run_ctx, uint32_t run_count) const override;
    // Count the current layer as one materialization unit.
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    // Apply only the cached decisions for the current layer.
    void run_impl(const plugin_run_context *run_ctx) const override;

    // True when setup completed and cached decisions match the current plan.
    mutable bool m_setup_valid = false;
    // Per-group and per-layer decisions materialized by parallel layer runs.
    mutable LayerDecisions m_decisions;
};

// Validate a non-negative configuration temperature and return the ABI-sized value; negative values disable a request and oversized values fail.
std::optional<int16_t> heater_temperature(const int32_t value)
{
    if (value < 0)
        return std::nullopt;
    if (value > (std::numeric_limits<int16_t>::max)())
        throw std::invalid_argument("A configured heater temperature is out of range.");
    return static_cast<int16_t>(value);
}

// Read first-layer then normal filament targets for one tool; the borrowed configuration is unchanged and non-positive values produce no target.
std::optional<int16_t> filament_target(const Config &config,
                                       const extruder_index_t tool,
                                       const bool first_layer)
{
    if (first_layer) {
        const int32_t first = config.vector_int_or_default("first_layer_temperature", tool, -1);
        if (first > 0)
            return heater_temperature(first);
    }
    const int32_t normal = config.vector_int_or_default("temperature", tool, -1);
    if (normal > 0)
        return heater_temperature(normal);
    return std::nullopt;
}

// Resolve the borrowed source layer's regional overrides before filament fallback using its first-layer status; return no target for disabled values, propagate conversion errors, and leave all state unchanged.
std::optional<int16_t> extrusion_target(const ExtrusionSettingsContext &context,
                                        const extruder_index_t tool)
{
    if (context.first_layer) {
        const int32_t override_temperature = context.region_config.int_or_default(
            "print_first_layer_temperature", -1);
        if (override_temperature >= 0)
            return heater_temperature(override_temperature);
    }
    const int32_t override_temperature = context.region_config.int_or_default(
        "print_temperature", -1);
    if (override_temperature >= 0)
        return heater_temperature(override_temperature);
    return filament_target(context.print_config, tool, context.first_layer);
}

// Resolve an enabled idle target, including zero, or a nonzero standby delta; a missing fallback produces no operation.
std::optional<int16_t> standby_target(const Config &config,
                                      const extruder_index_t tool,
                                      const std::optional<int16_t> &base)
{
    const int32_t idle = config.vector_int_or_default("idle_temperature", tool, -1);
    if (idle >= 0) {
        return heater_temperature(idle);
    }

    const int32_t delta = config.int_or_default("standby_temperature_delta", 0);
    if (delta == 0 || !base.has_value())
        return std::nullopt;
    const int32_t standby = int32_t(*base) + delta;
    if (standby <= 0)
        return std::nullopt;
    return heater_temperature(standby);
}

// Check the two profile switches that permit automatic startup heater events; the read-only configuration is not changed.
bool startup_enabled(const Config &config)
{
    return !config.bool_or_default("start_gcode_manual", false) &&
           config.bool_or_default("autoemit_temperature_commands", true);
}

// Recognize a SEMM wipe-tower visit from its first root role and absent source region; borrowed plan views remain unchanged.
bool tower_visit(PrintingToolGroup &tool_group)
{
    if (tool_group.extrusion_count() == 0)
        return false;
    const PrintingExtrusion first = tool_group.extrusion(0);
    return RAW_EXTRUSION_ROLE_HAS(first.role(), RAW_EXTRUSION_ROLE_WIPE_TOWER) &&
           !first.region_island().valid();
}

// Scan ordered tool visits and compute bed, chamber, initial-tool, and standby startup targets; invalid tool IDs fail and the plan is only read.
StartupHeating startup_heating(const Config &config, PrintingPlan &plan)
{
    StartupHeating heating;
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        PrintingGroup group = plan.group(group_idx);
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            PrintingLayerGroup layer = group.layer_group(layer_idx);
            for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
                PrintingToolGroup tool_group = layer.tool_group(tool_idx);
                const extruder_index_t tool = tool_group.extruder_id();
                if (tool == EXTRUDER_ID_UNKNOWN)
                    throw std::runtime_error("Temperature planning needs concrete tool ids.");
                heating.used_tools.insert(tool);
                if (heating.initial_tool == EXTRUDER_ID_UNKNOWN)
                    heating.initial_tool = tool;
            }
        }
    }
    if (!startup_enabled(config) || heating.initial_tool == EXTRUDER_ID_UNKNOWN)
        return heating;

    PrintingGroup first_group = plan.group(0);
    PrintingLayerGroup first_layer = first_group.layer_group(0);
    int32_t bed_temperature = config.int_or_default("print_first_layer_bed_temperature", -1);
    if (bed_temperature < 0) {
        for (uint32_t tool_idx = 0; tool_idx < first_layer.tool_group_count(); ++tool_idx) {
            PrintingToolGroup tool_group = first_layer.tool_group(tool_idx);
            bool has_real_extrusion = false;
            for (uint32_t extrusion_idx = 0;
                 extrusion_idx < tool_group.extrusion_count(); ++extrusion_idx) {
                const PrintingExtrusion extrusion = tool_group.extrusion(extrusion_idx);
                if (!ProcessParameterHelpers::is_regionless_synthetic(plan, extrusion)) {
                    has_real_extrusion = true;
                    break;
                }
            }
            if (has_real_extrusion)
                bed_temperature = std::max(
                    bed_temperature,
                    config.vector_int_or_default(
                        "first_layer_bed_temperature", tool_group.extruder_id(), 0));
        }
    }
    if (const std::optional<int16_t> bed = heater_temperature(bed_temperature); bed && *bed > 0)
        heating.bed = bed;

    const int32_t chamber = config.vector_int_or_default(
        "chamber_temperature", heating.initial_tool, 0);
    if (const std::optional<int16_t> value = heater_temperature(chamber); value && *value > 0)
        heating.chamber = value;

    heating.initial_tool_temperature = filament_target(config, heating.initial_tool, true);
    const bool semm = config.bool_or_default("single_extruder_multi_material", false);
    if (!semm) {
        for (const extruder_index_t tool : heating.used_tools) {
            if (tool == heating.initial_tool)
                continue;
            const std::optional<int16_t> base = filament_target(config, tool, true);
            std::optional<int16_t> target = base;
            if (config.bool_or_default("ooze_prevention", false)) {
                target = standby_target(config, tool, base);
                if (!target)
                    target = base;
            }
            if (target && *target > 0)
                heating.other_tools.push_back(StartupToolTarget{tool, *target});
        }
    }
    return heating;
}

// Read the first regional root's source-layer target without mutating the borrowed plan; visits without regional roots retain the group-layer filament fallback, and configuration errors propagate.
std::optional<int16_t> visit_target(const Print &print,
                                    PrintingPlan &plan,
                                    PrintingToolGroup &tool_group,
                                    const bool first_layer)
{
    const extruder_index_t tool = tool_group.extruder_id();
    for (uint32_t extrusion_idx = 0; extrusion_idx < tool_group.extrusion_count(); ++extrusion_idx) {
        PrintingExtrusion extrusion = tool_group.extrusion(extrusion_idx);
        if (!extrusion.region_island().valid() ||
            ProcessParameterHelpers::is_regionless_synthetic(plan, extrusion))
            continue;
        ExtrusionSettingsContext context = ProcessParameterHelpers::settings_context(
            print, tool_group, extrusion);
        return extrusion_target(context, tool);
    }
    return filament_target(print.config(), tool, first_layer);
}

// Update one heater's last target and known reached temperature after a waited or non-waited request.
void apply_tracked_request(TrackedHeater &state,
                           const int16_t temperature,
                           const bool wait)
{
    state.target = temperature;
    if (wait)
        state.known = temperature;
    else if (state.known.has_value() && temperature < *state.known)
        state.known = temperature;
}

// Return the state slot for a tool, sharing slot zero for SEMM; the map owns created state and the reference remains valid until that entry is erased or the map is cleared.
TrackedHeater &tracked_tool(std::map<extruder_index_t, TrackedHeater> &tools,
                            const extruder_index_t tool,
                            const bool semm)
{
    return tools[semm ? extruder_index_t(0) : tool];
}

// Allocate one owned heater event, configure its target, and append it to the requested scope phase; null storage or failed append throws.
void append_heater_event(storage_handle *storage,
                         PrintingScopeEvents events,
                         const HeaterRequest &request,
                         const bool before)
{
    if (storage == nullptr)
        throw std::invalid_argument("A temperature event needs plugin storage.");
    StoredExtrusionEntity event(storage);
    EPropertyHeater &heater = event.get_or_add(EPropertyHeater::key);
    if (request.heater == HeaterKind::Tool)
        heater.set_tool(request.temperature, request.tool, request.wait);
    else if (request.heater == HeaterKind::Bed)
        heater.set_bed(request.temperature, request.wait);
    else
        heater.set_chamber(request.temperature, request.wait);
    MutableExtrusionEntity appended = before ?
        events.append_before_move(event.mutable_view()) :
        events.append_after_move(event.mutable_view());
    if (!appended.valid())
        throw std::runtime_error("Unable to append a temperature event.");
}

// Return the process-local startup plugin singleton associated with the supplied orchestrator handle.
StartupTemperature &StartupTemperature::instance(orchestrator_handle *orchestrator)
{
    static StartupTemperature plugin(orchestrator);
    return plugin;
}

// Construct the startup plugin facade without taking ownership of the orchestrator handle.
StartupTemperature::StartupTemperature(orchestrator_handle *orchestrator) :
    PluginBase(orchestrator)
{
}

// Return the stable identifier used to select the startup temperature plugin.
const char *StartupTemperature::id_impl() const noexcept
{
    return k_startup_plugin_id;
}

// Return the human-readable startup temperature plugin name.
const char *StartupTemperature::name_impl() const noexcept
{
    return "Default startup temperatures";
}

// Return the description shown for startup heater planning.
const char *StartupTemperature::description_impl() const noexcept
{
    return "Requests bed, chamber, and filament temperatures after the start G-code.";
}

// Return the exclusive selection group for plan-level startup temperature providers.
const char *StartupTemperature::exclusive_group_impl() const noexcept
{
    return k_startup_group_id;
}

// Return the label displayed for the startup temperature selection group.
const char *StartupTemperature::exclusive_group_label_impl() const noexcept
{
    return "Startup temperatures";
}

// Return the tooltip explaining when startup heater events are appended.
const char *StartupTemperature::exclusive_group_tooltip_impl() const noexcept
{
    return "Selects how automatic startup heater requests are added after start G-code.";
}

// Select the plan-level extrusion edit step for startup heater event creation.
slicing_step_t StartupTemperature::step_impl() const noexcept
{
    return STEP_EXTRUSION_EDIT;
}

// Return the empty dependency list because startup temperature reads no other plugin output.
const char *const *StartupTemperature::dependencies_impl() const noexcept
{
    return k_no_dependencies;
}

// Return priority ten so startup heater events follow start-script events.
int32_t StartupTemperature::priority_impl() const noexcept
{
    return 10;
}

// Copy the startup options read by planning into the caller's metadata buffer; a null buffer performs only the count query.
int32_t StartupTemperature::used_config_keys(raw_used_config_key *keys) const noexcept
{
    static const raw_used_config_key used_keys[] = {
        {"autoemit_temperature_commands", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"chamber_temperature", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"first_layer_bed_temperature", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"first_layer_temperature", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"idle_temperature", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"ooze_prevention", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"print_first_layer_bed_temperature", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"single_extruder_multi_material", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"standby_temperature_delta", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"start_gcode_manual", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"temperature", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
    };
    if (keys != nullptr)
        std::copy(std::begin(used_keys), std::end(used_keys), keys);
    return int32_t(std::size(used_keys));
}

// Report that startup temperature defines no user configuration options.
int32_t StartupTemperature::defined_config_keys(const char **) const noexcept
{
    return 0;
}

// Initialize startup temperature without allocating options or retaining plugin storage.
void StartupTemperature::inilialize_impl(storage_handle *) const
{
}

// Return the progress format used while startup heater events are appended.
const char *StartupTemperature::progress_message_format_impl() const noexcept
{
    return "Adding startup temperatures: %u / %u requests";
}

// Compute startup requests from the plan and append owned events after start_gcode; invalid context is ignored and append/storage failures propagate.
void StartupTemperature::run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_extrusion_edition *ctx = plugin_ctx_as_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->print == nullptr || ctx->plan == nullptr ||
        run_ctx->plugin_storage == nullptr)
        return;
    const Print print(ctx->print);
    PrintingPlan plan(ctx->plan);
    const StartupHeating heating = startup_heating(print.config(), plan);
    // A startup request is materialized only when at least one positive heater operation was planned.
    if (!heating.bed && !heating.chamber && !heating.initial_tool_temperature &&
        heating.other_tools.empty())
        return;
    PrintingScopeEvents events = plan.events();
    if (heating.bed)
        append_heater_event(run_ctx->plugin_storage, events,
                            HeaterRequest{HeaterKind::Bed, EXTRUDER_ID_UNKNOWN, *heating.bed, true}, true);
    if (heating.chamber)
        append_heater_event(run_ctx->plugin_storage, events,
                            HeaterRequest{HeaterKind::Chamber, EXTRUDER_ID_UNKNOWN, *heating.chamber, true}, true);
    if (heating.initial_tool_temperature)
        append_heater_event(run_ctx->plugin_storage, events,
                            HeaterRequest{HeaterKind::Tool, heating.initial_tool,
                                          *heating.initial_tool_temperature, true}, true);
    for (const StartupToolTarget &target : heating.other_tools)
        append_heater_event(run_ctx->plugin_storage, events,
                            HeaterRequest{HeaterKind::Tool, target.tool, target.temperature, false}, true);
}

// Return the process-local layer plugin singleton associated with the supplied orchestrator handle.
LayerTemperature &LayerTemperature::instance(orchestrator_handle *orchestrator)
{
    static LayerTemperature plugin(orchestrator);
    return plugin;
}

// Construct the layer plugin facade without owning the orchestrator handle.
LayerTemperature::LayerTemperature(orchestrator_handle *orchestrator) :
    PluginBase(orchestrator)
{
}

// Return the stable identifier used to select the layer temperature plugin.
const char *LayerTemperature::id_impl() const noexcept
{
    return "layer_extrusion_edit.temperature.default";
}

// Return the human-readable layer temperature plugin name.
const char *LayerTemperature::name_impl() const noexcept
{
    return "Default layer temperatures";
}

// Return the description of sequential heater tracking and event placement.
const char *LayerTemperature::description_impl() const noexcept
{
    return "Tracks nozzle and bed targets through tool visits and extrusion roots.";
}

// Return the exclusive selection group for layer temperature providers.
const char *LayerTemperature::exclusive_group_impl() const noexcept
{
    return "layer_extrusion_edit.temperature";
}

// Return the label displayed for the layer temperature selection group.
const char *LayerTemperature::exclusive_group_label_impl() const noexcept
{
    return "Layer temperatures";
}

// Return the tooltip explaining layer and tool-change heater requests.
const char *LayerTemperature::exclusive_group_tooltip_impl() const noexcept
{
    return "Selects automatic nozzle, tool-change, second-layer, and bed requests.";
}

// Select the layer-level extrusion edit step for sequential heater decisions.
slicing_step_t LayerTemperature::step_impl() const noexcept
{
    return STEP_LAYER_EXTRUSION_EDIT;
}

// Return the empty dependency list because the temperature cache reads only plan and configuration data.
const char *const *LayerTemperature::dependencies_impl() const noexcept
{
    return k_no_dependencies;
}

// Return priority thirty so layer temperature runs after fan and tool-change producers.
int32_t LayerTemperature::priority_impl() const noexcept
{
    return 30;
}

// Copy every configuration option read by layer temperature planning; a null buffer performs only the count query.
int32_t LayerTemperature::used_config_keys(raw_used_config_key *keys) const noexcept
{
    static const raw_used_config_key used_keys[] = {
        {"autoemit_temperature_commands", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"bed_temperature", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"chamber_temperature", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"first_layer_bed_temperature", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"first_layer_temperature", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"idle_temperature", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"ooze_prevention", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"print_bed_temperature", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"print_first_layer_bed_temperature", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"print_first_layer_temperature", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"print_temperature", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"single_extruder_multi_material", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"standby_temperature_delta", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"start_gcode_manual", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"temperature", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
    };
    if (keys != nullptr)
        std::copy(std::begin(used_keys), std::end(used_keys), keys);
    return int32_t(std::size(used_keys));
}

// Report that layer temperature defines no user configuration options.
int32_t LayerTemperature::defined_config_keys(const char **) const noexcept
{
    return 0;
}

// Initialize layer temperature without allocating options or retaining plugin storage.
void LayerTemperature::inilialize_impl(storage_handle *) const
{
}

// Return the progress format used while cached layer decisions are materialized.
const char *LayerTemperature::progress_message_format_impl() const noexcept
{
    return "Resolving extrusion temperatures: %u / %u layers";
}

// Read the borrowed print and plan in print order, reset and fill the mutable
// per-layer decision cache for later parallel runs, and leave plan data
// unchanged; a null context is ignored, while a mismatched run count or
// planning failure is reported to the host.
void LayerTemperature::setup_impl(const plugin_run_context *run_ctx, const uint32_t run_count) const
{
    throw_if_cancelled(run_ctx);
    m_setup_valid = false;
    m_decisions.clear();
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->print == nullptr || ctx->plan == nullptr)
        return;

    const Print print(ctx->print);
    PrintingPlan plan(ctx->plan);
    const Config config = print.config();
    const StartupHeating startup = startup_heating(config, plan);
    const bool semm = config.bool_or_default("single_extruder_multi_material", false);
    const bool ooze_prevention = config.bool_or_default("ooze_prevention", false) && !semm;
    std::map<extruder_index_t, TrackedHeater> tools;
    TrackedHeater bed;
    // Preserve only the selected default startup provider's known reached temperature; targets and other heaters stay unknown.
    const orchestrator_plugin_handle *default_startup_plugin = orchestrator_find_plugin(
        m_orchestrator, k_startup_plugin_id);
    const bool startup_plugin_selected = default_startup_plugin != nullptr &&
        orchestrator_select_plugin(
            m_orchestrator,
            config.handle(),
            STEP_EXTRUSION_EDIT,
            k_startup_group_id) == default_startup_plugin;
    if (startup_plugin_selected && startup.initial_tool_temperature)
        tracked_tool(tools, startup.initial_tool, semm).known = *startup.initial_tool_temperature;

    m_decisions.resize(plan.group_count());
    uint32_t total_layers = 0;
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        PrintingGroup group = plan.group(group_idx);
        m_decisions[group_idx].resize(group.layer_group_count());
        total_layers += group.layer_group_count();
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx)
            m_decisions[group_idx][layer_idx].visits.resize(
                group.layer_group(layer_idx).tool_group_count());
    }
    if (total_layers != run_count)
        throw std::runtime_error("Temperature run count does not match the PrintingPlan.");

    PreviousVisit previous;
    uint32_t plan_layer_idx = 0;
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        throw_if_cancelled(run_ctx);
        PrintingGroup group = plan.group(group_idx);
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx, ++plan_layer_idx) {
            throw_if_cancelled(run_ctx);
            PrintingLayerGroup layer = group.layer_group(layer_idx);
            LayerTemperatureDecision &layer_decision = m_decisions[group_idx][layer_idx];
            const bool first_layer = layer_idx == 0;
            const extruder_index_t layer_start_tool = previous.valid ? previous.tool : EXTRUDER_ID_UNKNOWN;

            if (plan_layer_idx == 1) {
                // At the second plan layer, request normal temperatures for idle physical tools.
                if (!semm && !ooze_prevention) {
                    for (const extruder_index_t tool : startup.used_tools) {
                        if (tool == layer_start_tool)
                            continue;
                        const int32_t target = config.vector_int_or_default("temperature", tool, -1);
                        const std::optional<int16_t> value = target > 0 ? heater_temperature(target) : std::nullopt;
                        if (!value)
                            continue;
                        TrackedHeater &state = tracked_tool(tools, tool, semm);
                        if (!state.target || *state.target != *value) {
                            layer_decision.before.push_back(
                                HeaterRequest{HeaterKind::Tool, tool, *value, false});
                            apply_tracked_request(state, *value, false);
                        }
                    }
                }
                // The same second-layer pass also requests the configured bed target once.
                int32_t bed_target = config.int_or_default("print_bed_temperature", -1);
                if (bed_target < 0) {
                    bed_target = 0;
                    for (const extruder_index_t tool : startup.used_tools)
                        bed_target = std::max(
                            bed_target,
                            config.vector_int_or_default("bed_temperature", tool, 0));
                }
                const std::optional<int16_t> value = heater_temperature(bed_target);
                if (value && *value > 0 && (!bed.target || *bed.target != *value)) {
                    layer_decision.before.push_back(
                        HeaterRequest{HeaterKind::Bed, EXTRUDER_ID_UNKNOWN, *value, false});
                    apply_tracked_request(bed, *value, false);
                }
            }

            for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
                // Process each visit in order so standby, tool changes, and root targets share one history.
                throw_if_cancelled(run_ctx);
                PrintingToolGroup tool_group = layer.tool_group(tool_idx);
                const extruder_index_t tool = tool_group.extruder_id();
                if (tool == EXTRUDER_ID_UNKNOWN)
                    throw std::runtime_error("Temperature planning found an unresolved tool visit.");
                VisitTemperatureDecision &visit = layer_decision.visits[tool_idx];
                const bool tool_change = previous.valid && previous.tool != tool;
                const bool tower_change = semm && tool_change && tower_visit(tool_group);

                if (tool_change && !tower_change) {
                    // Put standby at the end of the outgoing visit, before the incoming TOOLCHANGE event.
                    const std::optional<int16_t> base = filament_target(
                        config, previous.tool, previous.first_layer);
                    if (ooze_prevention) {
                        const std::optional<int16_t> standby = standby_target(config, previous.tool, base);
                        if (standby) {
                            TrackedHeater &leaving_state = tracked_tool(tools, previous.tool, semm);
                            if (!leaving_state.target || *leaving_state.target != *standby) {
                                m_decisions[previous.group_idx][previous.layer_idx]
                                    .visits[previous.tool_idx].after.push_back(
                                        HeaterRequest{HeaterKind::Tool, previous.tool, *standby, false});
                                apply_tracked_request(leaving_state, *standby, false);
                            }
                        }
                    }

                    // Non-tower changes request the incoming target and wait if its reached temperature is unknown or too low.
                    const std::optional<int16_t> target = visit_target(
                        print, plan, tool_group, first_layer);
                    if (target && *target > 0) {
                        TrackedHeater &state = tracked_tool(tools, tool, semm);
                        const bool wait = !state.known ||
                            int32_t(*target) >= int32_t(*state.known) +
                            k_wait_temperature_margin_celsius;
                        if (wait || !state.target || *state.target != *target) {
                            visit.before.push_back(
                                HeaterRequest{HeaterKind::Tool, tool, *target, wait});
                            apply_tracked_request(state, *target, wait);
                        }
                    }
                }

                if (tower_change)
                    // The tower owns the SEMM change; ordinary roots start with an unknown shared heater state.
                    tracked_tool(tools, tool, semm) = TrackedHeater{};

                // Without a regional source, retain the group-layer fallback. Synthetic roots do not replace the last source-layer status.
                bool last_source_first_layer = first_layer;
                // Synthetic roots and non-regional roots stay untouched because settings_context needs a region.
                for (uint32_t extrusion_idx = 0;
                     extrusion_idx < tool_group.extrusion_count(); ++extrusion_idx) {
                    PrintingExtrusion extrusion = tool_group.extrusion(extrusion_idx);
                    if (!extrusion.region_island().valid() ||
                        ProcessParameterHelpers::is_regionless_synthetic(plan, extrusion))
                        continue;
                    ExtrusionSettingsContext context = ProcessParameterHelpers::settings_context(
                        print, tool_group, extrusion);
                    last_source_first_layer = context.first_layer;
                    const std::optional<int16_t> target = extrusion_target(context, tool);
                    if (!target || *target <= 0)
                        continue;
                    TrackedHeater &state = tracked_tool(tools, tool, semm);
                    if (!state.target || *state.target != *target) {
                        visit.roots.emplace_back(extrusion_idx, *target);
                        apply_tracked_request(state, *target, false);
                    }
                }

                previous = PreviousVisit{true, group_idx, layer_idx, tool_idx, tool, last_source_first_layer};
            }
        }
    }
    m_setup_valid = true;
}

// Add one progress unit for a valid cached layer run without mutating another layer's decisions.
void LayerTemperature::setup_run_impl(const plugin_run_context *run_ctx) const
{
    if (!m_setup_valid)
        return;
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx != nullptr && ctx->layer_group != nullptr)
        progress().add_max(1);
}

// Materialize only the cached requests for the current layer into owned event trees and roots; invalid indices or append failures propagate.
void LayerTemperature::run_impl(const plugin_run_context *run_ctx) const
{
    if (!m_setup_valid)
        return;
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->layer_group == nullptr || run_ctx->plugin_storage == nullptr)
        return;
    if (ctx->group_idx >= m_decisions.size() ||
        ctx->layer_group_idx >= m_decisions[ctx->group_idx].size())
        throw std::runtime_error("Temperature run references an unknown layer.");

    PrintingLayerGroup layer(ctx->layer_group);
    const LayerTemperatureDecision &decision = m_decisions[ctx->group_idx][ctx->layer_group_idx];
    for (const HeaterRequest &request : decision.before)
        append_heater_event(run_ctx->plugin_storage, layer.events(), request, true);
    for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
        PrintingToolGroup tool_group = layer.tool_group(tool_idx);
        const VisitTemperatureDecision &visit = decision.visits.at(tool_idx);
        for (const HeaterRequest &request : visit.after)
            append_heater_event(run_ctx->plugin_storage, tool_group.events(), request, false);
        for (const HeaterRequest &request : visit.before)
            append_heater_event(run_ctx->plugin_storage, tool_group.events(), request, true);
        for (const std::pair<uint32_t, int16_t> &root : visit.roots) {
            // Attach a non-waiting active-tool request to the existing root; the plan keeps root ownership.
            tool_group.extrusion(root.first).mutable_root().get_or_add(EPropertyHeater::key).set_tool(
                root.second, EXTRUDER_ID_UNKNOWN, false);
        }
    }
    progress().increment();
}

} // namespace

// Register the plan-level startup and layer-level temperature providers in the supplied orchestrator without taking ownership.
void register_default_temperature_plugins(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(
        orchestrator, StartupTemperature::instance(orchestrator).c_instance());
    orchestrator_register_plugin(
        orchestrator, LayerTemperature::instance(orchestrator).c_instance());
}

}}} // namespace slic3r_api::LayerExtrusionEdit::DefaultTemperaturePlugin
