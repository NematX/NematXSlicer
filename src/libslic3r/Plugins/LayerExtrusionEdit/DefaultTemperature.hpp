///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_Plugins_LayerExtrusionEdit_DefaultTemperature_hpp_
#define slic3r_Plugins_LayerExtrusionEdit_DefaultTemperature_hpp_

/*
Default temperature plugins
============================

This module registers the plan-level startup-temperature producer and the
layer-level extrusion-temperature producer.  The first appends heater events
after start_gcode; the second records sequential temperature decisions and
materializes only the decisions belonging to each layer run.  Its initial
heater state is known only when the default startup provider is the selected
active provider for its exclusive group.

LayerTemperature is also the base of alternative providers for the
layer_extrusion_edit.temperature exclusive group.  A derived provider keeps the
step and priority 30, which all providers of one exclusive group must share,
and gives its own identifier.  Its setup_impl() first calls
LayerTemperature::setup_impl(), which fills m_decisions in print order and sets
m_setup_valid, then edits those decisions before any layer run starts; the
inherited run_impl() appends each layer's decisions to the plan.
extrusion_target() resolves temperatures with the same rules as the base, and
TemperatureTargets::filament_target() reads the filament temperatures.  A typical derived provider:

    class MyTemperature final : public LayerTemperature
    {
        void setup_impl(const plugin_run_context *run_ctx, uint32_t run_count) const override
        {
            LayerTemperature::setup_impl(run_ctx, run_count);
            if (m_setup_valid)
                edit_decisions(m_decisions);
        }
    };
*/

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "ExtrusionProcessParameterHelpers.hpp"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace DefaultTemperaturePlugin {

// Heater addressed by one planned request.
enum class HeaterKind
{
    Tool,
    Bed,
    Chamber
};

// One planned heater operation, appended to the plan as an EPropertyHeater event.
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

// Planned temperature edits of one tool visit.
struct VisitTemperatureDecision
{
    // Requests appended before this visit, normally the incoming tool target.
    std::vector<HeaterRequest> before;
    // Requests appended after this visit, normally standby for its leaving tool.
    std::vector<HeaterRequest> after;
    // Root index and target for non-synthetic regional roots in visit order.
    std::vector<std::pair<uint32_t, int16_t>> roots;
};

// Planned temperature edits of one layer.
struct LayerTemperatureDecision
{
    // Requests appended after the layer's existing before events.
    std::vector<HeaterRequest> before;
    // Per-tool-visit requests and root edits for this layer.
    std::vector<VisitTemperatureDecision> visits;
};

// Decisions indexed by plan group, then by layer within that group.
using LayerDecisions = std::vector<std::vector<LayerTemperatureDecision>>;

// Plans heater requests in print order during setup, then appends each layer's share during its run.
class LayerTemperature : public PluginBase
{
public:
    // Return the singleton layer provider registered in the current orchestrator.
    static LayerTemperature &instance(orchestrator_handle *orchestrator);

    // Initialize the generic plugin facade without owning plan or print data.
    explicit LayerTemperature(orchestrator_handle *orchestrator);

protected:
    // Copy every configuration option read by layer temperature planning.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    // Record all sequential heater decisions before parallel layer runs begin.
    void setup_impl(const plugin_run_context *run_ctx, uint32_t run_count) const override;
    // Apply only the cached decisions for the current layer.
    void run_impl(const plugin_run_context *run_ctx) const override;

    // True when setup completed and cached decisions match the current plan.
    mutable bool m_setup_valid = false;
    // Per-group and per-layer decisions materialized by parallel layer runs.
    mutable LayerDecisions m_decisions;

private:
    // Return the stable layer provider identifier.
    const char *id_impl() const noexcept override;
    // Return the layer provider display name.
    const char *name_impl() const noexcept override;
    // Describe sequential target tracking and layer event placement.
    const char *description_impl() const noexcept override;
    // Select the exclusive layer-temperature group.
    const char *exclusive_group_impl() const noexcept override;
    // Run in the layer extrusion edit step.
    slicing_step_t step_impl() const noexcept override;
    // Declare that layer temperature has no plugin dependency.
    const char *const *dependencies_impl() const noexcept override;
    // Run after the default fan provider and tool-change producer.
    int32_t priority_impl() const noexcept override;
    // Layer temperature does not define a new user setting.
    int32_t defined_config_keys(const char **keys) const noexcept override;
    // Initialize without creating dynamic options.
    void inilialize_impl(storage_handle *) const override;
    // Describe layer-level temperature progress.
    const char *progress_message_format_impl() const noexcept override;
    // Count the current layer as one materialization unit.
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
};

// Read regional overrides before falling back to filament temperatures.
std::optional<int16_t> extrusion_target(const ProcessParameterHelpers::ExtrusionSettingsContext &context,
                                        extruder_index_t tool);

/* Register the startup and layer heater providers in the supplied orchestrator. */
void register_default_temperature_plugins(orchestrator_handle *orchestrator);

}}} // namespace slic3r_api::LayerExtrusionEdit::DefaultTemperaturePlugin

#endif // slic3r_Plugins_LayerExtrusionEdit_DefaultTemperature_hpp_
