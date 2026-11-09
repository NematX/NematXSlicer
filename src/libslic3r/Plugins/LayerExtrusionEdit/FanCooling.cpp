///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#include "FanCooling.hpp"

#include <algorithm>
#include <iterator>
#include <stdexcept>
#include <vector>

#include "FanTreeHelpers.hpp"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingLayerTimeProperty.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanTimeEstimator.hpp"

/*
This composable layer editor raises eligible fan requests for short layers.
Serialized setup copies valid RESOLVED timing or estimates all missing durations
in one ordered pass. Layer callbacks use only their cached duration and borrowed
tool trees; they preserve protected modes and publish TIME references.

    setup_impl(): validate timing and cache one total per layer
    setup_run_impl(): count extrusion trees for progress
    run_impl(): validate BASE, transform eligible leaves, publish TIME
        `-- FanTreeHelpers::for_each_leaf(): preserve geometry and service events

The plugin owns the timing cache and property keys. Layer runs edit disjoint
trees; cancellation and invalid timing/context errors reach the plugin bridge.
No motion speed changes here, and every tool visit shares the whole-layer time.
*/

namespace slic3r_api { namespace LayerExtrusionEdit { namespace FanCoolingPlugin {
namespace {
const char *const k_no_dependencies[] = {nullptr};
const raw_used_config_key k_used[] = {
    {"fan_below_layer_time", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"max_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"slowdown_below_layer_time", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"travel_speed", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
};
const char *const k_defined[] = {"fan_below_layer_time", "max_fan_speed", "slowdown_below_layer_time"};

// Keep the existing filament option metadata under a single active owner.
void initialize_settings(orchestrator_handle *orch);

class FanCooling : public PluginBase {
public:
    // Retain host-local property keys; the host outlives this registered provider.
    explicit FanCooling(orchestrator_handle *orch) : PluginBase(orch),
        m_time(printing_layer_time_property_key(orch)), m_context(FanPolicy::context_property_key(orch)) {}
private:
    // Return this independent policy's stable identity.
    const char *id_impl() const noexcept override { return "layer_extrusion_edit.fan.cooling"; }
    // Return borrowed display metadata.
    const char *name_impl() const noexcept override { return "Short-layer fan cooling"; }
    // These stages compose; only the standard reference provider is exclusive.
    const char *exclusive_group_impl() const noexcept override { return ""; }
    // Describe the transformation without claiming to slow down motion.
    const char *description_impl() const noexcept override { return "Increase modifiable fan requests from total layer duration."; }
    // Run after ordering on mutable plan clones.
    slicing_step_t step_impl() const noexcept override { return STEP_LAYER_EXTRUSION_EDIT; }
    // Consume the public phase contract without requiring a particular producer ID.
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    // Complete TIME before first-layer policies start.
    int32_t priority_impl() const noexcept override { return LAYER_EXTRUSION_EDIT_PRIORITY_FAN_COOLING; }
    // Copy borrowed input declarations, or return their count for a null output.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override {
        if (keys) std::copy(std::begin(k_used), std::end(k_used), keys);
        return int32_t(std::size(k_used));
    }
    // Copy the option names owned by this stage, or return their count.
    int32_t defined_config_keys(const char **keys) const noexcept override {
        if (keys) std::copy(std::begin(k_defined), std::end(k_defined), keys);
        return int32_t(std::size(k_defined));
    }
    // Publish option definitions before configuration freezes; errors propagate.
    void inilialize_impl(storage_handle *) const override { initialize_settings(m_orchestrator); }
    // Return the borrowed per-tree progress format.
    const char *progress_message_format_impl() const noexcept override { return "Applying short-layer fan cooling: %u / %u trees"; }
    // Rebuild the owned timing cache in the host's serialized setup phase.
    void setup_impl(const plugin_run_context *run, uint32_t) const override;
    // Count this layer's work without touching its trees.
    void setup_run_impl(const plugin_run_context *run) const override;
    // Transform eligible requests and publish TIME on each borrowed tool group.
    void run_impl(const plugin_run_context *run) const override;
    PluginPropertyKey<PrintingLayerTimeProperty> m_time;
    PluginPropertyKey<FanPolicy::Context> m_context;
    mutable bool m_valid = false;
    mutable std::vector<std::vector<PrintingLayerTimeProperty>> m_times;
};

void initialize_settings(orchestrator_handle *orch)
{
    raw_config_option_def def = raw_config_option_def_init();
    def.opt_key = "fan_below_layer_time";
    def.type = RAW_CO_VECTOR_FLOAT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_LAYER_EXTRUSION_EDIT;
    def.category = RAW_OPTION_CATEGORY_COOLING;
    def.label = "Enable fan if layer print time is below";
    def.tooltip = "If layer print time is estimated below this number of seconds, fan will be enabled and its speed will be calculated by interpolating the default and maximum speeds.\nSet zero to disable.";
    def.sidetext = "approximate seconds";
    def.is_vector_extruder = 1;
    def.has_min = def.has_max = 1;
    def.min_value = 0;
    def.max_value = 1000;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "60";
    if (orchestrator_create_option_def(orch, &def) != OPTION_DEF_ERROR_OK)
        throw std::runtime_error("Cannot register fan_below_layer_time.");

    def = raw_config_option_def_init();
    def.opt_key = "max_fan_speed";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_LAYER_EXTRUSION_EDIT;
    def.category = RAW_OPTION_CATEGORY_COOLING;
    def.label = "Max";
    def.full_label = "Max fan speed";
    def.tooltip = "This setting represents the maximum speed of your fan, used when the layer print time is Very short.";
    def.sidetext = "%";
    def.is_vector_extruder = 1;
    def.has_min = def.has_max = 1;
    def.min_value = 0;
    def.max_value = 100;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "100";
    if (orchestrator_create_option_def(orch, &def) != OPTION_DEF_ERROR_OK)
        throw std::runtime_error("Cannot register max_fan_speed.");

    def = raw_config_option_def_init();
    def.opt_key = "slowdown_below_layer_time";
    def.type = RAW_CO_VECTOR_FLOAT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_LAYER_EXTRUSION_EDIT;
    def.category = RAW_OPTION_CATEGORY_COOLING;
    def.label = "Slow down if layer print time is below";
    def.tooltip = "If layer print time is estimated below this number of seconds, print moves speed will be scaled down to extend duration to this value, if possible.\nSet zero to disable.";
    def.sidetext = "approximate seconds";
    def.is_vector_extruder = 1;
    def.has_min = def.has_max = 1;
    def.min_value = 0;
    def.max_value = 1000;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "5";
    if (orchestrator_create_option_def(orch, &def) != OPTION_DEF_ERROR_OK)
        throw std::runtime_error("Cannot register slowdown_below_layer_time.");
}

void FanCooling::setup_impl(const plugin_run_context *run, uint32_t) const
{
    throw_if_cancelled(run);
    m_valid = false;
    m_times.clear();
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run);
    if (!ctx || !ctx->print || !ctx->plan) return;
    const Print print(ctx->print);
    const PrintingPlan plan(ctx->plan);
    m_times.resize(plan.group_count());
    bool needs_estimate = false;
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        const ConstPrintingGroup group = plan.group(group_idx);
        m_times[group_idx].resize(group.layer_group_count());
        for (uint32_t idx = 0; idx < group.layer_group_count(); ++idx) {
            throw_if_cancelled(run);
            const PrintingLayerTimeProperty *time = m_time.get(group.layer_group(idx).properties());
            if (time) time->validate();
            if (time && time->is_resolved()) m_times[group_idx][idx] = *time;
            else needs_estimate = true;
        }
    }
    if (needs_estimate) {
        const std::vector<PrintingLayerTimeEstimate> estimates = PrintingPlanTimeEstimator(print.config()).estimate(plan);
        for (const PrintingLayerTimeEstimate &estimate : estimates) {
            throw_if_cancelled(run);
            const PrintingLayerTimeProperty *time = m_time.get(
                plan.group(estimate.group_idx).layer_group(estimate.layer_group_idx).properties());
            if (!time || !time->is_resolved()) {
                PrintingLayerTimeProperty &cached = m_times.at(estimate.group_idx).at(estimate.layer_group_idx);
                cached.duration_seconds = estimate.duration_seconds;
                cached.origin = RAW_PRINTING_LAYER_TIME_ORIGIN_ESTIMATED;
                cached.validate();
            }
        }
    }
    m_valid = true;
}

void FanCooling::setup_run_impl(const plugin_run_context *run) const
{
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run);
    if (m_valid && ctx && ctx->layer_group)
        progress().add_max(ProcessParameterHelpers::extrusion_tree_count(PrintingLayerGroup(ctx->layer_group)));
}

void FanCooling::run_impl(const plugin_run_context *run) const
{
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run);
    if (!m_valid || !ctx || !ctx->print || !ctx->layer_group) return;
    throw_if_cancelled(run);
    const Config config = Print(ctx->print).config();
    PrintingLayerGroup layer(ctx->layer_group);
    const PrintingLayerTimeProperty &time = m_times.at(ctx->group_idx).at(ctx->layer_group_idx);
    m_time.get_or_add(layer.properties()) = time;
    for (uint32_t idx = 0; idx < layer.tool_group_count(); ++idx) {
        throw_if_cancelled(run);
        PrintingToolGroup tool = layer.tool_group(idx);
        const FanPolicy::Context base = FanTreeHelpers::read_context(tool, m_context, RAW_FAN_POLICY_PHASE_BASE);
        const FanPolicy::Settings settings = FanPolicy::read_layer_time_settings(config, tool.extruder_id());
        const FanPolicy::Context timed = FanPolicy::apply_layer_time(base, settings, time.duration_seconds);
        for (uint32_t extrusion = 0; extrusion < tool.extrusion_count(); ++extrusion) {
            FanTreeHelpers::for_each_leaf(tool.extrusion(extrusion).mutable_root(),
                [&](MutableExtrusionEntity leaf, const ProcessParameterHelpers::EffectiveTreeState &state) {
                    if (!state.fan.can_modify(FAN_REQUEST_MODE_AUTOMATIC)) return;
                    // Missing overhang requests inherit their perimeter family
                    // only after first-layer closure; an upstream Automatic is
                    // already a real request and follows its own role rules.
                    if (!state.fan.is_set() && FanPolicy::fan_role(state.attributes.extrusion_role()) == RAW_FAN_ROLE_OVERHANG_PERIMETER)
                        return;
                    const float percent = FanPolicy::layer_time_percent(state.fan.is_set() ? state.fan.percent : -1.f,
                        FanPolicy::fan_role(state.attributes.extrusion_role()), settings, time.duration_seconds);
                    if (percent >= 0.f)
                        leaf.get_or_add(EPropertyFan::key) = FanPolicy::transform_request(state.fan, percent, FAN_REQUEST_MODE_AUTOMATIC);
                }, run);
            progress().increment();
        }
        m_context.get_or_add(tool.properties()) = timed;
    }
}
} // namespace

void register_fan_cooling_plugin(orchestrator_handle *orch)
{
    static FanCooling plugin(orch);
    orchestrator_register_plugin(orch, plugin.c_instance());
}
}}} // namespace slic3r_api::LayerExtrusionEdit::FanCoolingPlugin
