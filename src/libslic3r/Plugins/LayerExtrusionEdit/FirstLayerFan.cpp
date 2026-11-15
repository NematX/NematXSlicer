///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#include "FirstLayerFan.hpp"

#include <algorithm>
#include <iterator>
#include <stdexcept>

#include "FanTreeHelpers.hpp"
#include "libslic3r/Plugins/Common/SourceLayerIndices.hpp"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

/*
This layer editor applies the disabled-layer/full-speed-layer program to eligible
fan requests and closes role fallbacks. In Api/plugin/cpp/FanPolicy.hpp, BASE is
the initial reference table, TIME applies layer-duration limits, and CLOSED adds
first-layer limits and final fallbacks. This editor accepts TIME, or BASE when
optional cooling is absent. Protected requests are those whose mode forbids this
policy from modifying them; their speed and mode pair remains unchanged.
First-layer limits use cached source ranks: each copy merges its object and local
auxiliary heights, and a shared pass takes the minimum rank of its actual sources.

    setup_impl(): cache source ranks before parallel runs
    run_impl(): validate the preceding public context
    |-- close_first_layers(): compute CLOSED references and dynamic bounds
    `-- for_each_leaf(): apply the program once or fill a late absence

The provider owns host-local property keys, not geometry. Each callback borrows
only its layer's tool groups; no views survive the run. Registration errors,
cancellation and invalid contexts propagate through the normal plugin bridge.
*/

namespace slic3r_api { namespace LayerExtrusionEdit { namespace FirstLayerFanPlugin {
namespace {
const char *const k_no_dependencies[] = {nullptr};
const raw_used_config_key k_used[] = {
    {"disable_fan_first_layers", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"full_fan_speed_layer", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
};
const char *const k_defined[] = {"disable_fan_first_layers", "full_fan_speed_layer"};

// Preserve the existing filament metadata while assigning one owner per key.
void initialize_settings(orchestrator_handle *orch);

class FirstLayerFan : public PluginBase {
public:
    // Bind the public context key to the host that owns this provider.
    explicit FirstLayerFan(orchestrator_handle *orch) : PluginBase(orch), m_context(FanPolicy::context_property_key(orch)) {}
private:
    // Return the stable first-layer policy identity.
    const char *id_impl() const noexcept override { return "layer_extrusion_edit.fan.first_layers"; }
    // Return borrowed display metadata.
    const char *name_impl() const noexcept override { return "First-layer fan program"; }
    // The applicator is not a competing standard reference provider.
    const char *exclusive_group_impl() const noexcept override { return ""; }
    // Describe the request transformation and context closure.
    const char *description_impl() const noexcept override { return "Apply first-layer fan limits and close role fallbacks."; }
    // Edit only the ordered plan's mutable layer clones.
    slicing_step_t step_impl() const noexcept override { return STEP_LAYER_EXTRUSION_EDIT; }
    // Depend on a public phase, not a producer's identity.
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    // Close references before dynamic geometry processing.
    int32_t priority_impl() const noexcept override { return LAYER_EXTRUSION_EDIT_PRIORITY_FAN_FIRST_LAYERS; }
    // Copy input declarations, or return their count for null output.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override {
        if (keys) std::copy(std::begin(k_used), std::end(k_used), keys);
        return int32_t(std::size(k_used));
    }
    // Copy option names owned by this stage, or return their count.
    int32_t defined_config_keys(const char **keys) const noexcept override {
        if (keys) std::copy(std::begin(k_defined), std::end(k_defined), keys);
        return int32_t(std::size(k_defined));
    }
    // Register the owned option definitions before configuration freezes.
    void inilialize_impl(storage_handle *) const override { initialize_settings(m_orchestrator); }
    // Return the borrowed per-tree progress format.
    const char *progress_message_format_impl() const noexcept override { return "Applying first-layer fan program: %u / %u trees"; }
    // Count trees in the borrowed layer without editing them.
    void setup_run_impl(const plugin_run_context *run) const override;
    // Cache owned source-layer ranks once before parallel fan runs.
    void setup_impl(const plugin_run_context *run, uint32_t) const override {
        m_indices.clear();
        const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run);
        if (ctx && ctx->print && ctx->plan)
            m_indices = source_layer_indices(Print(ctx->print), ConstPrintingPlan(ctx->plan));
    }
    // Transform modifiable requests using cached source ranks and publish CLOSED context.
    void run_impl(const plugin_run_context *run) const override;
    PluginPropertyKey<FanPolicy::Context> m_context;
    mutable SourceLayerIndices m_indices;
};

void initialize_settings(orchestrator_handle *orch)
{
    raw_config_option_def def = raw_config_option_def_init();
    def.opt_key = "disable_fan_first_layers";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_LAYER_EXTRUSION_EDIT;
    def.category = RAW_OPTION_CATEGORY_COOLING;
    def.label = "Disable fan for the first";
    def.tooltip = "You can set this to a positive value to disable fan at all during the first layers, so that it does not make adhesion worse.";
    def.sidetext = "layers";
    def.is_vector_extruder = 1;
    def.has_min = def.has_max = 1;
    def.min_value = 0;
    def.max_value = 1000;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "1";
    if (orchestrator_create_option_def(orch, &def) != OPTION_DEF_ERROR_OK)
        throw std::runtime_error("Cannot register disable_fan_first_layers.");

    def = raw_config_option_def_init();
    def.opt_key = "full_fan_speed_layer";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_LAYER_EXTRUSION_EDIT;
    def.category = RAW_OPTION_CATEGORY_FILAMENT;
    def.label = "Full fan speed at layer";
    def.tooltip = "Fan speed will be ramped up linearly from zero at layer \"disable_fan_first_layers\" to maximum at layer \"full_fan_speed_layer\". \"full_fan_speed_layer\" will be ignored if equal or lower than \"disable_fan_first_layers\", in which case the fan will be running at maximum allowed speed at layer \"disable_fan_first_layers\" + 1.\nset 0 to disable";
    def.is_vector_extruder = 1;
    def.has_min = def.has_max = 1;
    def.min_value = 0;
    def.max_value = 1000;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "4";
    if (orchestrator_create_option_def(orch, &def) != OPTION_DEF_ERROR_OK)
        throw std::runtime_error("Cannot register full_fan_speed_layer.");
}

void FirstLayerFan::setup_run_impl(const plugin_run_context *run) const
{
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run);
    if (ctx && ctx->layer_group)
        progress().add_max(ProcessParameterHelpers::extrusion_tree_count(PrintingLayerGroup(ctx->layer_group)));
}

void FirstLayerFan::run_impl(const plugin_run_context *run) const
{
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run);
    if (!ctx || !ctx->print || !ctx->layer_group) return;
    const Config config = Print(ctx->print).config();
    PrintingLayerGroup layer(ctx->layer_group);
    const uint32_t source_index = m_indices.at(ctx->group_idx).at(ctx->layer_group_idx).index;
    for (uint32_t idx = 0; idx < layer.tool_group_count(); ++idx) {
        throw_if_cancelled(run);
        PrintingToolGroup tool = layer.tool_group(idx);
        const FanPolicy::Context *published = m_context.get(tool.properties());
        const raw_fan_policy_phase expected = published && published->phase == RAW_FAN_POLICY_PHASE_BASE ?
            RAW_FAN_POLICY_PHASE_BASE : RAW_FAN_POLICY_PHASE_TIME;
        FanPolicy::Context time = FanTreeHelpers::read_context(tool, m_context, expected);
        // With no cooling plugin, BASE is the numerical identity for TIME.
        time.phase = RAW_FAN_POLICY_PHASE_TIME;
        const FanPolicy::Settings settings = FanPolicy::read_first_layer_settings(config, tool.extruder_id());
        const FanPolicy::Context closed = FanPolicy::close_first_layers(time, settings, source_index);
        for (uint32_t extrusion = 0; extrusion < tool.extrusion_count(); ++extrusion) {
            FanTreeHelpers::for_each_leaf(tool.extrusion(extrusion).mutable_root(),
                [&](MutableExtrusionEntity leaf, const ProcessParameterHelpers::EffectiveTreeState &state) {
                    if (!state.fan.can_modify(FAN_REQUEST_MODE_AUTOMATIC)) return;
                    const raw_extrusion_role role = state.attributes.extrusion_role();
                    const float percent = state.fan.is_set() ?
                        FanPolicy::first_layer_percent(state.fan.percent, FanPolicy::fan_role(role), settings, source_index) :
                        FanTreeHelpers::closed_reference(closed, role);
                    if (percent >= 0.f)
                        leaf.get_or_add(EPropertyFan::key) = FanPolicy::transform_request(state.fan, percent, FAN_REQUEST_MODE_AUTOMATIC);
                }, run);
            progress().increment();
        }
        m_context.get_or_add(tool.properties()) = closed;
    }
}
} // namespace

void register_first_layer_fan_plugin(orchestrator_handle *orch)
{
    static FirstLayerFan plugin(orch);
    orchestrator_register_plugin(orch, plugin.c_instance());
}
}}} // namespace slic3r_api::LayerExtrusionEdit::FirstLayerFanPlugin
