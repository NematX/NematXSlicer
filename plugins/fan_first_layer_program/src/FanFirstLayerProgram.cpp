///|/ Copyright (c) 2026 NematX
///|/
///|/ Released under the GNU Affero General Public License, version 3 or later.

/*
First-layer fan program with enable and gradient layers
=======================================================

Filament settings, per extruder:
- enable_fan_first_layers (E, default 0): the fan runs normally on the first E layers;
- disable_fan_first_layers (D, defined by the host): then it stops for D layers;
- fan_speed_layer_gradient (G, default -1): the changes are ramped over G layers.
The formulas are those of the legacy slicer (FanLayerProgram.hpp).

This plugin only publishes the program of each layer and tool on
PrintingToolGroup::properties(), at LAYER_EXTRUSION_EDIT_PRIORITY_FAN_PROGRAM (23).
The host first-layer stage (24) applies it to the references and to the
modifiable requests; it never changes an Override or another plugin's mode.

With E = 0 and G = -1 (the defaults) nothing is published and the host keeps its
own program from disable_fan_first_layers and full_fan_speed_layer. With E > 0
and G = -1 the gradient is 4 layers, the legacy default. full_fan_speed_layer is
not converted into a gradient: a published program replaces it.

The layer ordinal is the index of the layer group in its printing group, as for
the host stage.
*/

#include <algorithm>
#include <iterator>
#include <stdexcept>
#include <string>

#include "FanLayerProgram.hpp"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_plugin.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/FanPolicy.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"

namespace nematx_fan_layer_program {
namespace {

using namespace slic3r_api;

constexpr const char *k_plugin_id = "nematx_fan_first_layer_program";
constexpr const char *k_enable_key = "enable_fan_first_layers";
constexpr const char *k_disable_key = "disable_fan_first_layers";
constexpr const char *k_gradient_key = "fan_speed_layer_gradient";

const char *const k_no_dependencies[] = { nullptr };
const raw_used_config_key k_used_config_keys[] = {
    { k_enable_key, RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { k_disable_key, RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { k_gradient_key, RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE }
};
const char *const k_defined_config_keys[] = { k_enable_key, k_gradient_key };

FanPolicy::FirstLayerProgram to_program(const Step &step)
{
    FanPolicy::FirstLayerProgram program;
    program.state = step.state == State::Off ? FanPolicy::FirstLayerProgram::Off :
                    step.state == State::Ramp ? FanPolicy::FirstLayerProgram::Ramp : FanPolicy::FirstLayerProgram::Normal;
    program.factor = step.factor;
    return program;
}

// A filament vector setting of the legacy slicer, in layers.
void define_layer_count(orchestrator_handle *orchestrator, const char *key, raw_option_category category,
                        const char *label, const char *tooltip, double min_value, const char *default_value)
{
    raw_config_option_def def = raw_config_option_def_init();
    def.opt_key = key;
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_LAYER_EXTRUSION_EDIT;
    def.category = category;
    def.label = label;
    def.tooltip = tooltip;
    def.sidetext = "layers";
    def.is_vector_extruder = 1;
    def.has_min = def.has_max = 1;
    def.min_value = min_value;
    def.max_value = 1000;
    def.mode = raw_config_option_mode(RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA);
    def.default_serialized_value = default_value;
    const option_def_error_code result = orchestrator_create_option_def(orchestrator, &def);
    if (result != OPTION_DEF_ERROR_OK)
        throw std::runtime_error(std::string("Cannot define ") + key + " (option definition error " +
                                 std::to_string(int(result)) + ").");
}

class FanFirstLayerProgram final : public PluginBase
{
public:
    explicit FanFirstLayerProgram(orchestrator_handle *orchestrator)
        : PluginBase(orchestrator), m_program(FanPolicy::first_layer_program_key(orchestrator)) {}

private:
    const char *id_impl() const noexcept override { return k_plugin_id; }
    const char *name_impl() const noexcept override { return "First-layer fan program with enable and gradient layers"; }
    const char *description_impl() const noexcept override
    {
        return "Publishes the first-layer fan program of the legacy slicer: enabled, disabled and ramped layers.";
    }
    slicing_step_t step_impl() const noexcept override { return STEP_LAYER_EXTRUSION_EDIT; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    int32_t priority_impl() const noexcept override { return LAYER_EXTRUSION_EDIT_PRIORITY_FAN_PROGRAM; }

    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        if (keys != nullptr)
            std::copy(std::begin(k_used_config_keys), std::end(k_used_config_keys), keys);
        return int32_t(std::size(k_used_config_keys));
    }
    int32_t defined_config_keys(const char **keys) const noexcept override
    {
        if (keys != nullptr)
            std::copy(std::begin(k_defined_config_keys), std::end(k_defined_config_keys), keys);
        return int32_t(std::size(k_defined_config_keys));
    }

    void inilialize_impl(storage_handle *) const override
    {
        define_layer_count(m_orchestrator, k_enable_key, RAW_OPTION_CATEGORY_COOLING, "Enable fan for the first",
            "You can set this to a positive value to force enable the fan during the first layers, if needed for "
            "adhesion tuning.\nThis setting is applied before 'disable_fan_first_layers': if you want to enable the "
            "fan on the first layer then disable it for the next 2 layers, set this setting to 1 and "
            "'disable_fan_first_layers' to 2.", 0., "0");
        define_layer_count(m_orchestrator, k_gradient_key, RAW_OPTION_CATEGORY_FILAMENT, "Change fan speed over",
            "Fan speed will be ramped up linearly from the previous value to the next value over this amount of "
            "layers. It's only used to go from 'enable_fan_first_layers' to 'disable_fan_first_layers' to the normal "
            "speed algorithm.\nSet 0 for an instant change.\nSet -1 to use 4 layers when 'enable_fan_first_layers' is "
            "set, and otherwise 'full_fan_speed_layer'.", -1., "-1");
    }

    // Publish the program of every tool of this layer group; requests are not edited.
    void run_impl(const plugin_run_context *run_ctx) const override
    {
        throw_if_cancelled(run_ctx);
        const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
        if (ctx == nullptr || ctx->print == nullptr || ctx->layer_group == nullptr)
            throw std::runtime_error("The first-layer fan program received an invalid layer-group callback context.");
        const Config config = Print(ctx->print).config();
        PrintingLayerGroup layer_group(ctx->layer_group);
        for (uint32_t tool_idx = 0; tool_idx < layer_group.tool_group_count(); ++tool_idx) {
            PrintingToolGroup tool = layer_group.tool_group(tool_idx);
            const extruder_index_t extruder = tool.extruder_id();
            const std::optional<Step> step = program_step(config.vector_int_or_default(k_enable_key, extruder, 0),
                                                          config.vector_int_or_default(k_disable_key, extruder, 0),
                                                          config.vector_int_or_default(k_gradient_key, extruder, -1),
                                                          ctx->layer_group_idx);
            if (!step)
                continue;
            if (m_program.get(tool.properties()) != nullptr)
                throw std::runtime_error("A first-layer fan program is already published for this layer and tool.");
            m_program.get_or_add(tool.properties()) = to_program(*step);
        }
    }

    PluginPropertyKey<FanPolicy::FirstLayerProgram> m_program;
};

} // namespace
} // namespace nematx_fan_layer_program

// Host ABI entry point. Register a static instance so it stays alive for every callback.
extern "C" SLIC3R_PLUGIN_API void register_plugin(orchestrator_handle *orchestrator)
{
    static nematx_fan_layer_program::FanFirstLayerProgram plugin(orchestrator);
    orchestrator_register_plugin(orchestrator, plugin.c_instance());
}

// Keep this header last so its ABI registration includes every public API header above.
#include "libslic3r/Api/plugin/c/slic3r_plugin_register_version.h"
