///|/ Copyright (c) 2026 NematX
///|/ Released under the GNU Affero General Public License, version 3 or later.

#include <stdexcept>
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/RetractionCrossingPolicy.hpp"

/* Publish the legacy top-surface exception to the common retraction planner.
   The setting is print-wide, defaults to true and invalidates the layer editing
   step. Each worker edits only its layer's roots, after transition creation and
   before retraction planning. No route or E-axis event is created here.

       register_plugin -> register the shared policy layout
       inilialize_impl -> define enforce_retract_top_surface
       run_impl -> publish the selected mask on each PrintingExtrusion root

   The host checks the selected travel against final published top surfaces,
   retaining its distance threshold, explicit modifiers and tool/layer rules.
   Fresh roots are created on replay; writing zero also clears an earlier policy
   when a caller reruns this editor over the same roots with the setting off. */
namespace nematx_top_surface_retraction {
namespace {
using namespace slic3r_api;
constexpr const char *k_setting = "enforce_retract_top_surface";
const char *const k_dependencies[] = {"layer_extrusion_edit.transition_scope.default", nullptr};

class TopSurfaceRetraction final : public PluginBase
{
public:
    // Retain only the orchestrator-local key; print geometry remains borrowed.
    explicit TopSurfaceRetraction(orchestrator_handle *host) : PluginBase(host),
        m_policy(retraction_crossing_policy_key(host)) {}

private:
    // Identify an additive policy, not a replacement travel provider.
    const char *id_impl() const noexcept override { return "nematx_top_surface_retraction"; }
    // Name the policy in the plugin catalogue.
    const char *name_impl() const noexcept override { return "Retraction over top surfaces"; }
    // Explain its limited effect on ordinary travel requests.
    const char *description_impl() const noexcept override { return "Retains ordinary travel retractions over exposed top areas."; }
    // Edit the plan only after ordering has fixed the printing roots.
    slicing_step_t step_impl() const noexcept override { return STEP_LAYER_EXTRUSION_EDIT; }
    // Require scopes so the following retraction planner can consume the policy.
    const char *const *dependencies_impl() const noexcept override { return k_dependencies; }
    // Run between scope creation (-100) and retraction planning (-90).
    int32_t priority_impl() const noexcept override { return -95; }

    // Declare the single legacy setting for profile storage and replay invalidation.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        if (keys != nullptr)
            keys[0] = {k_setting, RAW_CO_BOOL, RAW_CONTAINER_TYPE_PROJECT, RAW_PRESET_TYPE_FFF_PRINT};
        return 1;
    }
    // Own the setting without redefining the host's other retraction controls.
    int32_t defined_config_keys(const char **keys) const noexcept override
    {
        if (keys != nullptr) keys[0] = k_setting;
        return 1;
    }
    // Register the historical default and labels; a rejected definition is an error.
    void inilialize_impl(storage_handle *) const override
    {
        raw_config_option_def def = raw_config_option_def_init();
        def.opt_key = k_setting;
        def.type = RAW_CO_BOOL;
        def.category = RAW_OPTION_CATEGORY_INFILL;
        def.level = RAW_OPTION_LEVEL_EXPERT;
        def.container_type = RAW_CONTAINER_TYPE_PROJECT;
        def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
        def.printer_technology = RAW_PT_FFF;
        def.invalidates_step = STEP_LAYER_EXTRUSION_EDIT;
        def.label = "But on top areas";
        def.full_label = "Don't check crossings for retraction on top areas";
        def.tooltip = "Let the retraction happens if the travel path goes over a top area.";
        def.mode = raw_config_option_mode(RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI);
        def.default_serialized_value = "1";
        if (orchestrator_create_option_def(m_orchestrator, &def) != OPTION_DEF_ERROR_OK)
            throw std::runtime_error("Cannot define enforce_retract_top_surface.");
    }
    // Publish values, not borrowed surface pointers, on this worker's roots.
    void run_impl(const plugin_run_context *run_ctx) const override
    {
        throw_if_cancelled(run_ctx);
        const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
        if (ctx == nullptr || ctx->print == nullptr || ctx->layer_group == nullptr)
            throw std::runtime_error("Top-surface retraction needs a print and a layer group.");
        const bool enabled = Print(ctx->print).config().bool_or_default(k_setting, true);
        PrintingLayerGroup layer(ctx->layer_group);
        for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
            PrintingToolGroup tool = layer.tool_group(tool_idx);
            for (uint32_t idx = 0; idx < tool.extrusion_count(); ++idx) {
                throw_if_cancelled(run_ctx);
                PrintingExtrusion extrusion = tool.extrusion(idx);
                m_policy.get_or_add(extrusion.mutable_root()).exclude_top_surfaces = enabled ? 1 : 0;
            }
        }
    }
    PluginPropertyKey<RetractionCrossingPolicy> m_policy;
};
}
}

// Register one borrowed plugin instance for the process's orchestrator.
extern "C" SLIC3R_PLUGIN_API void register_plugin(orchestrator_handle *orchestrator)
{
    static nematx_top_surface_retraction::TopSurfaceRetraction plugin(orchestrator);
    orchestrator_register_plugin(orchestrator, plugin.c_instance());
}

// Keep last so the manifest records every public API used above.
#include "libslic3r/Api/plugin/c/slic3r_plugin_register_version.h"
