// Copyright (c) 2026 NematX. AGPL-3.0-or-later.
#include "InternalSeam.hpp"

#include <iterator>
#include <stdexcept>
#include <vector>
#include "libslic3r/Api/plugin/cpp/OrchestratorViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"

/*
NematX seam provider registration and service composition
=========================================================
This provider belongs to the host's regional seam selector. It is selected
before Cost by default, but explicit choices of another provider remain respected.
It owns no new settings: the host Cost service owns the three cost settings.

    register_plugin -> register one static provider
    run_impl -> execute exact host Cost service for the same plan/region batch
             -> make_internal_seam_instance: transfer fallback and publish decorator

The host orchestrator owns execution and temporary storage. The initialized
decorator owns the fallback, so no borrowed callback payload survives run().
Errors are exceptions inside PluginBase's guarded C ABI, never silent fallback
when a required service is absent. Session preparation honors cancellation.
*/
namespace nematx_seam {
namespace {
using namespace slic3r_api;
const char *const dependencies[]{"seam_placer.cost", nullptr};
const raw_used_config_key used[]{
    {"seam_angle_cost", RAW_CO_PERCENT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT},
    {"seam_travel_cost", RAW_CO_PERCENT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT},
    {"seam_visibility", RAW_CO_BOOL, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT}
};

class InternalSeamPlugin final : public PluginBase {
public:
    // Borrow the host orchestrator for registration and later service execution.
    explicit InternalSeamPlugin(orchestrator_handle *host) : PluginBase(host) {}
private:
    // Publish the stable identity used by activation and the regional selector.
    const char *id_impl() const noexcept override { return "nematx_seam"; }
    // Describe the attractor and its unchanged host fallback in the selector.
    const char *name_impl() const noexcept override { return "NematX internal attractor"; }
    const char *description_impl() const noexcept override { return "Attracts internal perimeter seams through a volume's height; otherwise uses cost-based placement."; }
    // Share the host's existing seam group, service step and execution priority.
    const char *exclusive_group_impl() const noexcept override { return "seam_placer_plugin"; }
    slicing_step_t step_impl() const noexcept override { return SEAM_PLACER; }
    int32_t priority_impl() const noexcept override { return 600; }
    // Prefer the decorator by default without changing explicit provider choices.
    int32_t exclusive_group_priority_impl() const noexcept override { return -50; }
    // Require the exact fallback provider, never recursively select this decorator.
    const char *const *dependencies_impl() const noexcept override { return dependencies; }
    // Declare the settings read by the composed fallback; define none a second time.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    // Prepare the fallback and decorator completely before publishing ownership.
    void run_impl(const plugin_run_context *context) const override;
};

int32_t InternalSeamPlugin::used_config_keys(raw_used_config_key *keys) const noexcept
{
    if (keys) for (size_t i = 0; i < std::size(used); ++i) keys[i] = used[i];
    return int32_t(std::size(used));
}

void InternalSeamPlugin::run_impl(const plugin_run_context *context) const
{
    run_ctx_seam_placer *payload = plugin_ctx_as_seam_placer(context);
    if (!payload || !payload->plan || (payload->region_count != 0 && !payload->regions) ||
        payload->instance.session || payload->instance.vtable)
        throw std::invalid_argument("Internal seam provider needs an empty seam payload and a valid plan/region batch.");
    const OrchestratorView host(m_orchestrator);
    const PluginView provider = host.find_plugin("seam_placer.cost");
    if (!provider.valid()) throw std::runtime_error("Internal seam attractor requires the host Cost seam provider.");
    run_ctx_seam_placer base{};
    base.plan = payload->plan; base.regions = payload->regions; base.region_count = payload->region_count;
    // Own any published fallback even if cancellation arrives just after its run.
    struct BaseOwner {
        raw_seam_placer_instance &instance;
        // After transfer instance is empty; otherwise release the original service.
        ~BaseOwner() { if (instance.session && instance.vtable) instance.vtable->destroy(instance.session); }
    } owner{base.instance};
    const raw_plugin_execution_status status = host.execute(provider, context->print, base);
    if (status == RAW_PLUGIN_EXECUTION_CANCELLED) throw PluginCancelled();
    if (status != RAW_PLUGIN_EXECUTION_SUCCESS)
        throw std::runtime_error("Could not initialize the host Cost seam service for the internal attractor.");
    std::vector<PrintRegion> regions;
    for (uint32_t i = 0; i < payload->region_count; ++i) regions.emplace_back(payload->regions[i]);
    payload->instance = make_internal_seam_instance(base.instance, PrintingPlan(payload->plan), regions,
                                                   context->plugin_storage, context);
}
} // namespace
} // namespace nematx_seam

#include "libslic3r/Api/plugin/c/slic3r_plugin_register_version.h"

// Register a static provider whose lifetime covers every host callback.
extern "C" SLIC3R_PLUGIN_API void register_plugin(orchestrator_handle *host)
{
    static nematx_seam::InternalSeamPlugin plugin(host);
    orchestrator_register_plugin(host, plugin.c_instance());
}
