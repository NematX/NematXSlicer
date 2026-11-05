/*
Whole-plan wipe dispatch. The host lends a priority-ordered service snapshot to
each manager and keeps separate error and diagnostic contexts for each service.
Wipe roots are plan-owned synthetic geometry with fixed execution order.
*/
#include "StepGenerateWipeTower.hpp"

#include <atomic>
#include <exception>
#include <memory>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_wipetower.h"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Steps/StepRunner.hpp"

namespace Slic3r::Steps::StepGenerateWipeTower {

// Install wipe-tower generation and wipe-service selector declarations on the supplied host.
void register_selector_traits(Orchestrator &orchestrator);

void register_selector_traits(Orchestrator &orchestrator)
{
    StepSelectorTraits tower;
    tower.category = RAW_OPTION_CATEGORY_OUTPUT;
    tower.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    tower.invalidates_step = STEP_WIPETOWER;
    tower.show_with_one_provider = false;
    const int32_t tower_registered = orchestrator.register_builtin_step_selector(STEP_WIPETOWER, std::move(tower));
    assert(tower_registered == 1);
    (void)tower_registered;
    StepSelectorTraits wipe;
    wipe.category = RAW_OPTION_CATEGORY_OUTPUT;
    wipe.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    const int32_t wipe_registered = orchestrator.register_builtin_step_selector(WIPE_TOWER_WIPE, std::move(wipe));
    assert(wipe_registered == 1);
    (void)wipe_registered;
}

// No source-tree output belongs to this step; plan rollback owns removal.
void clear(Print &print);
// No preparation changes are required before whole-plan dispatch.
void prepare(Print &print);
// No additional source preconditions are checked; accepts a borrowed Print.
bool validate_pre(const Print &print, std::string *error);
// Check synthetic provenance and execution flags, reporting the first path on failure.
bool validate_post(const Print &print, std::string *error);
// Run managers sequentially with run-scoped service state and throw on plugin or output failures.
void run_step(Orchestrator &orchestrator, Print &print);

namespace {

struct WipeServiceState {
    Plugin *plugin = nullptr;
    plugin_host_context host = {};
    std::atomic_bool error { false };
    bool initialized = false;
};

struct WipeStepHostContext {
    Orchestrator &orchestrator;
    Print &print;
    std::vector<std::unique_ptr<WipeServiceState>> services;
    std::exception_ptr failure;
};

// Inspect the borrowed tree recursively so wipe roles on descendants also require synthetic provenance.
bool has_wipe_role(const ExtrusionEntity &entity);
// Run an indexed borrowed service payload; preserve exceptions in the host and return zero on failure.
int32_t call_wipe_plugin(const run_ctx_generate_wipe_tower *ctx, uint32_t index,
                         run_ctx_wipe_tower_wipe *payload);

// Recursively inspect role modifiers without mutating or retaining any entity.
bool has_wipe_role(const ExtrusionEntity &entity)
{
    if (const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>())
        if (attributes->extrusion_role().is_wipetower())
            return true;
    for (size_t idx = 0; idx < entity.child_count(); ++idx)
        if (has_wipe_role(entity.child(idx)))
            return true;
    return false;
}

// Borrow an indexed phase payload, lend plan mutation only to EDIT, and initialize with null data; retain failures and return zero without transferring ownership.
int32_t call_wipe_plugin(const run_ctx_generate_wipe_tower *ctx, const uint32_t index,
                         run_ctx_wipe_tower_wipe *payload)
{
    if (ctx == nullptr || ctx->host_context == nullptr || payload == nullptr)
        return 0;
    WipeStepHostContext &host = *static_cast<WipeStepHostContext *>(ctx->host_context);
    if (index >= host.services.size() || host.failure != nullptr)
        return 0;
    WipeServiceState &service = *host.services[index];
    try {
        payload->print = ctx->print;
        payload->plan = ctx->plan;
        payload->mutable_plan = payload->phase == C_WIPE_TOWER_EDIT ? ctx->plan : nullptr;
        plugin_run_context run = host.orchestrator.prepare_plugin_run_context(
            WIPE_TOWER_WIPE, service.plugin, &service.host);
        if (!service.initialized) {
            run.data = nullptr;
            service.plugin->setup(run, 1);
            Detail::throw_if_plugin_stopped(host.orchestrator, host.print, WIPE_TOWER_WIPE,
                                            *service.plugin, service.error, "setup");
            service.initialized = true;
        }
        run.data = payload;
        service.plugin->setup_run(run);
        Detail::throw_if_plugin_stopped(host.orchestrator, host.print, WIPE_TOWER_WIPE,
                                        *service.plugin, service.error, "setup_run");
        service.plugin->run(run);
        Detail::throw_if_plugin_stopped(host.orchestrator, host.print, WIPE_TOWER_WIPE,
                                        *service.plugin, service.error, "run");
        return 1;
    } catch (...) {
        host.failure = std::current_exception();
        return 0;
    }
}

} // namespace

void clear(Print &) {}
void prepare(Print &) {}

bool validate_pre(const Print &, std::string *)
{
    return true;
}

// Walk the borrowed plan; every wipe bucket or subtree needs a complete synthetic entry and fixed root.
bool validate_post(const Print &print, std::string *error)
{
    if (error != nullptr)
        error->clear();
    const Printing::PrintingPlan *plan = print.printing_plan();
    if (plan == nullptr) {
        if (error != nullptr)
            *error = "plan: the PrintingPlan is missing";
        return false;
    }
    for (size_t g = 0; g < plan->groups.size(); ++g)
        for (size_t l = 0; l < plan->groups[g].layers.size(); ++l)
            for (size_t t = 0; t < plan->groups[g].layers[l].tool_groups.size(); ++t) {
                const Printing::PrintingToolGroup &tool = plan->groups[g].layers[l].tool_groups[t];
                for (size_t e = 0; e < tool.extrusions.size(); ++e) {
                    const Printing::PrintingExtrusion &extrusion = tool.extrusions[e];
                    if (!extrusion.sregion_island_role.is_wipetower() &&
                        (extrusion.root == nullptr || !has_wipe_role(*extrusion.root)))
                        continue;
                    const Printing::PrintingObjectInstance *instance = plan->object_instance(extrusion.printing_catalog_id);
                    std::string cause;
                    if (instance == nullptr || instance->object != nullptr ||
                        instance->object_instance_idx != size_t(-1) || instance->name.empty() ||
                        instance->projected_outline.empty() || instance->projected_outline.area() == 0.0)
                        cause = "wipe extrusion must reference a valid synthetic catalog entry";
                    else if (extrusion.region_island != nullptr)
                        cause = "wipe extrusion must have no source region island";
                    else if (extrusion.root == nullptr || extrusion.root->can_sort() || extrusion.root->can_reverse())
                        cause = "wipe root must exist and be non-sortable and non-reversible";
                    if (!cause.empty()) {
                        if (error != nullptr)
                            *error = "group[" + std::to_string(g) + "].layer[" + std::to_string(l) +
                                "].tool[" + std::to_string(t) + "].extrusion[" + std::to_string(e) +
                                "] (id=" + std::to_string(extrusion.printing_catalog_id) + "): " + cause;
                        return false;
                    }
                }
            }
    return true;
}

// Borrow the Print, initialize managers with null data, and dispatch whole-plan runs with lent service state; output stays plan-owned and plugin or validation failures throw.
void run_step(Orchestrator &orchestrator, Print &print)
{
    Printing::PrintingPlan &plan = print.mutable_printing_plan();
    std::string last_plugin_id = "<plugin pipeline>";
    const std::vector<Plugin *> managers = selected_or_active_plugins_for_step(
        orchestrator, STEP_WIPETOWER, &print.full_print_config());
    for (Plugin *manager : managers) {
        if (manager == nullptr)
            continue;
        last_plugin_id = manager->get_id();
        WipeStepHostContext host { orchestrator, print, {}, nullptr };
        for (Plugin *plugin : orchestrator.get_active_plugins_for_step(WIPE_TOWER_WIPE)) {
            std::unique_ptr<WipeServiceState> service = std::make_unique<WipeServiceState>();
            service->plugin = plugin;
            service->host = Detail::prepare_guarded_plugin_host_context(
                orchestrator, WIPE_TOWER_WIPE, *plugin, print, service->error);
            host.services.push_back(std::move(service));
        }
        run_ctx_generate_wipe_tower payload = {};
        payload.print = reinterpret_cast<const print_handle *>(&print);
        payload.plan = reinterpret_cast<printing_plan_handle *>(&plan);
        payload.host_context = &host;
        payload.wipe_plugin_count = static_cast<uint32_t>(host.services.size());
        payload.call_wipe_plugin = &call_wipe_plugin;

        std::atomic_bool execution_error { false };
        plugin_host_context manager_host = Detail::prepare_guarded_plugin_host_context(
            orchestrator, STEP_WIPETOWER, *manager, print, execution_error);
        plugin_run_context run = orchestrator.prepare_plugin_run_context(STEP_WIPETOWER, manager, &manager_host);
        run.data = nullptr;
        manager->setup(run, 1);
        Detail::throw_if_plugin_stopped(orchestrator, print, STEP_WIPETOWER, *manager, execution_error, "setup");
        run.data = &payload;
        manager->setup_run(run);
        Detail::throw_if_plugin_stopped(orchestrator, print, STEP_WIPETOWER, *manager, execution_error, "setup_run");
        manager->run(run);
        if (host.failure != nullptr)
            std::rethrow_exception(host.failure);
        Detail::throw_if_plugin_stopped(orchestrator, print, STEP_WIPETOWER, *manager, execution_error, "run");
        print.pipeline_progress().complete_plugin(STEP_WIPETOWER, manager->get_id());
    }
    std::string error;
    if (!validate_post(print, &error))
        throw RuntimeError("STEP_WIPETOWER post-validation failed after the plugin pipeline "
                           "(last executed plugin: '" + last_plugin_id + "'): " + error);
}

} // namespace Slic3r::Steps::StepGenerateWipeTower
