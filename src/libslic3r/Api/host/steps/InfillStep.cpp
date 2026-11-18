///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "InfillStep.hpp"
#include "libslic3r/ExtrusionSourcePublication.hpp"

#include <atomic>
#include <algorithm>
#include <cassert>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/InfillPatternService.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Steps/StepRunner.hpp"
#include "libslic3r/Surface.hpp"

/*
Phase-local host services for STEP_INFILL plugins. These callbacks resolve
patterns, collect discovery assignments, apply recipe modifiers and publish
extrusions. They do not run the STEP_INFILL plugin chain.

The host retains the C++ step state behind the data passed to C callbacks.
It catches callback failures before returning through C. Stop checks pass the
owning Print to Orchestrator. A registered execution uses its own stop flag;
an unregistered Print uses the shared flag.
Publication stages owned scopes in a generic Children root. Existing buckets
retain their prefixes; a new bucket receives the prepared base state. Source
scopes and child order survive without a compatibility collection staging object.
This source publication is distinct from canonical acquisition into PrintingPlan.

Operation notes:
- payload: expose the unchanged C callback table to one object job
- finish_discovery: seal assignments after the common runner's setup_run barrier
- Host callbacks: resolve patterns, collect work and publish extrusion
- Phase-boundary stop checks: stop before starting the next phase

Representative direct calls (branches are not a timeline):

    modify_surface_recipe_callback()
        `-- prepare_recipe_modifier_plugins() [once, when first used]

    generate_pattern_callback()
        `-- initialize_pattern_plugins() [once, at the first call after discovery]

    append_region_island_extrusion_callback()
        |-- append_source_extrusions() [prepare owned scopes before touching output]
        `-- append prepared children or exchange an empty bucket's base state
*/

namespace Slic3r::ApiHost::Steps {
namespace {

bool surface_needs_infill(const Surface &surface)
{
    return !surface.empty() && !surface.has_fill_void();
}

size_t count_candidate_surfaces(const Print &print)
{
    size_t count = 0;
    for (const PrintObject &object : print.objects())
        for (const Layer &layer : object.layers())
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region_island : island.regions_islands())
                    for (const Surface &surface : region_island.fill_surfaces())
                        if (surface_needs_infill(surface))
                            ++count;
    return count;
}

// A generator phase can use:
// - one selected STEP_INFILL generator;
// - many INFILL_PATTERN service plugins, one per available fill pattern;
// - optional INFILL_SURFACE_RECIPE_MODIFIER service plugins.
//
// The C ABI stores only opaque pointers and function callbacks, so this struct
// is the host-side state behind those callbacks. It lives for the duration of
// the step run and is reached through run_ctx_generate_infill::host_context.
} // namespace

struct InfillGenerationHostContext
{
    // Build the pattern service before any generator callback can run.
    InfillGenerationHostContext(Orchestrator &orchestrator, Print &print)
        : orchestrator(&orchestrator), print(&print),
          patterns(orchestrator, print, count_candidate_surfaces(print), Slic3r::Steps::InfillPatternService::SetupPatterns::Allow)
    {}

    Orchestrator *orchestrator = nullptr;
    Print *print = nullptr;

    // Runtime pattern ids, per-pattern host contexts and error flags. Config
    // still stores stable string plugin ids; the generator resolves those
    // strings through resolve_pattern_id_callback() before filling each
    // surface.
    Slic3r::Steps::InfillPatternService patterns;

    // Discovery result reported by the generator during its setup_run() phase.
    // The host owns a copy of every group so the arrays stay valid until the
    // per-pattern initialization call.
    struct HostAssignment
    {
        const layer_island_handle *layer_island = nullptr;
        const layer_region_island_handle *region_island = nullptr;
        raw_infill_pattern_params params = {};
        std::vector<const surface_handle *> surfaces;
    };

    struct HostObjectInit
    {
        const object_handle *object = nullptr;
        std::vector<HostAssignment> assignments;
        // Built once before the pattern setup call; points into assignments.
        std::vector<raw_infill_pattern_assignment> raw_assignments;
    };

    std::map<infill_pattern_runtime_id, std::vector<HostObjectInit>> pattern_assignments;
    std::mutex assignments_mutex;
    std::once_flag modifier_setup;
    std::once_flag pattern_setup;
    bool discovery_finished = false;

    // Recipe modifiers are called as a priority-ordered chain for every fill
    // surface. They share the same mutable raw_infill_pattern_params so each
    // modifier sees the changes made by the previous one.
    std::vector<Plugin *> recipe_modifier_plugins;
    std::map<Plugin *, plugin_host_context> recipe_modifier_host_contexts;
    std::map<Plugin *, std::unique_ptr<std::atomic_bool>> recipe_modifier_execution_errors;
};

namespace {
using InfillStepHostContext = InfillGenerationHostContext;
void initialize_pattern_plugins(InfillStepHostContext &host);
void prepare_recipe_modifier_plugins(InfillStepHostContext &host);

// Check whether the plugin runner or the current print has requested a stop.
bool step_is_stopping(const InfillStepHostContext &host) noexcept;
// Resolve a stable pattern id, returning INVALID for refusal, cancellation or a contained failure.
infill_pattern_runtime_id resolve_pattern_id_callback(const run_ctx_generate_infill *ctx,
                                                      const char *pattern_plugin_id) noexcept;
// Return the stable plugin id, never NULL, or "" when the runtime id is unknown or the step is stopping.
const char *pattern_plugin_id_callback(const run_ctx_generate_infill *ctx,
                                       infill_pattern_runtime_id pattern_id) noexcept;
// Copy discovery assignments and return 0 when the call is refused or contained.
int32_t report_pattern_assignments_callback(const run_ctx_generate_infill *ctx,
                                            infill_pattern_runtime_id pattern_id,
                                            const raw_infill_pattern_assignment *assignments,
                                            uint32_t assignment_count) noexcept;
// Run the selected pattern and return 0 with empty output after a stop or failure.
int32_t generate_pattern_callback(const run_ctx_generate_infill *ctx,
                                  infill_pattern_runtime_id pattern_id,
                                  const layer_handle *layer,
                                  const layer_island_handle *island,
                                  const layer_region_island_handle *region_island,
                                  const layer_region_handle *primary_region,
                                  const surface_handle *surface,
                                  const expolygon_collection_handle *no_overlap_areas,
                                  const raw_infill_pattern_params *params,
                                  extrusion_entity_handle *output) noexcept;
// Run recipe modifiers in priority order and return 0 when the step is stopping (including after a modifier call) or
// at the first contained failure.
int32_t modify_surface_recipe_callback(const run_ctx_generate_infill *ctx,
                                       const layer_handle *layer,
                                       const layer_island_handle *island,
                                       const layer_region_island_handle *region_island,
                                       const layer_region_handle *primary_region,
                                       const surface_handle *surface,
                                       const expolygon_collection_handle *no_overlap_areas,
                                       raw_infill_pattern_params *params) noexcept;
// Append staged source scopes to the selected bucket; return 1 on success and 0 on a stop or contained failure.
// Borrow handles under exclusive access for this call. Preparation may partly consume the source before publication.
// The host records failures and requests plugin cancellation; print cancellation is handled by the runner.
int32_t append_region_island_extrusion_callback(layer_region_island_handle *region_island_handle,
                                                raw_extrusion_role role,
                                                extrusion_entity_handle *extrusion_handle) noexcept;

// Read this Print's user-cancel and execution-stop flags without changing them.
bool step_is_stopping(const InfillStepHostContext &host) noexcept
{
    return (host.orchestrator != nullptr && host.orchestrator->is_plugin_cancelled(host.print)) ||
           (host.print != nullptr && host.print->canceled());
}

std::atomic_bool &execution_error_for(std::map<Plugin *, std::unique_ptr<std::atomic_bool>> &errors,
                                      Plugin &plugin)
{
    std::unique_ptr<std::atomic_bool> &error = errors[&plugin];
    if (error == nullptr)
        error = std::make_unique<std::atomic_bool>(false);
    return *error;
}

// All callbacks stored in run_ctx_generate_infill use this helper to recover
// the C++ state that cannot be captured by a plain C function pointer.
InfillStepHostContext *host_from_context(const run_ctx_generate_infill *ctx)
{
    if (ctx == nullptr || ctx->host_context == nullptr)
        return nullptr;
    return reinterpret_cast<InfillStepHostContext *>(ctx->host_context);
}

// Resolve the borrowed plugin-id string through the step-owned pattern registry. Null contexts return INVALID;
// unknown ids keep the service's normal fallback behavior. A C++ failure is contained, recorded and requests a
// plugin stop, while an existing plugin or print stop adds no message. The callback owns no input strings.
infill_pattern_runtime_id resolve_pattern_id_callback(const run_ctx_generate_infill *ctx,
                                                      const char *pattern_plugin_id) noexcept
{
    InfillStepHostContext *host = host_from_context(ctx);
    if (host == nullptr)
        return INFILL_PATTERN_RUNTIME_ID_INVALID;
    try {
        if (step_is_stopping(*host))
            return INFILL_PATTERN_RUNTIME_ID_INVALID;
        if (host->patterns.empty())
            throw RuntimeError("No active infill pattern plugin is available.");
        return host->patterns.resolve_pattern_id(pattern_plugin_id);
    } catch (...) {
        Slic3r::Steps::contain_pattern_callback_failure(*host->orchestrator, STEP_INFILL, nullptr);
        return INFILL_PATTERN_RUNTIME_ID_INVALID;
    }
}

// Return the registry-owned plugin id for a runtime id. Null contexts and stopped steps return ""; unknown ids
// also return "". Map lookup and c_str() do not allocate, and the returned pointer remains borrowed from Plugin.
const char *pattern_plugin_id_callback(const run_ctx_generate_infill *ctx,
                                       infill_pattern_runtime_id pattern_id) noexcept
{
    InfillStepHostContext *host = host_from_context(ctx);
    if (host == nullptr || step_is_stopping(*host))
        return "";
    return host->patterns.pattern_plugin_id(pattern_id);
}

// Copy borrowed discovery arrays into step-owned vectors. Invalid payloads return 0 without mutation; an existing
// stop returns 0 without a message. Allocation failure is contained and requests cancellation, and any partial
// copy remains private to this step because pattern initialization follows the generator's setup_run() check.
int32_t report_pattern_assignments_callback(const run_ctx_generate_infill *ctx,
                                            infill_pattern_runtime_id pattern_id,
                                            const raw_infill_pattern_assignment *assignments,
                                            uint32_t assignment_count) noexcept
{
    InfillStepHostContext *host = host_from_context(ctx);
    if (host == nullptr || ctx->object == nullptr || pattern_id == INFILL_PATTERN_RUNTIME_ID_INVALID ||
        (assignment_count != 0 && assignments == nullptr))
        return 0;

    try {
        if (step_is_stopping(*host))
            return 0;

        std::lock_guard<std::mutex> lock(host->assignments_mutex);
        if (host->discovery_finished)
            throw RuntimeError("Infill assignments were reported after discovery finished.");
        std::vector<InfillStepHostContext::HostObjectInit> &objects = host->pattern_assignments[pattern_id];
        if (objects.empty() || objects.back().object != ctx->object) {
            objects.emplace_back();
            objects.back().object = ctx->object;
        }
        InfillStepHostContext::HostObjectInit &target = objects.back();

        for (uint32_t index = 0; index < assignment_count; ++index) {
            const raw_infill_pattern_assignment &source = assignments[index];
            InfillStepHostContext::HostAssignment assignment;
            assignment.layer_island = source.layer_island;
            assignment.region_island = source.region_island;
            if (source.params != nullptr)
                assignment.params = *source.params;
            if (source.surfaces != nullptr)
                assignment.surfaces.assign(source.surfaces, source.surfaces + source.surface_count);
            target.assignments.emplace_back(std::move(assignment));
        }
        return 1;
    } catch (...) {
        Slic3r::Steps::contain_pattern_callback_failure(*host->orchestrator, STEP_INFILL, nullptr);
        return 0;
    }
}

// Build a borrowed pattern payload and run it through the step-owned service. Null required inputs are ordinary
// refusals; a pending stop returns 0 after emptying output. A failure empties output and requests a plugin stop.
// A PluginStopped adds no diagnostic (a known cause already has its own message; an Unknown cause has none, and
// the final step error says a stop was requested); any other exception is recorded as a host failure, unless the
// print was cancelled. The service owns its contexts; output stays caller-owned.
int32_t generate_pattern_callback(const run_ctx_generate_infill *ctx,
                                  infill_pattern_runtime_id pattern_id,
                                  const layer_handle *layer,
                                  const layer_island_handle *island,
                                  const layer_region_island_handle *region_island,
                                  const layer_region_handle *primary_region,
                                  const surface_handle *surface,
                                  const expolygon_collection_handle *no_overlap_areas,
                                  const raw_infill_pattern_params *params,
                                  extrusion_entity_handle *output) noexcept
{
    if (ctx == nullptr || ctx->host_context == nullptr || output == nullptr)
        return 0;

    InfillStepHostContext &host = *reinterpret_cast<InfillStepHostContext *>(ctx->host_context);
    try {
        if (step_is_stopping(host)) {
            Slic3r::Steps::empty_output(output);
            return 0;
        }
        if (params == nullptr)
            return 0;
        // The first generation after the discovery barrier initializes the used
        // patterns once, with the groups of every object; concurrent callers wait.
        if (host.discovery_finished)
            std::call_once(host.pattern_setup, [&host] {
                if (!step_is_stopping(host)) initialize_pattern_plugins(host);
            });
        if (step_is_stopping(host)) {
            Slic3r::Steps::empty_output(output);
            return 0;
        }

        run_ctx_infill_pattern payload = {};
        payload.print = ctx->print;
        payload.object = ctx->object;
        payload.layer = layer;
        payload.island = island;
        payload.region_island = region_island;
        payload.primary_region = primary_region;
        payload.surface = surface;
        payload.no_overlap_areas = no_overlap_areas;
        payload.params = params;
        payload.output = output;
        return host.patterns.generate_pattern(pattern_id, payload);
    } catch (...) {
        Slic3r::Steps::contain_pattern_callback_failure(*host.orchestrator, STEP_INFILL, output);
        return 0;
    }
}

// Run borrowed modifiers in priority order against one mutable recipe. Invalid inputs and a pending stop are refused;
// plugin failures are contained without rolling back earlier edits. Each plugin keeps its own host context, while the
// generator retains recipe ownership. A plugin stop or print cancellation observed after a modifier phase returns 0
// without another diagnostic: a known cause already has its own message, an Unknown cause has none (the final step
// error says a stop was requested), and the runner rethrows a print cancellation.
int32_t modify_surface_recipe_callback(const run_ctx_generate_infill *ctx,
                                       const layer_handle *layer,
                                       const layer_island_handle *island,
                                       const layer_region_island_handle *region_island,
                                       const layer_region_handle *primary_region,
                                       const surface_handle *surface,
                                       const expolygon_collection_handle *no_overlap_areas,
                                       raw_infill_pattern_params *params) noexcept
{
    if (ctx == nullptr || ctx->host_context == nullptr || params == nullptr) {
        return 0;
    }

    InfillStepHostContext &host = *reinterpret_cast<InfillStepHostContext *>(ctx->host_context);
    try {
        if (step_is_stopping(host))
            return 0;
        std::call_once(host.modifier_setup, [&host] {
            if (!step_is_stopping(host)) prepare_recipe_modifier_plugins(host);
        });
        if (step_is_stopping(host)) return 0;
        if (host.recipe_modifier_plugins.empty())
            return 1;

        for (Plugin *plugin : host.recipe_modifier_plugins) {
            if (plugin == nullptr)
                continue;

            std::map<Plugin *, plugin_host_context>::iterator host_context =
                host.recipe_modifier_host_contexts.find(plugin);
            assert(host_context != host.recipe_modifier_host_contexts.end());
            if (host_context == host.recipe_modifier_host_contexts.end())
                continue;

            plugin_run_context run_context =
                host.orchestrator->prepare_plugin_run_context(
                    INFILL_SURFACE_RECIPE_MODIFIER, plugin, &host_context->second);

            run_ctx_infill_surface_recipe_modifier payload = {};
            payload.print = ctx->print;
            payload.object = ctx->object;
            payload.layer = layer;
            payload.island = island;
            payload.region_island = region_island;
            payload.primary_region = primary_region;
            payload.surface = surface;
            payload.no_overlap_areas = no_overlap_areas;
            payload.generate_infill_ctx = ctx;
            payload.resolve_pattern_id = &resolve_pattern_id_callback;
            payload.pattern_plugin_id = &pattern_plugin_id_callback;
            payload.params = params;
            run_context.data = &payload;

            plugin->setup_run(run_context);
            if (step_is_stopping(host))
                return 0;
            Slic3r::Steps::Detail::throw_if_plugin_stopped(
                *host.orchestrator, *host.print, INFILL_SURFACE_RECIPE_MODIFIER, *plugin,
                *host.recipe_modifier_execution_errors.at(plugin), "setup_run");
            plugin->run(run_context);
            if (step_is_stopping(host))
                return 0;
            Slic3r::Steps::Detail::throw_if_plugin_stopped(
                *host.orchestrator, *host.print, INFILL_SURFACE_RECIPE_MODIFIER, *plugin,
                *host.recipe_modifier_execution_errors.at(plugin), "run");
        }

        return 1;
    } catch (...) {
        Slic3r::Steps::contain_pattern_callback_failure(*host.orchestrator, STEP_INFILL, nullptr);
        return 0;
    }
}

ExtrusionRole bucket_role_from_raw(raw_extrusion_role role)
{
    if ((role & RAW_EXTRUSION_ROLE_THIN) != 0 || role == RAW_EXTRUSION_ROLE_GAP_FILL)
        return LayerRegionIsland::GAP_FILLS;
    if ((role & RAW_EXTRUSION_ROLE_INFILL) != 0)
        return LayerRegionIsland::INFILLS;
    if ((role & RAW_EXTRUSION_ROLE_IRONING) != 0)
        return LayerRegionIsland::IRONINGS;
    if ((role & RAW_EXTRUSION_ROLE_MILL) != 0)
        return LayerRegionIsland::MILLS;
    if ((role & RAW_EXTRUSION_ROLE_SUPPORT) != 0)
        return (role & RAW_EXTRUSION_ROLE_EXTERNAL) != 0 ? LayerRegionIsland::SUPPORT_INTERFACE :
                                                           LayerRegionIsland::SUPPORT;
    return LayerRegionIsland::INFILLS;
}

int32_t append_region_island_extrusion_callback(layer_region_island_handle *region_island_handle,
                                                raw_extrusion_role role,
                                                extrusion_entity_handle *extrusion_handle) noexcept
{
    if (region_island_handle == nullptr || extrusion_handle == nullptr)
        return 0;

    Orchestrator &orchestrator = Orchestrator::instance();
    if (orchestrator.is_plugin_cancelled())
        return 0;

    try {
        LayerRegionIsland *region_island = reinterpret_cast<LayerRegionIsland *>(region_island_handle);
        ExtrusionEntity *extrusion = reinterpret_cast<ExtrusionEntity *>(extrusion_handle);
        if (region_island == nullptr || extrusion == nullptr)
            return 0;

        ExtrusionEntity prepared(ExtrusionEntity::Children{}, true, true, false);
        append_source_extrusions(prepared, *extrusion);
        if (prepared.child_count() == 0)
            return 1;

        const ExtrusionRole bucket_role = bucket_role_from_raw(role);
        // Geometry-free source scopes still carry recipes for post-infill.
        // Append to the stored bucket even when has_extrusion() is false.
        ExtrusionEntity &dst = region_island->mutable_extrusion(bucket_role);
        ExtrusionEntity::Children &dst_children = dst.children();
        ExtrusionEntity::Children &src_children = prepared.children();
        dst_children.reserve(dst_children.size() + src_children.size());
        dst_children.insert(dst_children.end(), std::make_move_iterator(src_children.begin()), std::make_move_iterator(src_children.end()));
        src_children.clear();
        return 1;
    } catch (...) {
        Slic3r::Steps::contain_pattern_callback_failure(orchestrator, STEP_INFILL, nullptr);
        return 0;
    }
}

// Call setup() once per pattern plugin that the generator actually uses. The
// payload lists every object of the print and the work groups discovered for
// that pattern, so the plugin can precompute per-object state and parallelize
// the initialization itself.
void initialize_pattern_plugins(InfillStepHostContext &host)
{
    assert(host.orchestrator != nullptr);
    assert(host.print != nullptr);

    for (std::pair<const infill_pattern_runtime_id, std::vector<InfillStepHostContext::HostObjectInit>> &entry :
         host.pattern_assignments) {
        if (step_is_stopping(host)) return;
        const auto object_index = [&host](const object_handle *object) {
            for (size_t index = 0; index < host.print->objects().size(); ++index)
                if (reinterpret_cast<const object_handle *>(&host.print->object(index)) == object) return index;
            return host.print->objects().size();
        };
        std::sort(entry.second.begin(), entry.second.end(), [&object_index](const auto &a, const auto &b) {
            return object_index(a.object) < object_index(b.object);
        });
        uint32_t surface_count = 0;
        for (InfillStepHostContext::HostObjectInit &object : entry.second) {
            object.raw_assignments.clear();
            object.raw_assignments.reserve(object.assignments.size());
            for (InfillStepHostContext::HostAssignment &assignment : object.assignments) {
                raw_infill_pattern_assignment raw = {};
                raw.layer_island = assignment.layer_island;
                raw.region_island = assignment.region_island;
                raw.params = &assignment.params;
                raw.surfaces = assignment.surfaces.empty() ? nullptr : assignment.surfaces.data();
                raw.surface_count = uint32_t(assignment.surfaces.size());
                surface_count += raw.surface_count;
                object.raw_assignments.emplace_back(raw);
            }
        }

        std::vector<raw_infill_pattern_object_init> objects;
        objects.reserve(entry.second.size());
        for (const InfillStepHostContext::HostObjectInit &object : entry.second) {
            raw_infill_pattern_object_init raw = {};
            raw.object = object.object;
            raw.assignments = object.raw_assignments.empty() ? nullptr : object.raw_assignments.data();
            raw.assignment_count = uint32_t(object.raw_assignments.size());
            objects.emplace_back(raw);
        }

        run_ctx_infill_pattern_init payload = {};
        payload.print = reinterpret_cast<const print_handle *>(host.print);
        payload.objects = objects.empty() ? nullptr : objects.data();
        payload.object_count = uint32_t(objects.size());
        host.patterns.initialize_pattern(entry.first, payload, surface_count);
    }
}

void prepare_recipe_modifier_plugins(InfillStepHostContext &host)
{
    assert(host.orchestrator != nullptr);
    assert(host.print != nullptr);

    // Recipe modifiers are another service chain owned by STEP_INFILL. They do
    // not traverse the print by themselves: the generator asks the host to run
    // them for each surface after it has built the default recipe and before it
    // calls the selected pattern plugin.
    const size_t run_count = count_candidate_surfaces(*host.print);
    host.recipe_modifier_plugins =
        host.orchestrator->get_active_plugins_for_step(INFILL_SURFACE_RECIPE_MODIFIER);

    for (Plugin *plugin : host.recipe_modifier_plugins) {
        if (plugin == nullptr)
            continue;

        // Use the number of candidate surfaces as the advertised run count
        // because each modifier can be called once for every fillable surface.
        // The actual call order still follows the generator traversal.
        std::atomic_bool &execution_error = execution_error_for(host.recipe_modifier_execution_errors, *plugin);
        plugin_host_context plugin_host =
            Slic3r::Steps::Detail::prepare_guarded_plugin_host_context(
                *host.orchestrator, INFILL_SURFACE_RECIPE_MODIFIER, *plugin, *host.print, execution_error);
        plugin_host.object_count = run_count;
        host.recipe_modifier_host_contexts.emplace(plugin, plugin_host);

        plugin_run_context setup_context =
            host.orchestrator->prepare_plugin_run_context(
                INFILL_SURFACE_RECIPE_MODIFIER,
                plugin,
                &host.recipe_modifier_host_contexts.find(plugin)->second);
        plugin->setup(setup_context, uint32_t(run_count));
        Slic3r::Steps::Detail::throw_if_plugin_stopped(
            *host.orchestrator, *host.print, INFILL_SURFACE_RECIPE_MODIFIER, *plugin, execution_error, "setup");
    }
}

} // namespace

InfillGenerationContext::InfillGenerationContext(Orchestrator &orchestrator, Print &print)
    : m_host(std::make_unique<InfillGenerationHostContext>(orchestrator, print)) {}

InfillGenerationContext::~InfillGenerationContext() = default;

run_ctx_generate_infill InfillGenerationContext::payload(PrintObject &object) const
{
    run_ctx_generate_infill result = {};
    result.print = reinterpret_cast<const print_handle *>(m_host->print);
    result.object = reinterpret_cast<const object_handle *>(&object);
    result.host_context = m_host.get();
    result.resolve_pattern_id = &resolve_pattern_id_callback;
    result.pattern_plugin_id = &pattern_plugin_id_callback;
    result.generate_pattern = &generate_pattern_callback;
    result.modify_surface_recipe = &modify_surface_recipe_callback;
    result.append_region_island_extrusion = &append_region_island_extrusion_callback;
    result.report_pattern_assignments = &report_pattern_assignments_callback;
    return result;
}

void InfillGenerationContext::finish_discovery() noexcept
{
    // The generic object runner has joined all setup_run workers.
    m_host->discovery_finished = true;
}

} // namespace Slic3r::ApiHost::Steps
