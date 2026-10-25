///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "StepGenerateInfill.hpp"

#include <atomic>
#include <cassert>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/InfillPatternService.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Steps/StepRunner.hpp"
#include "libslic3r/Surface.hpp"

namespace Slic3r::Steps::StepGenerateInfill {
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

// STEP_INFILL has three kinds of plugins involved in one run:
// - one exclusive STEP_INFILL generator, selected by the step setting;
// - many INFILL_PATTERN service plugins, one per available fill pattern;
// - optional INFILL_SURFACE_RECIPE_MODIFIER service plugins.
//
// The C ABI stores only opaque pointers and function callbacks, so this struct
// is the host-side state behind those callbacks. It lives for the duration of
// the step run and is reached through run_ctx_generate_infill::host_context.
struct InfillStepHostContext
{
    // Build the pattern service before any generator callback can run.
    InfillStepHostContext(Orchestrator &orchestrator, Print &print)
        : orchestrator(&orchestrator), print(&print),
          patterns(orchestrator, print, count_candidate_surfaces(print), InfillPatternService::SetupPatterns::Allow)
    {}

    Orchestrator *orchestrator = nullptr;
    Print *print = nullptr;

    // Runtime pattern ids, per-pattern host contexts and error flags. Config
    // still stores stable string plugin ids; the generator resolves those
    // strings through resolve_pattern_id_callback() before filling each
    // surface.
    InfillPatternService patterns;

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

    // Recipe modifiers are called as a priority-ordered chain for every fill
    // surface. They share the same mutable raw_infill_pattern_params so each
    // modifier sees the changes made by the previous one.
    std::vector<Plugin *> recipe_modifier_plugins;
    std::map<Plugin *, plugin_host_context> recipe_modifier_host_contexts;
    std::map<Plugin *, std::unique_ptr<std::atomic_bool>> recipe_modifier_execution_errors;
};

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

// Translate the stable pattern id stored in configuration into the compact
// runtime id used by raw_infill_pattern_params. Keeping this translation in
// the host lets recipe modifiers switch patterns without depending on string
// buffer ownership or active-plugin ordering details.
infill_pattern_runtime_id resolve_pattern_id_callback(const run_ctx_generate_infill *ctx,
                                                      const char *pattern_plugin_id)
{
    InfillStepHostContext *host = host_from_context(ctx);
    if (host == nullptr)
        return INFILL_PATTERN_RUNTIME_ID_INVALID;
    return host->patterns.resolve_pattern_id(pattern_plugin_id);
}

// Reverse lookup used mostly by diagnostics and recipe modifiers that want to
// inspect the current recipe. The returned string is owned by Plugin and stays
// valid while the plugin registry exists; callers should treat it as borrowed.
const char *pattern_plugin_id_callback(const run_ctx_generate_infill *ctx,
                                       infill_pattern_runtime_id pattern_id)
{
    InfillStepHostContext *host = host_from_context(ctx);
    if (host == nullptr)
        return "";
    return host->patterns.pattern_plugin_id(pattern_id);
}

// Collect the work groups the generator discovered for one pattern plugin in
// the object currently processed by setup_run(). The host copies everything it
// needs, so the generator keeps ownership of its own arrays.
int32_t report_pattern_assignments_callback(const run_ctx_generate_infill *ctx,
                                            infill_pattern_runtime_id pattern_id,
                                            const raw_infill_pattern_assignment *assignments,
                                            uint32_t assignment_count)
{
    InfillStepHostContext *host = host_from_context(ctx);
    if (host == nullptr || ctx->object == nullptr || pattern_id == INFILL_PATTERN_RUNTIME_ID_INVALID ||
        (assignment_count != 0 && assignments == nullptr))
        return 0;

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
}

// Run the pattern selected by pattern_id on one surface of the current object.
// The shared service validates the id and guards the plugin phases.
int32_t generate_pattern_callback(const run_ctx_generate_infill *ctx,
                                  infill_pattern_runtime_id pattern_id,
                                  const layer_handle *layer,
                                  const layer_island_handle *island,
                                  const layer_region_island_handle *region_island,
                                  const layer_region_handle *primary_region,
                                  const surface_handle *surface,
                                  const expolygon_collection_handle *no_overlap_areas,
                                  const raw_infill_pattern_params *params,
                                  extrusion_entity_handle *output)
{
    if (ctx == nullptr || ctx->host_context == nullptr || params == nullptr || output == nullptr)
        return 0;

    InfillStepHostContext &host = *reinterpret_cast<InfillStepHostContext *>(ctx->host_context);
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
}

int32_t modify_surface_recipe_callback(const run_ctx_generate_infill *ctx,
                                       const layer_handle *layer,
                                       const layer_island_handle *island,
                                       const layer_region_island_handle *region_island,
                                       const layer_region_handle *primary_region,
                                       const surface_handle *surface,
                                       const expolygon_collection_handle *no_overlap_areas,
                                       raw_infill_pattern_params *params)
{
    if (ctx == nullptr || ctx->host_context == nullptr || params == nullptr) {
        return 0;
    }

    InfillStepHostContext &host = *reinterpret_cast<InfillStepHostContext *>(ctx->host_context);
    if (host.recipe_modifier_plugins.empty())
        return 1;

    // Recipe modifiers are a small service chain. Each plugin receives the
    // recipe produced by the generator and any changes made by earlier
    // modifiers. This makes independent tweaks composable: one plugin can
    // adjust density while another changes priority or pattern id.
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

        // A recipe modifier edits only raw_infill_pattern_params. It gets the
        // same layer/surface context as the generator plus resolver callbacks
        // so it can switch to another active INFILL_PATTERN without touching
        // string buffers owned by the generator.
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
        Detail::throw_if_plugin_stopped(
            *host.orchestrator, *host.print, INFILL_SURFACE_RECIPE_MODIFIER, *plugin,
            execution_error_for(host.recipe_modifier_execution_errors, *plugin), "setup_run");
        plugin->run(run_context);
        Detail::throw_if_plugin_stopped(
            *host.orchestrator, *host.print, INFILL_SURFACE_RECIPE_MODIFIER, *plugin,
            execution_error_for(host.recipe_modifier_execution_errors, *plugin), "run");
    }

    return 1;
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

void append_extrusion_children(ExtrusionEntityCollection &dst, ExtrusionEntity &src)
{
    if (src.is_nop())
        return;

    if (ExtrusionEntityCollection *collection = dynamic_cast<ExtrusionEntityCollection *>(&src)) {
        // A collection with direct properties is a semantic subtree root. For
        // example, STEP_INFILL stores the source Surface id there and expects
        // every generated child to inherit it. Flatten only plain transport
        // collections that do not carry metadata of their own.
        if (collection->has_properties()) {
            dst.append(std::move(src));
            return;
        }
        dst.append_move_from(*collection);
        return;
    }

    if (src.is_leaf()) {
        dst.append(std::move(src));
        return;
    }

    ExtrusionEntity::Children &children = src.children();
    while (!children.empty()) {
        dst.append(std::move(children.front()));
        children.erase(children.begin());
    }
}

int32_t append_region_island_extrusion_callback(layer_region_island_handle *region_island_handle,
                                                raw_extrusion_role role,
                                                extrusion_entity_handle *extrusion_handle)
{
    LayerRegionIsland *region_island = reinterpret_cast<LayerRegionIsland *>(region_island_handle);
    ExtrusionEntity *extrusion = reinterpret_cast<ExtrusionEntity *>(extrusion_handle);
    if (region_island == nullptr || extrusion == nullptr)
        return 0;

    // Publication is append-only for this step. The generator may call the
    // callback several times for one LayerRegionIsland, once per surface, and
    // all generated paths must stay under the same infill bucket.
    ExtrusionEntityCollection &dst = region_island->mutable_extrusion(bucket_role_from_raw(role));
    append_extrusion_children(dst, *extrusion);
    region_island->remove_empty_extrusions();
    return 1;
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
            Detail::prepare_guarded_plugin_host_context(
                *host.orchestrator, INFILL_SURFACE_RECIPE_MODIFIER, *plugin, *host.print, execution_error);
        plugin_host.object_count = run_count;
        host.recipe_modifier_host_contexts.emplace(plugin, plugin_host);

        plugin_run_context setup_context =
            host.orchestrator->prepare_plugin_run_context(
                INFILL_SURFACE_RECIPE_MODIFIER,
                plugin,
                &host.recipe_modifier_host_contexts.find(plugin)->second);
        plugin->setup(setup_context, uint32_t(run_count));
        Detail::throw_if_plugin_stopped(
            *host.orchestrator, *host.print, INFILL_SURFACE_RECIPE_MODIFIER, *plugin, execution_error, "setup");
    }
}

void prepare_generator_for_object(Orchestrator &orchestrator,
                                  Plugin &plugin,
                                  Print &print,
                                  PrintObject &object,
                                  plugin_host_context &host_context,
                                  InfillStepHostContext &infill_context,
                                  std::atomic_bool &execution_error)
{
    plugin_run_context run_context =
        orchestrator.prepare_plugin_run_context(STEP_INFILL, &plugin, &host_context);

    // Discovery pass: the generator resolves every fill surface recipe, groups
    // surfaces that share one recipe and reports the groups so the host can
    // initialize the used pattern plugins before any extrusion is generated.
    run_ctx_generate_infill payload = {};
    payload.print = reinterpret_cast<const print_handle *>(&print);
    payload.object = reinterpret_cast<const object_handle *>(&object);
    payload.host_context = &infill_context;
    payload.resolve_pattern_id = &resolve_pattern_id_callback;
    payload.pattern_plugin_id = &pattern_plugin_id_callback;
    payload.generate_pattern = &generate_pattern_callback;
    payload.modify_surface_recipe = &modify_surface_recipe_callback;
    payload.append_region_island_extrusion = &append_region_island_extrusion_callback;
    payload.report_pattern_assignments = &report_pattern_assignments_callback;
    run_context.data = &payload;

    plugin.setup_run(run_context);
    Detail::throw_if_plugin_stopped(orchestrator, print, STEP_INFILL, plugin, execution_error, "setup_run");
}

void run_generator_for_object(Orchestrator &orchestrator,
                              Plugin &plugin,
                              Print &print,
                              PrintObject &object,
                              plugin_host_context &host_context,
                              InfillStepHostContext &infill_context,
                              std::atomic_bool &execution_error)
{
    plugin_run_context run_context =
        orchestrator.prepare_plugin_run_context(STEP_INFILL, &plugin, &host_context);

    // The generator receives a small callback table instead of direct access to
    // host internals. This keeps pattern selection, recipe modifier ordering
    // and publication into LayerRegionIsland owned by the host step.
    run_ctx_generate_infill payload = {};
    payload.print = reinterpret_cast<const print_handle *>(&print);
    payload.object = reinterpret_cast<const object_handle *>(&object);
    payload.host_context = &infill_context;
    payload.resolve_pattern_id = &resolve_pattern_id_callback;
    payload.pattern_plugin_id = &pattern_plugin_id_callback;
    payload.generate_pattern = &generate_pattern_callback;
    payload.modify_surface_recipe = &modify_surface_recipe_callback;
    payload.append_region_island_extrusion = &append_region_island_extrusion_callback;
    payload.report_pattern_assignments = &report_pattern_assignments_callback;
    run_context.data = &payload;

    plugin.run(run_context);
    Detail::throw_if_plugin_stopped(orchestrator, print, STEP_INFILL, plugin, execution_error, "run");
}

} // namespace

void clear(Print &)
{
    // Intentionally empty: infill lives in the region islands, which are either
    // destroyed by a slicing restart or left untouched by ordering/plan replays.
}

void prepare(Print &) {}

bool validate_pre(const Print &, std::string *)
{
    return true;
}

bool validate_post(const Print &, std::string *)
{
    return true;
}

void run_step(Orchestrator &orchestrator, Print &print)
{
    // STEP_INFILL is an exclusive step. The selected generator owns the
    // high-level traversal and parameter preparation, while this host wrapper
    // provides callbacks for pattern-service plugins and data-tree mutation.
    Plugin *plugin = selected_or_active_plugin_for_step(orchestrator, STEP_INFILL, &print.full_print_config());
    if (plugin == nullptr) {
        orchestrator.add_plugin_message(Orchestrator::PluginMessageLevel::Error,
                                        nullptr,
                                        STEP_INFILL,
                                        "No active infill generator plugin is available.");
        return;
    }

    InfillStepHostContext infill_context(orchestrator, print);
    prepare_recipe_modifier_plugins(infill_context);
    if (infill_context.patterns.empty()) {
        orchestrator.add_plugin_message(Orchestrator::PluginMessageLevel::Error,
                                        plugin,
                                        INFILL_PATTERN,
                                        "No active infill pattern plugin is available.");
        return;
    }

    std::atomic_bool execution_error { false };
    plugin_host_context host_context =
        Detail::prepare_guarded_plugin_host_context(orchestrator, STEP_INFILL, *plugin, print, execution_error);
    host_context.object_count = print.objects().size();
    plugin_run_context setup_context = orchestrator.prepare_plugin_run_context(STEP_INFILL, plugin, &host_context);
    plugin->setup(setup_context, uint32_t(print.objects().size()));
    Detail::throw_if_plugin_stopped(orchestrator, print, STEP_INFILL, *plugin, execution_error, "setup");

    // Discovery: the generator resolves every fill surface recipe and reports
    // its work groups. This must finish for every object before the used
    // pattern plugins are initialized.
    for (size_t object_idx = 0; object_idx < print.objects().size(); ++object_idx) {
        host_context.object_idx = object_idx;
        if (setup_context.is_cancelled != nullptr && setup_context.is_cancelled(setup_context.host_context))
            Detail::throw_if_plugin_stopped(orchestrator, print, STEP_INFILL, *plugin, execution_error, "setup_run");
        prepare_generator_for_object(
            orchestrator, *plugin, print, print.object(object_idx), host_context, infill_context, execution_error);
    }

    // Per-object initialization of every pattern plugin the generator uses.
    initialize_pattern_plugins(infill_context);

    // Generation uses the recipes cached by the discovery pass.
    for (size_t object_idx = 0; object_idx < print.objects().size(); ++object_idx) {
        host_context.object_idx = object_idx;
        if (setup_context.is_cancelled != nullptr && setup_context.is_cancelled(setup_context.host_context))
            Detail::throw_if_plugin_stopped(orchestrator, print, STEP_INFILL, *plugin, execution_error, "run");
        run_generator_for_object(
            orchestrator, *plugin, print, print.object(object_idx), host_context, infill_context, execution_error);
    }
    print.pipeline_progress().complete_plugin(STEP_INFILL, plugin->get_id());
}

} // namespace Slic3r::Steps::StepGenerateInfill
