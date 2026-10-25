///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "SeamPlacerRouter.hpp"

#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_seam_placer.h"
#include "libslic3r/Api/plugin/cpp/OrchestratorViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/SeamPlacerViews.hpp"
#include "libslic3r/Plugins/Ordering/ExtrusionTreeOrderingGeometry.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"

/*
SeamPlacerRouter.cpp
====================

This file adapts the configuration-oriented view of seam placement to the
geometry-oriented view used by the ordering pipeline. A PrintRegion selects a
SEAM_PLACER provider, while an ordering worker later receives only an atomic
closed loop and its current starting position. The router builds the missing
association before workers start.

The complete construction path is:

    SeamPlacerRouter::SeamPlacerRouter()
    |-- collect_print_regions()
    |-- group_regions_by_provider()
    |   `-- OrchestratorView::select_plugin()
    |-- create_sessions()
    |   `-- OrchestratorView::execute()
    |       `-- one provider session initialization per batch
    `-- build_plan_routes()
        `-- build_entity_routes() recursively for every extrusion tree

The resulting tables are immutable after construction:

    PrintRegion handle -> shared SeamPlacer session
    ExtrusionEntity handle -> selected LayerRegion and shared session

Several PrintRegions can select the same provider, so they share one session
instead of causing repeated initialization. Several loops can use that session
concurrently during ordering; the session implementation is therefore required
to be safe for the concurrent calls described by SeamPlacerViews.hpp. The
router itself only reads its maps after construction.

Borrowed Print, PrintingPlan, LayerRegion and extrusion handles must remain
valid for the lifetime of the router. The router does not take ownership of
any data-tree object. Its shared_ptr values own only the C++ SeamPlacer wrapper
and, through it, the provider session published by the provider.
*/

namespace slic3r_api { namespace SeamPlacement {
namespace {

const char *const k_seam_placer_group = "seam_placer_plugin";

struct ProviderBatch;
struct LoopRoute;

/*
Enumerate every valid PrintRegion that may be referenced by the plan, once.

The Print is visited first to preserve the normal object/region order. The
plan is then visited as a fallback for regions referenced by generated plan
data but not present in the ordinary object list. This order is used for
deterministic provider batches and session initialization. In
build_entity_routes(), exact per-loop priority ties retain the candidate
LayerRegion/source-island order.
*/
std::vector<PrintRegion> collect_print_regions(const Print &print, const PrintingPlan &plan);

/*
Resolve the configured SEAM_PLACER provider for each region and group regions
selecting the same provider into one initialization batch. The orchestrator and
region views are borrowed; returned batches own their storage and retain borrowed
provider and region handles. An unresolved choice is a configuration error, reported
with its region index, selector key, and ID, "#<rank>", or "<missing>" so it is distinct
from a provider-session execution failure.
*/
std::vector<ProviderBatch> group_regions_by_provider(
    orchestrator_handle *orchestrator,
    const std::vector<PrintRegion> &regions);

/*
Execute one provider session per batch and map every assigned PrintRegion to
the resulting shared SeamPlacer wrapper. The raw region-pointer array is valid
for the provider execution call; the provider receives borrowed plan and
region views and publishes ownership only for its session object.
*/
std::map<const print_region_handle *, std::shared_ptr<SeamPlacer>> create_sessions(
    orchestrator_handle *orchestrator,
    const Print &print,
    PrintingPlan &plan,
    std::vector<ProviderBatch> &batches);

/*
Resolve all atomic loops below one extrusion entity. A loop is an indivisible
seam-placement unit; non-loop entities are only traversal nodes and are
visited recursively. When several LayerRegions can describe a loop, the
provider priority selects one region before the route is stored.
*/
void build_entity_routes(
    storage_handle *scratch_storage,
    PrintingPlan &plan,
    const PrintingExtrusion &extrusion,
    const ExtrusionEntity &entity,
    const std::map<const print_region_handle *, std::shared_ptr<SeamPlacer>> &sessions,
    std::map<const extrusion_entity_handle *, LoopRoute> &routes);

/*
Walk every PrintingExtrusion root in plan order and populate the immutable
loop-to-provider routing table used by place_seam().
*/
std::map<const extrusion_entity_handle *, LoopRoute> build_plan_routes(
    storage_handle *scratch_storage,
    PrintingPlan &plan,
    const std::map<const print_region_handle *, std::shared_ptr<SeamPlacer>> &sessions);

struct ProviderBatch
{
    // The provider is stored once for all regions selecting it.
    PluginView provider;
    // C++ views are kept while the provider is executed and while the region
    // handles are copied into raw_regions for the C ABI call.
    std::vector<PrintRegion> regions;
    std::vector<const print_region_handle *> raw_regions;
};

struct LoopRoute
{
    // The LayerRegion whose configuration was selected for this loop.
    LayerRegion region;
    // Shared because all loops assigned to one provider use the same session.
    std::shared_ptr<SeamPlacer> session;
};

std::vector<PrintRegion> collect_print_regions(const Print &print, const PrintingPlan &plan)
{
    std::vector<PrintRegion> result;
    std::map<const print_region_handle *, bool> seen;

    // Start with the ordinary Print object graph. The map is keyed by the
    // borrowed handle because one PrintRegion can be referenced by many
    // layers, islands or extrusions in the plan.
    for (uint32_t object_idx = 0; object_idx < print.object_count(); ++object_idx) {
        const Object object = print.object(object_idx);
        for (uint32_t region_idx = 0; region_idx < object.print_region_count(); ++region_idx) {
            const PrintRegion region = object.print_region(region_idx);
            if (region.valid() && seen.emplace(region.handle(), true).second)
                result.push_back(region);
        }
    }

    // Include regions discoverable only through the transformed PrintingPlan.
    // This also makes the router robust to generated or synthetic plan data
    // that is not represented by the ordinary object-region enumeration.
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        const ConstPrintingGroup group = plan.group(group_idx);
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            const ConstPrintingLayerGroup layer = group.layer_group(layer_idx);
            for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
                const ConstPrintingToolGroup tool = layer.tool_group(tool_idx);
                for (uint32_t idx = 0; idx < tool.extrusion_count(); ++idx) {
                    const LayerRegionIsland island = tool.extrusion(idx).region_island();
                    if (!island.valid()) continue;
                    for (uint32_t region_idx = 0; region_idx < island.region_count(); ++region_idx) {
                        const PrintRegion region = island.region(region_idx).print_region();
                        if (region.valid() && seen.emplace(region.handle(), true).second)
                            result.push_back(region);
                    }
                }
            }
        }
    }
    return result;
}

/* Group borrowed region views by provider handle; the returned vector owns only its batch storage. */
std::vector<ProviderBatch> group_regions_by_provider(
    orchestrator_handle *orchestrator,
    const std::vector<PrintRegion> &regions)
{
    OrchestratorView orchestrator_view(orchestrator);
    std::vector<ProviderBatch> batches;
    for (size_t region_idx = 0; region_idx < regions.size(); ++region_idx) {
        const PrintRegion &region = regions[region_idx];

        // The selector is stored in the region configuration. The router does
        // not choose a default provider: selection must be resolved by the
        // orchestrator so that all provider activation rules are respected.
        const PluginView provider = orchestrator_view.select_plugin(
            region.config().handle(), SEAM_PLACER, k_seam_placer_group);
        if (!provider.valid()) {
            // An absent or stale choice is a configuration error, not a provider-session error.
            const std::string option_key = Slic3r::Steps::plugin_exclusive_group_option_key(
                *reinterpret_cast<const Slic3r::Orchestrator *>(orchestrator),
                SEAM_PLACER, k_seam_placer_group);
            const ConfigOption option = region.config().get(option_key.c_str());
            std::string selected = "<missing>";
            if (option.valid()) {
                selected = option.serialize();
                if (selected.empty())
                    selected = "#" + std::to_string(option.get_int());
            }
            throw std::runtime_error(
                "PrintRegion #" + std::to_string(region_idx) + " has selector '" +
                option_key + "' = '" + selected +
                "', which does not identify an active provider from '" +
                k_seam_placer_group + "'.");
        }

        // Reuse the same batch for equal provider handles. Provider identity,
        // rather than region identity, determines how many sessions are built.
        ProviderBatch *batch = nullptr;
        for (ProviderBatch &candidate : batches) {
            if (candidate.provider.handle() == provider.handle()) {
                batch = &candidate;
                break;
            }
        }
        if (batch == nullptr) {
            batches.push_back(ProviderBatch{});
            batch = &batches.back();
            batch->provider = provider;
        }
        batch->regions.push_back(region);
    }
    return batches;
}

std::map<const print_region_handle *, std::shared_ptr<SeamPlacer>> create_sessions(
    orchestrator_handle *orchestrator,
    const Print &print,
    PrintingPlan &plan,
    std::vector<ProviderBatch> &batches)
{
    OrchestratorView orchestrator_view(orchestrator);
    std::map<const print_region_handle *, std::shared_ptr<SeamPlacer>> result;
    for (ProviderBatch &batch : batches) {
        // The C ABI stores the number of regions in uint32_t. Reject an
        // unrepresentable batch before converting the size or building the
        // pointer array.
        if (batch.regions.size() > std::numeric_limits<uint32_t>::max())
            throw std::runtime_error("A seam placer provider received too many PrintRegions.");
        batch.raw_regions.reserve(batch.regions.size());
        for (const PrintRegion &region : batch.regions)
            batch.raw_regions.push_back(region.handle());

        run_ctx_seam_placer context = {};
        context.plan = plan.mutable_handle();
        context.regions = batch.raw_regions.empty() ? nullptr : batch.raw_regions.data();
        context.region_count = uint32_t(batch.raw_regions.size());

        // execute() calls the provider once for this complete batch. The
        // context and raw_regions remain alive until the call returns; the
        // published instance must own whatever session state it needs after
        // that point.
        const raw_plugin_execution_status status = orchestrator_view.execute(
            batch.provider, print.handle(), context);
        if (status == RAW_PLUGIN_EXECUTION_CANCELLED)
            throw PluginCancelled();
        if (status != RAW_PLUGIN_EXECUTION_SUCCESS)
            throw std::runtime_error(
                "SEAM_PLACER provider '" + batch.provider.id() +
                "' could not initialize its regional session.");

        std::shared_ptr<SeamPlacer> session;
        try {
            // SeamPlacer takes ownership of the C instance's provider session.
            // The shared wrapper is then assigned to every region in this
            // batch, so all loops using one provider share one initialized
            // session without copying it.
            session = std::make_shared<SeamPlacer>(context.instance);
        } catch (const std::exception &exception) {
            throw std::runtime_error(
                "SEAM_PLACER provider '" + batch.provider.id() +
                "' published an invalid session: " + exception.what());
        }
        for (const PrintRegion &region : batch.regions)
            result.emplace(region.handle(), session);
    }
    return result;
}

void build_entity_routes(
    storage_handle *scratch_storage,
    PrintingPlan &plan,
    const PrintingExtrusion &extrusion,
    const ExtrusionEntity &entity,
    const std::map<const print_region_handle *, std::shared_ptr<SeamPlacer>> &sessions,
    std::map<const extrusion_entity_handle *, LoopRoute> &routes)
{
    // Empty nodes carry no executable geometry and cannot produce a route.
    if (entity.empty())
        return;
    if (entity.is_loop()) {
        // The resolver follows the PrintingPlan ancestry and returns the
        // LayerRegions that can legitimately provide configuration for this
        // atomic loop. It must be done once during construction, not by every
        // worker request.
        const std::vector<LayerRegion> candidates =
            Ordering::TreeOrderingGeometry::resolve_source_layer_regions(
                scratch_storage, plan, extrusion, entity);
        LayerRegion selected = candidates.front();
        const print_region_handle *selected_print_region = selected.print_region().handle();
        const std::map<const print_region_handle *, std::shared_ptr<SeamPlacer>>::const_iterator first =
            sessions.find(selected_print_region);
        if (first == sessions.end())
            throw std::runtime_error(
                "A loop candidate region has no initialized SEAM_PLACER session.");
        std::shared_ptr<SeamPlacer> selected_session = first->second;

        if (candidates.size() > 1) {
            // Providers may rank the same loop differently. Strict comparison
            // keeps the first source region on an exact tie and therefore
            // preserves the deterministic plan traversal order.
            float best_priority = selected_session->get_priority(entity, selected);
            for (size_t candidate_idx = 1; candidate_idx < candidates.size(); ++candidate_idx) {
                const LayerRegion candidate = candidates[candidate_idx];
                const print_region_handle *print_region = candidate.print_region().handle();
                const std::map<const print_region_handle *, std::shared_ptr<SeamPlacer>>::const_iterator found =
                    sessions.find(print_region);
                if (found == sessions.end())
                    throw std::runtime_error(
                        "A loop candidate region has no initialized SEAM_PLACER session.");
                const float priority = found->second->get_priority(entity, candidate);
                // Strict comparison preserves source-island order on exact ties.
                if (priority > best_priority) {
                    best_priority = priority;
                    selected = candidate;
                    selected_session = found->second;
                }
            }
        }

        if (!routes.emplace(entity.handle(), LoopRoute{selected, selected_session}).second)
            throw std::runtime_error(
                "An atomic loop appears more than once in the PrintingPlan.");
        return;
    }

    // Composite entities do not receive a seam themselves. Their children are
    // visited depth-first so every atomic loop gets exactly one route entry.
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        build_entity_routes(
            scratch_storage, plan, extrusion, entity.child(child_idx), sessions, routes);
}

std::map<const extrusion_entity_handle *, LoopRoute> build_plan_routes(
    storage_handle *scratch_storage,
    PrintingPlan &plan,
    const std::map<const print_region_handle *, std::shared_ptr<SeamPlacer>> &sessions)
{
    std::map<const extrusion_entity_handle *, LoopRoute> result;

    // Traverse the same group/layer/tool/extrusion hierarchy that ordering
    // later uses. Keeping this order makes duplicate detection and exact
    // priority ties deterministic.
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        PrintingGroup group = plan.group(group_idx);
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            PrintingLayerGroup layer = group.layer_group(layer_idx);
            for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
                PrintingToolGroup tool = layer.tool_group(tool_idx);
                for (uint32_t extrusion_idx = 0; extrusion_idx < tool.extrusion_count(); ++extrusion_idx) {
                    const PrintingExtrusion extrusion = tool.extrusion(extrusion_idx);
                    build_entity_routes(
                        scratch_storage, plan, extrusion, extrusion.root(), sessions, result);
                }
            }
        }
    }
    return result;
}

} // namespace

struct SeamPlacerRouter::Impl
{
    // These values are immutable after the constructor finishes. They are
    // kept behind Impl so the public header needs no definitions for the
    // routing details or the provider session maps.
    std::vector<ProviderBatch> batches;
    std::map<const print_region_handle *, std::shared_ptr<SeamPlacer>> sessions;
    std::map<const extrusion_entity_handle *, LoopRoute> routes;
};

SeamPlacerRouter::SeamPlacerRouter(orchestrator_handle *orchestrator,
                                   const Print &print,
                                   PrintingPlan plan,
                                   storage_handle *scratch_storage) :
    m_impl(new Impl())
{
    // The router is created before parallel seam requests begin. Rejecting a
    // missing input here prevents a partially populated route table from
    // being observed by ordering workers.
    if (orchestrator == nullptr || !print.valid() || !plan.valid() || scratch_storage == nullptr)
        throw std::invalid_argument(
            "SeamPlacerRouter needs an orchestrator, Print, PrintingPlan, and scratch storage.");

    // Construction has four ordered phases: discover regions, resolve their
    // providers, initialize one session per provider, and route every loop.
    // None of the later phases may run with an incomplete result from an
    // earlier phase.
    const std::vector<PrintRegion> regions = collect_print_regions(print, plan);
    m_impl->batches = group_regions_by_provider(orchestrator, regions);
    m_impl->sessions = create_sessions(orchestrator, print, plan, m_impl->batches);
    m_impl->routes = build_plan_routes(scratch_storage, plan, m_impl->sessions);
}

SeamPlacerRouter::~SeamPlacerRouter() = default;

c_point SeamPlacerRouter::place_seam(const ExtrusionEntity &loop,
                                     const c_point start_position) const
{
    // The ordering code may call this method repeatedly and concurrently. All
    // provider and region decisions were made during construction, so this
    // method only performs const map lookup and delegates to the selected
    // session with the caller's current travel position.
    if (!loop.is_loop())
        throw std::invalid_argument(
            "SeamPlacerRouter needs a non-empty fixed continuous closed loop.");
    const std::map<const extrusion_entity_handle *, LoopRoute>::const_iterator route =
        m_impl->routes.find(loop.handle());
    if (route == m_impl->routes.end())
        throw std::runtime_error(
            "The loop has no precomputed regional SEAM_PLACER route.");
    return route->second.session->place_seam(
        loop, route->second.region, start_position);
}

}} // namespace slic3r_api::SeamPlacement
