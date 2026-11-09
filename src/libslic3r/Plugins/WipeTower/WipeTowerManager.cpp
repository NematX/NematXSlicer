///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* The manager enumerates config-only global/instance towers, adds contour visits
   only through each owner's last tool-change layer in the unmodified plan before
   service EDIT, and collects value anchors against the final edited plan.
   Pure layout reserves bands and tower layers in print order. Services write
   scaled tower-local roots; maintenance reuses the Monotonic/Grid pattern
   algorithms. All roots are transformed and inserted only after generation.
   Every request that printed something is also bracketed by one speed-ratio save
   before its first leaf and one restore after its last, so the whole tower tool
   change, its T command included, runs at 100% speed.
   A request flagged floating is laid out with the regular bands when the tower
   already reaches its Z; above the top it gets its own floating layer, without
   maintenance, which changes neither depth nor cone.
   A private tool-visit marker survives vector moves and permits removal of unused
   manager-created visits after insertion, without moving any pending anchor. */
#include "WipeTowerManager.hpp"
#include "WipeTowerLayout.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_wipetower.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PluginPropertyKey.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Geometry/ConvexHull.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Slicing.hpp"
#include "libslic3r/WipeTowerFootprint.hpp"
#include "libslic3r/Plugins/Infill/Rectilinear/RectilinearFills.hpp"

namespace slic3r_api::WipeTower {
namespace {
namespace raster = Infill::RectilinearPatternsPlugin::fills;
const uint32_t no_tower = UINT32_MAX;
const raw_used_config_key k_used_keys[] = {
    {"wipe_tower", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"wipe_tower_x", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"wipe_tower_y", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"wipe_tower_width", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"wipe_tower_rotation_angle", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"wipe_tower_extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"wipe_tower_brim_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"wipe_tower_extra_spacing", RAW_CO_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"wipe_tower_bridging", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"wipe_tower_cone_angle", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"wipe_tower_no_sparse_layers", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"wipe_tower_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"wipe_tower_extruder_toolchange", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"wipe_tower_per_object", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"wipe_tower_object_x", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"wipe_tower_object_y", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"wipe_tower_speed", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"first_layer_speed", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"first_layer_min_speed", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"max_layer_height", RAW_CO_VECTOR_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"min_layer_height", RAW_CO_VECTOR_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"z_step", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"nozzle_diameter", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"complete_objects", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"parallel_objects_step", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"spiral_vase", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
};

struct ForcedVisit { uint32_t present; };
struct Tower {
    Slic3r::WipeTowerFootprint footprint;
    const Slic3r::PrintObjectConfig *config;
    uint32_t catalog_id = UINT32_MAX;
    std::vector<uint32_t> owner_ids;
};
struct CollectedRequest {
    uint32_t plugin_index;
    c_wipe_tower_request request;
    uint32_t tower_index = no_tower;
};
struct RequestCollector {
    uint32_t plugin_index = 0;
    std::vector<CollectedRequest> requests;
    std::exception_ptr failure;
};
struct TowerLayer {
    uint32_t group;
    uint32_t layer;
    c_wipe_tower_anchor first_owner;
    LayoutLayerInput input;
};
struct GeneratedPart {
    c_wipe_tower_anchor anchor;
    uint32_t tower;
    bool maintenance;
    double print_z;
    StoredExtrusionEntity root;
};

class WipeTowerManager final : public PluginBase {
public:
    // Borrow the orchestrator and register the private forced-visit marker.
    explicit WipeTowerManager(orchestrator_handle *orchestrator);
private:
    // Return the stable manager activation identifier.
    const char *id_impl() const noexcept override;
    // Return the display name.
    const char *name_impl() const noexcept override;
    // Select the whole-plan wipe-tower producer step.
    slicing_step_t step_impl() const noexcept override;
    // Declare no plugin dependencies.
    const char *const *dependencies_impl() const noexcept override;
    // Use the default manager priority.
    int32_t priority_impl() const noexcept override;
    // Join the wipe-tower step's exclusive selector.
    const char *exclusive_group_impl() const noexcept override;
    // Copy typed configuration dependencies into the optional caller array.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    // Edit contour visits, dispatch services, lay out all towers and publish generated geometry.
    void run_impl(const plugin_run_context *run) const override;
    PluginPropertyKey<ForcedVisit> m_forced_visit;
};

// Copy a service request into manager-owned storage and retain reporter failures.
int32_t collect_request(void *context, const c_wipe_tower_request *request);
// Check post-EDIT anchor bounds and concrete configured tools.
void validate_anchor(const ConstPrintingPlan &plan, const c_wipe_tower_anchor &anchor, size_t extruder_count);
// Find an owner's first extrusion in a layer, or the global layer-start anchor.
bool first_owner_anchor(const ConstPrintingLayerGroup &layer, const Tower &tower, c_wipe_tower_anchor &anchor);
// Map the following extrusion to a tower, preferring its instance over the global tower.
uint32_t choose_tower(const ConstPrintingPlan &plan, const c_wipe_tower_request &request,
                      const std::vector<Tower> &towers);
// Add marked contour visits through the owner's last unmodified tool-change layer before service EDIT.
void add_contour_visits(PrintingPlan &plan, const std::vector<Tower> &towers,
                        const PluginPropertyKey<ForcedVisit> &marker);
// Remove only marked visits that remain otherwise empty and erase all manager markers.
void clean_contour_visits(PrintingPlan &plan, const PluginPropertyKey<ForcedVisit> &marker);
// Scan an owner's layer candidates and attach ordered requests and their resolved widths.
std::vector<TowerLayer> tower_layers(const Slic3r::Print &print, const ConstPrintingPlan &plan,
                                    const Tower &tower, uint32_t tower_idx,
                                    const std::vector<CollectedRequest> &requests);
// Return true when a service filled an owned root with points, children or properties.
bool root_has_content(const StoredExtrusionEntity &root);
// Insert one fixed point-less speed command before or after the current content of an owned root.
void insert_speed_command(MutableExtrusionEntity root, OrderedLeafPosition position,
                          c_extrusion_special_command code, double value);
// Append one fixed point-less speed command to the before events of a tool visit.
void append_visit_speed_command(storage_handle *storage, PrintingToolGroup tool,
                                c_extrusion_special_command code, double value);
// Find the maintenance anchor using the selected contour tool or the first tower visit.
c_wipe_tower_anchor maintenance_anchor(const ConstPrintingPlan &plan, const Tower &tower,
                                       const TowerLayer &layer, const std::vector<CollectedRequest> &requests,
                                       uint32_t tower_idx);
// Append complete local maintenance geometry using the shared Monotonic/Grid generators.
StoredExtrusionEntity maintenance_root(storage_handle *storage, const Slic3r::Print &print,
                                       const Tower &tower, const TowerLayout &layout, size_t layer_idx,
                                       extruder_index_t extruder);
// Append one scaled local path with complete flow and speed attributes to an owned root.
void append_path(storage_handle *storage, MutableExtrusionEntity root, const Slic3r::Points &points,
                  const Slic3r::Flow &flow, double speed);
// Rotate then translate every local polyline recursively before ownership moves into the plan.
void transform_root(MutableExtrusionEntity root, double angle, c_point origin);

// Initialize borrowed plugin metadata and a movable tool marker; registration/layout failures propagate.
WipeTowerManager::WipeTowerManager(orchestrator_handle *orchestrator) : PluginBase(orchestrator),
    m_forced_visit(PluginPropertyKey<ForcedVisit>::register_dynamic(orchestrator, "wipe_tower.manager.forced_visit")) {}
// Return constant module-owned metadata without allocation or failure.
const char *WipeTowerManager::id_impl() const noexcept { return "wipe_tower.manager.default"; }
// Return a constant module-owned display name without failure.
const char *WipeTowerManager::name_impl() const noexcept { return "Default wipe tower manager"; }
// Return the producer identity without changing state.
slicing_step_t WipeTowerManager::step_impl() const noexcept { return STEP_WIPETOWER; }
// Return no dependencies; the host lends wipe services explicitly.
const char *const *WipeTowerManager::dependencies_impl() const noexcept { return nullptr; }
// Return the default scheduling priority without side effects.
int32_t WipeTowerManager::priority_impl() const noexcept { return 0; }
// Return the existing selector as a borrowed constant string.
const char *WipeTowerManager::exclusive_group_impl() const noexcept { return "step_wipetower_plugin"; }
// Copy fixed descriptors when the caller supplies enough array space; strings remain borrowed and no allocation occurs.
int32_t WipeTowerManager::used_config_keys(raw_used_config_key *keys) const noexcept
{
    const int32_t count = static_cast<int32_t>(sizeof(k_used_keys) / sizeof(k_used_keys[0]));
    if (keys != nullptr)
        std::copy(k_used_keys, k_used_keys + count, keys);
    return count;
}

// Borrow a valid whole-plan run, bound contour visits by original tool changes, reserve layouts and catalog only nonempty towers before moving generated roots into the plan; a request that printed something is bracketed by one speed-ratio save before its first leaf and one restore after its last, so its tool change runs at 100%; a floating request above the regular tower top is published on its own layer without maintenance; stopped dispatch throws silent PluginCancelled so the step rethrows the retained service cause, ordinary dispatch refusal throws runtime_error, and collector failures rethrow their retained exception; invalid input and allocation/insertion failures throw without transferring source ownership.
void WipeTowerManager::run_impl(const plugin_run_context *run) const
{
    const run_ctx_generate_wipe_tower *ctx = plugin_ctx_as_generate_wipe_tower(run);
    if (ctx == nullptr || ctx->print == nullptr || ctx->plan == nullptr)
        throw std::invalid_argument("Wipe tower manager requires a Print and PrintingPlan.");
    const Slic3r::Print &print = *reinterpret_cast<const Slic3r::Print *>(ctx->print);
    const std::vector<double> &nozzles = print.config().nozzle_diameter.get_values();
    const std::vector<Slic3r::WipeTowerFootprint> footprints = Slic3r::wipe_tower_footprints(print);
    if (footprints.empty() || ctx->wipe_plugin_count == 0)
        return;
    if (ctx->call_wipe_plugin == nullptr)
        throw std::invalid_argument("Wipe tower manager requires service dispatch.");
    PrintingPlan plan(ctx->plan);
    const ConstPrintingPlan source(plan);
    std::vector<Tower> towers;
    for (const Slic3r::WipeTowerFootprint &footprint : footprints) {
        Tower tower{footprint, footprint.print_object == nullptr ? &print.default_object_config() :
            &footprint.print_object->config(), UINT32_MAX, {}};
        for (uint32_t id = 0; id < source.object_instance_count(); ++id) {
            const PrintingObjectInstance owner = source.object_instance(id);
            if (!owner.has_object() || owner.object().handle() != reinterpret_cast<const object_handle *>(footprint.print_object))
                continue;
            const uint64_t index = owner.object_instance_idx();
            if (index < footprint.print_object->instances().size() &&
                footprint.print_object->instances()[size_t(index)].model_instance == footprint.instance)
                tower.owner_ids.push_back(id);
        }
        towers.push_back(std::move(tower));
    }
    add_contour_visits(plan, towers, m_forced_visit);
    RequestCollector collector;
    for (uint32_t idx = 0; idx < ctx->wipe_plugin_count; ++idx) {
        throw_if_cancelled(run);
        run_ctx_wipe_tower_wipe service = {};
        service.phase = C_WIPE_TOWER_EDIT;
        if (!ctx->call_wipe_plugin(ctx, idx, &service)) {
            // A stopped service has its cause retained by the step; only an ordinary refusal needs a manager error.
            throw_if_cancelled(run);
            throw std::runtime_error("Wipe service EDIT failed.");
        }
    }
    for (uint32_t idx = 0; idx < ctx->wipe_plugin_count; ++idx) {
        throw_if_cancelled(run);
        collector.plugin_index = idx;
        run_ctx_wipe_tower_wipe service = {};
        service.phase = C_WIPE_TOWER_COLLECT;
        service.report_context = &collector;
        service.report_request = &collect_request;
        if (!ctx->call_wipe_plugin(ctx, idx, &service)) {
            // A stopped service has its cause retained by the step; only an ordinary refusal needs a manager error.
            throw_if_cancelled(run);
            throw std::runtime_error("Wipe service COLLECT failed.");
        }
        if (collector.failure != nullptr)
            std::rethrow_exception(collector.failure);
    }
    for (CollectedRequest &collected : collector.requests) {
        const c_wipe_tower_request &request = collected.request;
        if ((!request.has_incoming && !request.has_outgoing) || !std::isfinite(request.purge_volume) || request.purge_volume < 0.)
            throw std::invalid_argument("Wipe request needs a part and a nonnegative finite volume.");
        if (request.has_incoming)
            validate_anchor(source, request.incoming, nozzles.size());
        if (request.has_outgoing)
            validate_anchor(source, request.outgoing, nozzles.size());
        if (request.has_incoming && request.has_outgoing &&
            (request.incoming.group_idx != request.outgoing.group_idx || request.incoming.layer_group_idx != request.outgoing.layer_group_idx))
            throw std::invalid_argument("Both wipe parts must share a layer group.");
        collected.tower_index = choose_tower(source, request, towers);
    }
    collector.requests.erase(std::remove_if(collector.requests.begin(), collector.requests.end(),
        [](const CollectedRequest &request) { return request.tower_index == no_tower; }), collector.requests.end());
    if (collector.requests.empty()) {
        clean_contour_visits(plan, m_forced_visit);
        return;
    }
    // The incoming visit defines a two-part request's order; equal anchors retain service collection order.
    std::stable_sort(collector.requests.begin(), collector.requests.end(), [](const CollectedRequest &left, const CollectedRequest &right) {
        const c_wipe_tower_anchor &a = left.request.has_incoming ? left.request.incoming : left.request.outgoing;
        const c_wipe_tower_anchor &b = right.request.has_incoming ? right.request.incoming : right.request.outgoing;
        return std::tie(a.group_idx, a.layer_group_idx, a.tool_group_idx, a.extrusion_idx) <
               std::tie(b.group_idx, b.layer_group_idx, b.tool_group_idx, b.extrusion_idx);
    });
    std::vector<std::vector<TowerLayer>> candidates(towers.size());
    std::vector<TowerLayout> layouts;
    for (uint32_t t = 0; t < towers.size(); ++t) {
        const Tower &tower = towers[t];
        candidates[t] = tower_layers(print, source, tower, t, collector.requests);
        std::vector<LayoutLayerInput> inputs;
        for (const TowerLayer &layer : candidates[t])
            inputs.push_back(layer.input);
        layouts.push_back(layout_tower(tower.footprint.width, tower.config->wipe_tower_extra_spacing.value / 100.,
            print.config().wipe_tower_no_sparse_layers.value, tower.config->wipe_tower_cone_angle.value, inputs));
    }
    for (uint32_t t = 0; t < towers.size(); ++t) {
        if (layouts[t].layers.empty())
            continue;
        Tower &tower = towers[t];
        std::string name = "Wipe tower";
        if (tower.footprint.instance != nullptr) {
            size_t number = 0;
            for (const Slic3r::ModelInstance *instance : tower.footprint.object->instances) {
                ++number;
                if (instance == tower.footprint.instance)
                    break;
            }
            name += " " + tower.footprint.object->name + " #" + std::to_string(number);
        }
        tower.catalog_id = plan.add_synthetic_object_instance(name,
            Polygon(reinterpret_cast<const polygon_handle *>(&tower.footprint.polygon)));
    }
    std::vector<GeneratedPart> parts;
    std::map<std::tuple<uint32_t, uint32_t, uint32_t, uint32_t>, c_vec2d> exits;
    // Dispatch in global plan order, not tower enumeration order; no source vector is mutated here.
    for (size_t r = 0; r < collector.requests.size(); ++r) {
        throw_if_cancelled(run);
        const CollectedRequest &collected = collector.requests[r];
        const c_wipe_tower_request &request = collected.request;
        const c_wipe_tower_anchor &anchor = request.has_incoming ? request.incoming : request.outgoing;
        const Tower &tower = towers[collected.tower_index];
        const LayoutLayer *layer = nullptr;
        const LayoutBand *reserved = nullptr;
        for (const LayoutLayer &candidate : layouts[collected.tower_index].layers)
            for (const LayoutBand &band : candidate.bands)
                if (band.request_index == r) {
                    layer = &candidate;
                    reserved = &band;
                }
        if (reserved == nullptr)
            throw std::runtime_error("A wipe request has no owner tower layer.");
        StoredExtrusionEntity outgoing(run->plugin_storage), incoming(run->plugin_storage);
        c_wipe_tower_band band = {};
        band.tower_catalog_id = tower.catalog_id;
        band.print_z = layer->print_z;
        band.layer_height = layer->height;
        band.min = {0., reserved->y0};
        band.max = {tower.footprint.width, reserved->y1};
        band.entry = {reserved->enter_right ? tower.footprint.width : 0., reserved->y0};
        band.extrusion_width = reserved->line_width;
        band.outgoing_root = request.has_outgoing ? outgoing.mutable_handle() : nullptr;
        band.incoming_root = request.has_incoming ? incoming.mutable_handle() : nullptr;
        band.line_spacing = reserved->line_spacing;
        const std::tuple<uint32_t, uint32_t, uint32_t, uint32_t> visit{collected.tower_index, anchor.group_idx, anchor.layer_group_idx, anchor.tool_group_idx};
        const std::map<std::tuple<uint32_t, uint32_t, uint32_t, uint32_t>, c_vec2d>::const_iterator prior = exits.find(visit);
        if (prior != exits.end())
            band.entry = prior->second;
        c_vec2d exit = band.entry;
        run_ctx_wipe_tower_wipe service = {};
        service.phase = C_WIPE_TOWER_GENERATE;
        service.request = &request;
        service.band = &band;
        service.exit = &exit;
        if (!ctx->call_wipe_plugin(ctx, collected.plugin_index, &service)) {
            // A stopped service has its cause retained by the step; only an ordinary refusal needs a manager error.
            throw_if_cancelled(run);
            throw std::runtime_error("Wipe service GENERATE failed.");
        }
        exits[visit] = exit;
        const bool outgoing_filled = request.has_outgoing && root_has_content(outgoing);
        const bool incoming_filled = request.has_incoming && root_has_content(incoming);
        if (outgoing_filled || incoming_filled) {
            // Exactly one speed-ratio save per request, before everything it prints. It is the
            // first leaf of the outgoing root, or a before event of the incoming visit when the
            // request has no outgoing part: CreateToolChange appends its T command at the end of
            // those events, so the save precedes the T too. A second save would make Marlin back
            // up the forced 100% ratio and lose the user's speed.
            if (request.has_outgoing)
                insert_speed_command(outgoing.mutable_view(), OrderedLeafPosition::Before,
                    C_EXTRUSION_SPECIAL_COMMAND_SAVE_AND_RESET_SPEED_RATIO, 1.);
            else
                append_visit_speed_command(run->plugin_storage,
                    plan.group(request.incoming.group_idx).layer_group(request.incoming.layer_group_idx)
                        .tool_group(request.incoming.tool_group_idx),
                    C_EXTRUSION_SPECIAL_COMMAND_SAVE_AND_RESET_SPEED_RATIO, 1.);
            // The restore closes the last printed part of the request: the incoming purge, or
            // the outgoing root of an outgoing-only request such as the final unload.
            insert_speed_command(request.has_incoming ? incoming.mutable_view() : outgoing.mutable_view(),
                OrderedLeafPosition::After, C_EXTRUSION_SPECIAL_COMMAND_RESTORE_SPEED_RATIO, 0.);
            // An enabled root that stayed empty receives only its command and is inserted like
            // any other generated part.
            if (request.has_outgoing)
                parts.push_back({request.outgoing, collected.tower_index, false, layer->print_z, std::move(outgoing)});
            if (request.has_incoming)
                parts.push_back({request.incoming, collected.tower_index, false, layer->print_z, std::move(incoming)});
        }
    }
    // Maintenance geometry is generated only after every service GENERATE call has completed;
    // a floating layer holds only its band and receives none.
    for (uint32_t t = 0; t < towers.size(); ++t)
        for (size_t l = 0; l < layouts[t].layers.size(); ++l) {
            throw_if_cancelled(run);
            if (layouts[t].layers[l].floating)
                continue;
            const TowerLayer &candidate = candidates[t][layouts[t].layers[l].source_index];
            const c_wipe_tower_anchor anchor = maintenance_anchor(source, towers[t], candidate, collector.requests, t);
            const extruder_index_t tool = source.group(anchor.group_idx).layer_group(anchor.layer_group_idx).tool_group(anchor.tool_group_idx).extruder_id();
            parts.push_back({anchor, t, true, layouts[t].layers[l].print_z,
                maintenance_root(run->plugin_storage, print, towers[t], layouts[t], l, tool)});
        }
    std::stable_sort(parts.begin(), parts.end(), [](const GeneratedPart &left, const GeneratedPart &right) {
        const c_wipe_tower_anchor &a = left.anchor, &b = right.anchor;
        return std::make_tuple(a.group_idx, a.layer_group_idx, a.tool_group_idx, a.extrusion_idx, !left.maintenance) <
               std::make_tuple(b.group_idx, b.layer_group_idx, b.tool_group_idx, b.extrusion_idx, !right.maintenance);
    });
    for (std::vector<GeneratedPart>::reverse_iterator it = parts.rbegin(); it != parts.rend(); ++it) {
        throw_if_cancelled(run);
        GeneratedPart &part = *it;
        const Tower &tower = towers[part.tower];
        transform_root(part.root.mutable_view(), Slic3r::Geometry::deg2rad(tower.footprint.rotation),
            {Slic3r::scale_i(tower.footprint.origin.x()), Slic3r::scale_i(tower.footprint.origin.y())});
        part.root.disable_sort().disable_reverse();
        PrintingGroup group = plan.group(part.anchor.group_idx);
        const coord_t z_offset = coord_t(std::llround(Slic3r::scale_d(part.print_z))) -
            group.layer_group(part.anchor.layer_group_idx).print_z();
        if (z_offset != 0)
            part.root.get_or_add(EPropertyZOffset::key).set(z_offset);
        bool listed = false;
        for (uint32_t id = 0; id < group.object_instance_count(); ++id)
            listed = listed || group.printing_catalog_id(id) == tower.catalog_id;
        if (!listed)
            group.append_printing_catalog_id(plan, tower.catalog_id);
        PrintingToolGroup tool = group.layer_group(part.anchor.layer_group_idx).tool_group(part.anchor.tool_group_idx);
        const uint32_t appended = tool.extrusion_count();
        PrintingExtrusion extrusion(printing_tool_group_append_extrusion_move(plan.handle(), tool.mutable_handle(), nullptr,
            RAW_EXTRUSION_ROLE_WIPE_TOWER, part.root.mutable_handle(), tower.catalog_id));
        if (!extrusion.valid() || !tool.move_extrusion(appended, part.anchor.extrusion_idx))
            throw std::runtime_error("Cannot insert generated tower extrusion.");
    }
    clean_contour_visits(plan, m_forced_visit);
}

// Copy a nonnull reported value without lending vector memory; return zero on invalid arguments/allocation failure and retain the exception for the manager.
int32_t collect_request(void *context, const c_wipe_tower_request *request)
{
    if (context == nullptr || request == nullptr)
        return 0;
    RequestCollector &collector = *static_cast<RequestCollector *>(context);
    try {
        collector.requests.push_back({collector.plugin_index, *request, no_tower});
        return 1;
    } catch (...) {
        collector.failure = std::current_exception();
        return 0;
    }
}

// Resolve borrowed anchor indices including end positions against configured concrete tools; mutate nothing and throw for out-of-range input.
void validate_anchor(const ConstPrintingPlan &plan, const c_wipe_tower_anchor &anchor, const size_t extruder_count)
{
    if (anchor.group_idx >= plan.group_count())
        throw std::invalid_argument("Wipe anchor group index is out of range.");
    const ConstPrintingGroup group = plan.group(anchor.group_idx);
    if (anchor.layer_group_idx >= group.layer_group_count())
        throw std::invalid_argument("Wipe anchor layer index is out of range.");
    const ConstPrintingLayerGroup layer = group.layer_group(anchor.layer_group_idx);
    if (anchor.tool_group_idx >= layer.tool_group_count())
        throw std::invalid_argument("Wipe anchor tool index is out of range.");
    const ConstPrintingToolGroup tool = layer.tool_group(anchor.tool_group_idx);
    if (anchor.extrusion_idx > tool.extrusion_count() || tool.extruder_id() < 0 || size_t(tool.extruder_id()) >= extruder_count)
        throw std::invalid_argument("Wipe anchor extrusion index or extruder is invalid.");
}

// Search borrowed layer extrusions for an owner id, updating only the caller's tool/extrusion indices; global towers use a nonempty visit list's start and missing owners return false.
bool first_owner_anchor(const ConstPrintingLayerGroup &layer, const Tower &tower, c_wipe_tower_anchor &anchor)
{
    if (tower.footprint.instance == nullptr && layer.tool_group_count() != 0) {
        anchor.tool_group_idx = anchor.extrusion_idx = 0;
        return true;
    }
    for (uint32_t t = 0; t < layer.tool_group_count(); ++t)
        for (uint32_t e = 0; e < layer.tool_group(t).extrusion_count(); ++e)
            if (std::find(tower.owner_ids.begin(), tower.owner_ids.end(), layer.tool_group(t).extrusion(e).printing_catalog_id()) != tower.owner_ids.end()) {
                anchor.tool_group_idx = t;
                anchor.extrusion_idx = e;
                return true;
            }
    return false;
}

// Inspect the incoming visit's first extrusion, or the outgoing anchor's next extrusion across this layer then its previous one; return an instance/global tower index or no_tower without mutating borrowed input.
uint32_t choose_tower(const ConstPrintingPlan &plan, const c_wipe_tower_request &request, const std::vector<Tower> &towers)
{
    const c_wipe_tower_anchor &anchor = request.has_incoming ? request.incoming : request.outgoing;
    const ConstPrintingLayerGroup layer = plan.group(anchor.group_idx).layer_group(anchor.layer_group_idx);
    uint32_t owner = UINT32_MAX;
    bool found = false;
    const uint32_t start = request.has_incoming ? 0 : anchor.extrusion_idx;
    for (uint32_t t = anchor.tool_group_idx; t < layer.tool_group_count() && !found; ++t) {
        const ConstPrintingToolGroup tool = layer.tool_group(t);
        const uint32_t e = t == anchor.tool_group_idx ? start : 0;
        if (e < tool.extrusion_count()) {
            owner = tool.extrusion(e).printing_catalog_id();
            found = true;
        }
    }
    if (!found)
        for (uint32_t t = anchor.tool_group_idx + 1; t > 0 && !found; --t) {
            const ConstPrintingToolGroup tool = layer.tool_group(t - 1);
            const uint32_t end = t - 1 == anchor.tool_group_idx ? start : tool.extrusion_count();
            if (end > 0) {
                owner = tool.extrusion(end - 1).printing_catalog_id();
                found = true;
            }
        }
    uint32_t global = no_tower;
    for (uint32_t t = 0; t < towers.size(); ++t) {
        if (towers[t].footprint.instance == nullptr)
            global = t;
        if (std::find(towers[t].owner_ids.begin(), towers[t].owner_ids.end(), owner) != towers[t].owner_ids.end())
            return t;
    }
    return global;
}

// Scan borrowed original visits for each owner's last tool change, then insert absent contour tools only through that layer at minimum transition cost, ties at layer start; markers follow moves, source ownership is unchanged and allocation/insertion failures throw.
void add_contour_visits(PrintingPlan &plan, const std::vector<Tower> &towers, const PluginPropertyKey<ForcedVisit> &marker)
{
    const std::pair<uint32_t, uint32_t> absent{UINT32_MAX, UINT32_MAX};
    std::vector<std::pair<uint32_t, uint32_t>> last_change(towers.size(), absent);
    extruder_index_t previous = EXTRUDER_ID_UNKNOWN;
    // Determine every boundary before inserting anything, so a forced visit can
    // never extend its own tower's range or another tower's range.
    for (uint32_t g = 0; g < plan.group_count(); ++g)
        for (uint32_t l = 0; l < plan.group(g).layer_group_count(); ++l) {
            const ConstPrintingLayerGroup layer = plan.group(g).layer_group(l);
            bool changed = false;
            for (uint32_t t = 0; t < layer.tool_group_count(); ++t) {
                const extruder_index_t tool = layer.tool_group(t).extruder_id();
                changed = changed || (previous != EXTRUDER_ID_UNKNOWN && previous != tool);
                previous = tool;
            }
            if (changed)
                for (uint32_t t = 0; t < towers.size(); ++t) {
                    c_wipe_tower_anchor owner{g, l, 0, 0};
                    if (first_owner_anchor(layer, towers[t], owner))
                        last_change[t] = {g, l};
                }
        }
    previous = EXTRUDER_ID_UNKNOWN;
    for (uint32_t g = 0; g < plan.group_count(); ++g)
        for (uint32_t l = 0; l < plan.group(g).layer_group_count(); ++l) {
            PrintingLayerGroup layer = plan.group(g).layer_group(l);
            for (uint32_t t = 0; t < towers.size(); ++t) {
                const Tower &tower = towers[t];
                c_wipe_tower_anchor owner{g, l, 0, 0};
                if (last_change[t] == absent || std::make_pair(g, l) > last_change[t] ||
                    !first_owner_anchor(ConstPrintingLayerGroup(layer), tower, owner) ||
                    tower.config->wipe_tower_extruder.value == 0 || !tower.config->wipe_tower_extruder_toolchange.value)
                    continue;
                const extruder_index_t requested = extruder_index_t(tower.config->wipe_tower_extruder.value - 1);
                bool exists = false;
                for (uint32_t t = 0; t < layer.tool_group_count(); ++t)
                    exists = exists || layer.tool_group(t).extruder_id() == requested;
                if (exists)
                    continue;
                extruder_index_t next = EXTRUDER_ID_UNKNOWN;
                for (uint32_t ng = g; ng < plan.group_count() && next == EXTRUDER_ID_UNKNOWN; ++ng)
                    for (uint32_t nl = ng == g ? l + 1 : 0; nl < plan.group(ng).layer_group_count() && next == EXTRUDER_ID_UNKNOWN; ++nl)
                        if (plan.group(ng).layer_group(nl).tool_group_count() != 0)
                            next = plan.group(ng).layer_group(nl).tool_group(0).extruder_id();
                uint32_t best = 0;
                int best_cost = std::numeric_limits<int>::max();
                for (uint32_t pos = 0; pos <= layer.tool_group_count(); ++pos) {
                    const extruder_index_t left = pos == 0 ? previous : layer.tool_group(pos - 1).extruder_id();
                    const extruder_index_t right = pos == layer.tool_group_count() ? next : layer.tool_group(pos).extruder_id();
                    const int cost = int(left != EXTRUDER_ID_UNKNOWN && left != requested) +
                        int(right != EXTRUDER_ID_UNKNOWN && right != requested) -
                        int(left != EXTRUDER_ID_UNKNOWN && right != EXTRUDER_ID_UNKNOWN && left != right);
                    if (cost < best_cost) {
                        best_cost = cost;
                        best = pos;
                    }
                }
                const uint32_t appended = layer.tool_group_count();
                PrintingToolGroup forced = layer.append_tool_group(requested);
                forced.properties().get_or_add(marker).present = 1;
                if (!layer.move_tool_group(appended, best))
                    throw std::runtime_error("Cannot insert a contour tool visit.");
            }
            if (layer.tool_group_count() != 0)
                previous = layer.tool_group(layer.tool_group_count() - 1).extruder_id();
        }
}

// Remove the private marker from all borrowed visits then delete only marked visits that are otherwise empty; source/events/other properties are preserved and no pending insertion anchors exist.
void clean_contour_visits(PrintingPlan &plan, const PluginPropertyKey<ForcedVisit> &marker)
{
    for (uint32_t g = 0; g < plan.group_count(); ++g)
        for (uint32_t l = 0; l < plan.group(g).layer_group_count(); ++l) {
            PrintingLayerGroup layer = plan.group(g).layer_group(l);
            for (uint32_t t = layer.tool_group_count(); t > 0; --t) {
                PrintingToolGroup tool = layer.tool_group(t - 1);
                if (tool.properties().remove(marker))
                    layer.remove_empty_tool_group(t - 1);
            }
        }
}

// Build print-ordered owned inputs from borrowed source layers using their minimum height and owner membership; band inputs keep each request's floating flag; involved tools share the smallest resolved sparse limit, missing source heights/config/allocation failures throw without changing ownership.
std::vector<TowerLayer> tower_layers(const Slic3r::Print &print, const ConstPrintingPlan &plan, const Tower &tower,
                                    const uint32_t tower_idx, const std::vector<CollectedRequest> &requests)
{
    std::vector<TowerLayer> layers;
    double max_height = std::numeric_limits<double>::max();
    for (uint32_t g = 0; g < plan.group_count(); ++g)
        for (uint32_t l = 0; l < plan.group(g).layer_group_count(); ++l) {
            const ConstPrintingLayerGroup source = plan.group(g).layer_group(l);
            c_wipe_tower_anchor owner{g, l, 0, 0};
            if (!first_owner_anchor(source, tower, owner))
                continue;
            double height = std::numeric_limits<double>::max();
            for (uint32_t i = 0; i < source.layer_count(); ++i)
                height = std::min(height, unscaled(source.layer(i).height()));
            if (source.layer_count() == 0)
                throw std::invalid_argument("Tower owner layer group has no source layer height.");
            TowerLayer layer{g, l, owner, {unscaled(source.print_z()), height, 0., {}}};
            for (uint32_t tool = 0; tool < source.tool_group_count(); ++tool) {
                const extruder_index_t extruder = source.tool_group(tool).extruder_id();
                max_height = std::min(max_height, Slic3r::Slicing::max_layer_height_from_nozzle(print.full_print_config(), uint16_t(extruder)));
            }
            for (size_t r = 0; r < requests.size(); ++r) {
                const CollectedRequest &request = requests[r];
                const c_wipe_tower_anchor &anchor = request.request.has_incoming ? request.request.incoming : request.request.outgoing;
                if (request.tower_index != tower_idx || anchor.group_idx != g || anchor.layer_group_idx != l)
                    continue;
                const extruder_index_t extruder = source.tool_group(anchor.tool_group_idx).extruder_id();
                const double nozzle = print.config().nozzle_diameter.get_at(extruder);
                const double width = tower.config->wipe_tower_extrusion_width.get_effective_value(nozzle);
                layer.input.bands.push_back({r, request.request.purge_volume, width > 0. ? width : nozzle, nozzle,
                    request.request.floating != 0});
            }
            layers.push_back(std::move(layer));
        }
    for (TowerLayer &layer : layers)
        layer.input.max_height = max_height;
    return layers;
}

// Return true when a service filled an owned root with points, children or properties; a root
// that stayed empty must not make its request look printable.
bool root_has_content(const StoredExtrusionEntity &root)
{
    return root.has_local_points() || root.child_count() != 0 || root.property_count() != 0;
}

// Insert one fixed point-less special command before or after the current content of a generated
// root. The command carries no points, so the firmware emits it where the nozzle stands. The root
// and its storage stay caller-owned; an insertion or property failure throws.
void insert_speed_command(MutableExtrusionEntity root, const OrderedLeafPosition position,
                          const c_extrusion_special_command code, const double value)
{
    MutableExtrusionEntity command =
        root.emplace_ordered_leaf(position, ExistingPropertyPlacement::KeepOnParent);
    if (!command.valid())
        throw std::runtime_error("Cannot insert a tower speed command.");
    command.get_or_add(EPropertySpecialCommand::key).set(code, value);
    command.disable_sort().disable_reverse();
}

// Append one fixed point-less special command to the before events of a borrowed tool visit; the
// event tree stays host-owned and a failed append throws.
void append_visit_speed_command(storage_handle *storage, PrintingToolGroup tool,
                                const c_extrusion_special_command code, const double value)
{
    StoredExtrusionEntity command(storage);
    command.set_can_rotate(false);
    command.get_or_add(EPropertySpecialCommand::key).set(code, value);
    command.disable_sort().disable_reverse();
    if (!tool.events().append_before_move(command.mutable_view()).valid())
        throw std::runtime_error("Cannot append a tower speed command to the incoming visit.");
}

// Select the earliest enabled tower part in this borrowed layer, otherwise the owner start; prefer the first configured contour-tool visit, falling back only when that tool is absent, without modifying the plan.
c_wipe_tower_anchor maintenance_anchor(const ConstPrintingPlan &plan, const Tower &tower, const TowerLayer &layer,
                                       const std::vector<CollectedRequest> &requests, const uint32_t tower_idx)
{
    c_wipe_tower_anchor first = layer.first_owner;
    bool has_band = false;
    for (const CollectedRequest &request : requests) {
        if (request.tower_index != tower_idx)
            continue;
        for (const std::pair<int32_t, c_wipe_tower_anchor> &part : {std::make_pair(request.request.has_outgoing, request.request.outgoing),
                                                               std::make_pair(request.request.has_incoming, request.request.incoming)}) {
            const c_wipe_tower_anchor &a = part.second;
            if (part.first && a.group_idx == layer.group && a.layer_group_idx == layer.layer &&
                (!has_band || std::tie(a.tool_group_idx, a.extrusion_idx) < std::tie(first.tool_group_idx, first.extrusion_idx))) {
                first = a;
                has_band = true;
            }
        }
    }
    if (tower.config->wipe_tower_extruder.value > 0) {
        const ConstPrintingLayerGroup source = plan.group(layer.group).layer_group(layer.layer);
        for (uint32_t t = 0; t < source.tool_group_count(); ++t)
            if (source.tool_group(t).extruder_id() == tower.config->wipe_tower_extruder.value - 1)
                return {layer.group, layer.layer, t, 0};
    }
    return first;
}

// Generate local brim/perimeter/cone/fill into an owned root using resolved positive flow and explicit speed; first-layer speed is capped then floored like object paths, source owners remain borrowed and geometry/allocation failures propagate.
StoredExtrusionEntity maintenance_root(storage_handle *storage, const Slic3r::Print &print, const Tower &tower,
                                       const TowerLayout &layout, const size_t layer_idx, const extruder_index_t extruder)
{
    const LayoutLayer &layer = layout.layers[layer_idx];
    const double nozzle = print.config().nozzle_diameter.get_at(extruder);
    const double requested_width = tower.config->wipe_tower_extrusion_width.get_effective_value(nozzle);
    const Slic3r::Flow flow = Slic3r::Flow::new_from_width(float(requested_width > 0. ? requested_width : nozzle),
        float(nozzle), float(layer.height), 1.f, false);
    double speed = print.config().wipe_tower_speed.value > 0. ? print.config().wipe_tower_speed.value : 80.;
    if (layer_idx == 0) {
        const Slic3r::ConfigBase &overrides = tower.footprint.object == nullptr ?
            static_cast<const Slic3r::ConfigBase &>(print.full_print_config()) : tower.footprint.object->config.get();
        const Slic3r::ConfigOptionFloatOrPercent *first = overrides.opt<Slic3r::ConfigOptionFloatOrPercent>("first_layer_speed");
        if (first == nullptr)
            first = print.full_print_config().opt<Slic3r::ConfigOptionFloatOrPercent>("first_layer_speed");
        if (first != nullptr && first->value > 0.)
            speed = std::min(speed, first->get_effective_value(speed));
        const Slic3r::ConfigOptionFloat *minimum = overrides.opt<Slic3r::ConfigOptionFloat>("first_layer_min_speed");
        if (minimum == nullptr)
            minimum = print.full_print_config().opt<Slic3r::ConfigOptionFloat>("first_layer_min_speed");
        if (minimum != nullptr)
            speed = std::max(speed, minimum->value);
    }
    StoredExtrusionEntity root(storage);
    root.set_can_rotate(false);
    const Slic3r::Polygon rectangle({Slic3r::Point::new_scale(0., 0.), Slic3r::Point::new_scale(tower.footprint.width, 0.),
        Slic3r::Point::new_scale(tower.footprint.width, layer.depth), Slic3r::Point::new_scale(0., layer.depth)});
    Slic3r::Polygon envelope = rectangle;
    const double radius = layout.cone_base_radius + layer.depth / 2. - layout.cone_slope * layer.print_z;
    if (layout.cone_base_radius > 0.) {
        Slic3r::Points points = rectangle.points;
        for (size_t n = 0; n < 80; ++n) {
            const double angle = 2. * PI * double(n) / 80.;
            points.push_back(Slic3r::Point::new_scale(tower.footprint.width / 2. + radius * std::cos(angle) / layout.cone_scale,
                layer.depth / 2. + radius * std::sin(angle)));
        }
        envelope = Slic3r::Geometry::convex_hull(points);
    }
    if (layer_idx == 0 && tower.footprint.brim_width > 0.) {
        const size_t loops = size_t((tower.footprint.brim_width + flow.spacing() / 2.) / flow.spacing());
        for (size_t i = loops; i > 0; --i)
            for (const Slic3r::Polygon &polygon : Slic3r::offset(envelope, Slic3r::scale_d(double(i) * flow.spacing()))) {
                Slic3r::Points points = polygon.points;
                points.push_back(points.front());
                append_path(storage, root.mutable_view(), points, flow, speed);
            }
    }
    Slic3r::Points perimeter = rectangle.points;
    perimeter.push_back(perimeter.front());
    append_path(storage, root.mutable_view(), perimeter, flow, speed);
    if (envelope.area() > rectangle.area() + 1.) {
        Slic3r::Points ring = envelope.points;
        ring.push_back(ring.front());
        append_path(storage, root.mutable_view(), ring, flow, speed);
    }
    Slic3r::ExPolygons fill = Slic3r::offset_ex(envelope, -flow.scaled_spacing() / 3.);
    if (layer.used > 0.) {
        const Slic3r::Polygon used({Slic3r::Point::new_scale(0., 0.), Slic3r::Point::new_scale(tower.footprint.width, 0.),
            Slic3r::Point::new_scale(tower.footprint.width, layer.used), Slic3r::Point::new_scale(0., layer.used)});
        fill = Slic3r::diff_ex(fill, Slic3r::Polygons{used});
    }
    raw_infill_pattern_params params = {};
    params.extrusion_role = RAW_EXTRUSION_ROLE_WIPE_TOWER;
    params.flow.width = flow.scaled_width();
    params.flow.height = flow.scaled_height();
    params.flow.spacing = flow.scaled_spacing();
    params.flow.nozzle_diameter = Slic3r::scale_i(nozzle);
    params.flow.spacing_ratio = 1.;
    params.flow.mm3_per_mm = flow.mm3_per_mm();
    params.flow_mult = 1.f;
    params.connection = C_INFILL_CONNECTION_CONNECTED;
    params.angle = PI / 4.;
    params.bridge_angle = -1.;
    params.layer_id = uint32_t(layer_idx);
    params.layer_height = layer.height;
    params.z = layer.print_z;
    params.fill_resolution = Slic3r::scale_i(0.01);
    // The fill pattern is centred on the tower rectangle; a cone envelope may extend beyond it.
    params.object_bounding_box = {{0, 0}, {Slic3r::scale_i(tower.footprint.width), Slic3r::scale_i(layout.layers.front().depth)}};
    params.spacing = flow.spacing();
    params.density = 1.f;
    if (layer_idx != 0) {
        // Grid divides density between two sweeps. Limit cell diagonals, not just their perpendicular pitch.
        const double nominal_pitch = 20. * flow.spacing() * tower.config->wipe_tower_extra_spacing.value / 100.;
        const double pitch = tower.config->wipe_tower_bridging.value > 0. ?
            std::min(nominal_pitch, tower.config->wipe_tower_bridging.value / std::sqrt(2.)) : nominal_pitch;
        params.density = float(std::min(1., 2. * flow.spacing() / pitch));
    }
    raster::FillMonotonic solid;
    raster::FillGrid sparse;
    raster::FillRectilinear &filler = layer_idx == 0 ? static_cast<raster::FillRectilinear &>(solid) : sparse;
    raster::configure_filler(filler, params, storage);
    for (const Slic3r::ExPolygon &polygon : fill) {
        const PluginCommon::polygon_work::Polylines lines = filler.fill_surface(
            ExPolygon(reinterpret_cast<const expolygon_handle *>(&polygon)), params);
        for (const PluginCommon::polygon_work::Polyline &line : lines) {
            Slic3r::Points points;
            for (const PluginCommon::polygon_work::Point &point : line.points)
                points.emplace_back(point.x(), point.y());
            if (points.size() >= 2)
                append_path(storage, root.mutable_view(), points, flow, speed);
        }
    }
    root.disable_sort().disable_reverse();
    return root;
}

// Copy at least two scaled local points into a child owned by the root, setting rounded-flow width/height/mm3 per mm and positive speed; invalid storage or append/allocation failures throw.
void append_path(storage_handle *storage, MutableExtrusionEntity root, const Slic3r::Points &points,
                  const Slic3r::Flow &flow, const double speed)
{
    StoredExtrusionEntity path(storage);
    std::vector<c_point> converted;
    for (const Slic3r::Point &point : points)
        converted.push_back({point.x(), point.y()});
    path.set_points(converted);
    path.get_or_add(EPropertyAttributes::key).extrusion_role(RAW_EXTRUSION_ROLE_WIPE_TOWER)
        .width(flow.width()).height(flow.height()).mm3_per_mm(flow.mm3_per_mm());
    path.get_or_add(EPropertySpeed::key).speed_mm_per_s = float(speed);
    path.disable_sort().disable_reverse();
    if (root.append_child_move(path.mutable_view()) == UINT32_MAX)
        throw std::runtime_error("Cannot append tower maintenance path.");
}

// Rotate then translate borrowed local polylines recursively in scaled coordinates; ownership is unchanged and failed mutations throw before insertion.
void transform_root(MutableExtrusionEntity root, const double angle, const c_point origin)
{
    if (root.has_local_points() && (!root.rotate(angle) || !root.translate(origin)))
        throw std::runtime_error("Cannot transform tower extrusion to platter coordinates.");
    for (uint32_t idx = 0; idx < root.child_count(); ++idx)
        transform_root(root.child_mutable(idx), angle, origin);
}
} // namespace

// Keep the plugin alive for every host callback; registration borrows its C instance and registration failures are reported by the host.
void register_wipe_tower_manager_plugin(orchestrator_handle *orchestrator)
{
    static WipeTowerManager manager(orchestrator);
    orchestrator_register_plugin(orchestrator, manager.c_instance());
}
} // namespace slic3r_api::WipeTower
