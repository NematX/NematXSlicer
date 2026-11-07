///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ Copyright (c) NematX 2026
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
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

#include "FilamentSections.hpp"
#include "TowerFlow.hpp"
#include "TowerFootprints.hpp"
#include "WipeTowerLayout.hpp"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_plugin.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_wipetower.h"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Api/plugin/cpp/LayerHeightLimits.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PluginPropertyKey.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "rectilinear/RectilinearFills.hpp"

/* NematX wipe tower manager: the exclusive STEP_WIPETOWER plugin of the
nematx-wipetower package, selected by step_wipetower_plugin = nematx_wipetower.

It does what the host's default manager (wipe_tower.manager.default) does, with
the plugin SDK only, so that the NematX tower variants can be added to it. The
host lends the wipe services (WIPE_TOWER_WIPE, such as the tool-change purge)
through run_ctx_generate_wipe_tower; the contract of both steps is in
slic3r_step_wipetower.h and slic3r_step_wipe_tower_wipe.h.

The run has separate phases because each one needs the result of the previous
one for the whole print:
1. Towers come from the settings (TowerFootprints.hpp). Each one gets the plan
   object instances that own it.
2. EDIT: the manager adds forced visits of the configured contour tool, only up
   to each owner's last tool-change layer of the unmodified plan, then every
   service may edit the plan.
3. COLLECT: every service reports its purge requests, anchored in the final plan.
4. Layout (WipeTowerLayout.hpp): bands and tower layers, per tower, in print order.
   Only towers with printed layers get a synthetic catalog entry.
5. GENERATE: each request's service fills its band, in plan order. Every
   request that printed something is bracketed by one speed-ratio save and
   restore, so that the whole tool change, its T command included, runs at 100%.
6. Maintenance: brim, perimeters, cone and fill of the unused area of every
   non-floating tower layer, with the host's Monotonic and Grid fills
   (rectilinear/, a reviewed copy). wipe_tower_perimeters, a setting of this
   package, gives the number of perimeters: 1 is the default manager's band
   rectangle and cone ring, each further one is one line spacing outside, and 0
   prints none. The host's tower footprint (collision check, preview) does not
   include these further perimeters. The fill is dense on the first layer, and
   on every layer with wipe_tower_only_solid, another setting of this package;
   otherwise it is a sparse grid.
7. Insertion: every generated root is rotated, moved to the platter and inserted
   by descending anchor, so that pending anchors stay valid; then the forced
   visits that stayed empty are removed.
A floating request is laid out with the regular bands when the tower already
reaches its Z; above the top it gets its own layer, without maintenance, which
changes neither the depth nor the cone.

wipe_tower_separate_filament, a setting of this package, gives each tool its own
section of the tower depth (FilamentSections.hpp), so that incompatible
materials never need to stick together:
- it is refused with a single-extruder multi-material printer and with
  filament_multitool_ramming, whose outgoing filament would print in the section
  of the incoming one;
- a tool's section lasts from the first tower layer until the last layer where
  the tool prints something of the tower's owners, and at most until the last
  tool change; in step 2, every tool with an active section that is absent from
  a layer gets a forced visit there, instead of the contour tool;
- in step 4 the layout decides the tower layers, without cone, and each band then
  moves into the section of its tool;
- in step 6 each active section gets its own maintenance, printed by its tool
  after its bands: perimeters around the section and fill of its unused part;
  the first section of the first layer also prints the brim around all of them.

Geometry is built in the tower's local frame (X in [0, width], Y toward the open
side) in scaled coordinates; all roots and the plan stay owned by the host, and
temporary geometry lives in the plugin storage of the run.

    run_impl
        towers_of_print                   tower settings and position
        check_separate_filament           refusals of separate sections
        add_forced_visits                 section ends, forced visits (EDIT)
        services: EDIT, COLLECT           collect_request, validate_anchor, choose_tower
        tower_layers, layout_tower        bands and tower layers
            place_sections                bands moved into the sections of their tools
        services: GENERATE                one band per request, speed save and restore
            insert_speed_command, append_visit_speed_command
        maintenance_anchor, maintenance_root          shared tower
        append_section_maintenance, maintenance_root  separate sections
            append_path
        transform_root, insertion          descending anchors
        clean_forced_visits */

namespace nematx_wipetower {
namespace {
using namespace slic3r_api;
namespace raster = NematXWipeTowerCopy::RectilinearPatternsPlugin::fills;
namespace raster_work = NematXWipeTowerCopy::RectilinearPatternsPlugin::work;

constexpr const char *k_plugin_id = "nematx_wipetower";
constexpr const char *k_perimeters_key = "wipe_tower_perimeters";
constexpr const char *k_separate_key = "wipe_tower_separate_filament";
constexpr const char *k_only_solid_key = "wipe_tower_only_solid";
const uint32_t no_tower = UINT32_MAX;
const char *const k_no_dependencies[] = { nullptr };
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
    {"spiral_vase", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"single_extruder_multi_material", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_multitool_ramming", RAW_CO_VECTOR_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {k_perimeters_key, RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {k_separate_key, RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {k_only_solid_key, RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
};
// Settings defined by this package; their values are object settings, like the other wipe tower settings.
const char *const k_defined_keys[] = { k_perimeters_key, k_separate_key, k_only_solid_key };

struct ForcedVisit { uint32_t present; };
// A layer group of the plan: group index, then layer group index; pairs compare in print order.
using PlanLayer = std::pair<uint32_t, uint32_t>;
struct Tower {
    TowerFootprint footprint;
    uint32_t catalog_id = UINT32_MAX;
    std::vector<uint32_t> owner_ids;
    // wipe_tower_separate_filament: one section per tool.
    bool separate = false;
    // Last layer of each tool's section: its last use in the tower, at most the last tool change.
    std::map<extruder_index_t, PlanLayer> section_ends;
    // Section places, planned once the bands are laid out.
    SectionLayout sections;
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
// Order of the parts that share an anchor: a shared tower's maintenance comes before the bands,
// a section's maintenance after the bands of its tool.
enum class PartOrder { SharedMaintenance, Band, SectionMaintenance };
struct GeneratedPart {
    c_wipe_tower_anchor anchor;
    uint32_t tower;
    PartOrder order;
    double print_z;
    StoredExtrusionEntity root;
};
// Area of one maintenance root, in local mm.
struct MaintenanceArea {
    // Band rectangle along local Y, and the top of the bands printed in it on this layer.
    double y0;
    double y1;
    double used_y1;
    // Add the cone envelope of the layout; only a shared tower has one.
    bool cone;
    // Print the first-layer brim; without the cone, it surrounds [brim_y0, brim_y1].
    bool brim;
    double brim_y0;
    double brim_y1;
    // Depth of the whole tower, which places the fill pattern the same way in every area.
    double pattern_y1;
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
    // Return the description shown with the selector.
    const char *description_impl() const noexcept override;
    // Select the whole-plan wipe-tower producer step.
    slicing_step_t step_impl() const noexcept override;
    // Declare no plugin dependencies.
    const char *const *dependencies_impl() const noexcept override;
    // Use the default manager priority.
    int32_t priority_impl() const noexcept override;
    // Join the wipe-tower step's exclusive selector.
    const char *exclusive_group_impl() const noexcept override;
    // Rank after the host's default manager, so that only an explicit choice selects this one.
    int32_t exclusive_group_priority_impl() const noexcept override;
    // Copy typed configuration dependencies into the optional caller array.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    // Copy the keys of the settings defined by this package into the optional caller array.
    int32_t defined_config_keys(const char **keys) const noexcept override;
    // Define the settings of this package.
    void inilialize_impl(storage_handle *storage) const override;
    // Add forced visits, dispatch services, lay out all towers and publish generated geometry.
    void run_impl(const plugin_run_context *run) const override;
    PluginPropertyKey<ForcedVisit> m_forced_visit;
};

// Define one object setting of this package in the wipe tower settings; a refused definition throws.
void define_tower_option(orchestrator_handle *orchestrator, const char *key, raw_config_option_type type,
                         const char *label, const char *tooltip, const char *default_value);
// Copy a service request into manager-owned storage and retain reporter failures.
int32_t collect_request(void *context, const c_wipe_tower_request *request);
// Check post-EDIT anchor bounds and concrete configured tools.
void validate_anchor(const ConstPrintingPlan &plan, const c_wipe_tower_anchor &anchor, size_t extruder_count);
// Find an owner's first extrusion in a layer, or the global layer-start anchor.
bool first_owner_anchor(const ConstPrintingLayerGroup &layer, const Tower &tower, c_wipe_tower_anchor &anchor);
// Map the following extrusion to a tower, preferring its instance over the global tower.
uint32_t choose_tower(const ConstPrintingPlan &plan, const c_wipe_tower_request &request,
                      const std::vector<Tower> &towers);
// Refuse separate sections with the SEMM printers and the rammed tools: their outgoing filament
// would print in the section of the incoming one.
void check_separate_filament(const Config &print_config, const ConstPrintingPlan &plan);
// Before service EDIT, record each separate tower's section ends, then add marked visits through
// each owner's last unmodified tool-change layer: the contour tool of a shared tower, every tool
// with an active section of a separate tower.
void add_forced_visits(PrintingPlan &plan, std::vector<Tower> &towers, const PluginPropertyKey<ForcedVisit> &marker);
// Remove only marked visits that remain otherwise empty and erase all manager markers.
void clean_forced_visits(PrintingPlan &plan, const PluginPropertyKey<ForcedVisit> &marker);
// Return the largest layer height of a tool in mm, as Slic3r::Slicing::max_layer_height_from_nozzle().
double max_layer_height_from_nozzle(const Config &print_config, extruder_index_t extruder);
// Round a height in mm to z_step, as Slic3r::check_z_step(double, double).
double check_z_step(double value, double z_step);
// Scan an owner's layer candidates and attach ordered requests and their resolved widths.
std::vector<TowerLayer> tower_layers(const Config &print_config, const ConstPrintingPlan &plan,
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
// Plan the sections of a separate tower and move each band of its layout into the section of its tool.
void place_sections(Tower &tower, TowerLayout &layout, const ConstPrintingPlan &plan,
                    const std::vector<CollectedRequest> &requests);
// Append one maintenance part per active section of each layer of a separate tower, after its tool's bands.
void append_section_maintenance(std::vector<GeneratedPart> &parts, storage_handle *storage, const Config &print_config,
                                const ConstPrintingPlan &plan, const Tower &tower, uint32_t tower_idx,
                                const TowerLayout &layout, const std::vector<TowerLayer> &candidates,
                                const std::vector<CollectedRequest> &requests);
// Append complete local maintenance geometry of one area using the copied Monotonic/Grid generators.
StoredExtrusionEntity maintenance_root(storage_handle *storage, const Config &print_config,
                                       const Tower &tower, const TowerLayout &layout, size_t layer_idx,
                                       extruder_index_t extruder, const MaintenanceArea &area);
// Return the tool of a request: the incoming tool, or the outgoing one of an outgoing-only request.
extruder_index_t request_tool(const ConstPrintingPlan &plan, const c_wipe_tower_request &request);
// Append one scaled local path with complete flow and speed attributes to an owned root.
void append_path(storage_handle *storage, MutableExtrusionEntity root, const std::vector<c_point> &points,
                 const TowerFlow &flow, double speed);
// Rotate then translate every local polyline recursively before ownership moves into the plan.
void transform_root(MutableExtrusionEntity root, double angle, c_point origin);
// Return a scaled point of a point in mm, rounded like Slic3r::Point::new_scale().
c_point new_scale(double x, double y);
// Return a storage-owned polygon with the given points.
StoredPolygon make_polygon(storage_handle *storage, const std::vector<c_point> &points);

// Initialize borrowed plugin metadata and a movable tool marker; registration/layout failures propagate.
WipeTowerManager::WipeTowerManager(orchestrator_handle *orchestrator) : PluginBase(orchestrator),
    m_forced_visit(PluginPropertyKey<ForcedVisit>::register_dynamic(orchestrator, "nematx_wipetower.forced_visit")) {}
const char *WipeTowerManager::id_impl() const noexcept { return k_plugin_id; }
const char *WipeTowerManager::name_impl() const noexcept { return "NematX wipe tower manager"; }
const char *WipeTowerManager::description_impl() const noexcept
{
    return "Wipe tower manager of the NematX printers.";
}
slicing_step_t WipeTowerManager::step_impl() const noexcept { return STEP_WIPETOWER; }
const char *const *WipeTowerManager::dependencies_impl() const noexcept { return k_no_dependencies; }
int32_t WipeTowerManager::priority_impl() const noexcept { return 0; }
const char *WipeTowerManager::exclusive_group_impl() const noexcept { return "step_wipetower_plugin"; }
int32_t WipeTowerManager::exclusive_group_priority_impl() const noexcept { return 200; }
int32_t WipeTowerManager::used_config_keys(raw_used_config_key *keys) const noexcept
{
    const int32_t count = static_cast<int32_t>(sizeof(k_used_keys) / sizeof(k_used_keys[0]));
    if (keys != nullptr)
        std::copy(k_used_keys, k_used_keys + count, keys);
    return count;
}
int32_t WipeTowerManager::defined_config_keys(const char **keys) const noexcept
{
    const int32_t count = static_cast<int32_t>(sizeof(k_defined_keys) / sizeof(k_defined_keys[0]));
    if (keys != nullptr)
        std::copy(k_defined_keys, k_defined_keys + count, keys);
    return count;
}
void WipeTowerManager::inilialize_impl(storage_handle *) const
{
    define_tower_option(m_orchestrator, k_perimeters_key, RAW_CO_INT, "Wipe tower perimeters",
        "Number of perimeter loops printed around the wipe tower, or around each separate filament section. "
        "Set to zero to disable the wipe tower perimeters while keeping the purge, the fill and the brim.", "1");
    define_tower_option(m_orchestrator, k_separate_key, RAW_CO_BOOL, "Separate filament sections",
        "Purge each filament in its own wipe tower section, with a dedicated perimeter around each section. "
        "This avoids relying on adhesion between incompatible materials inside the wipe tower.", "0");
    define_tower_option(m_orchestrator, k_only_solid_key, RAW_CO_BOOL, "Solid wipe tower spare layers",
        "If enabled, the unused area of the wipe tower, or of each separate filament section, is filled densely "
        "on every layer, like on the first layer, instead of with a sparse grid.", "0");
}

void define_tower_option(orchestrator_handle *orchestrator, const char *key, const raw_config_option_type type,
                         const char *label, const char *tooltip, const char *default_value)
{
    raw_config_option_def def = raw_config_option_def_init();
    def.opt_key = key;
    def.type = type;
    def.container_type = RAW_CONTAINER_TYPE_OBJECT;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_WIPETOWER;
    def.category = RAW_OPTION_CATEGORY_MMSETUP;
    def.label = label;
    def.tooltip = tooltip;
    def.has_min = type == RAW_CO_INT;
    def.min_value = 0;
    def.mode = raw_config_option_mode(RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI);
    def.default_serialized_value = default_value;
    const option_def_error_code result = orchestrator_create_option_def(orchestrator, &def);
    if (result != OPTION_DEF_ERROR_OK)
        throw std::runtime_error(std::string("Cannot define ") + key + " (option definition error " +
                                 std::to_string(int(result)) + ").");
}

void WipeTowerManager::run_impl(const plugin_run_context *run) const
{
    const run_ctx_generate_wipe_tower *ctx = plugin_ctx_as_generate_wipe_tower(run);
    if (ctx == nullptr || ctx->print == nullptr || ctx->plan == nullptr)
        throw std::invalid_argument("Wipe tower manager requires a Print and PrintingPlan.");
    const Print print(ctx->print);
    const Config print_config = print.config();
    const size_t nozzle_count = print_config.size("nozzle_diameter", 0);
    std::vector<TowerFootprint> footprints = towers_of_print(print);
    if (footprints.empty() || ctx->wipe_plugin_count == 0)
        return;
    if (ctx->call_wipe_plugin == nullptr)
        throw std::invalid_argument("Wipe tower manager requires service dispatch.");
    PrintingPlan plan(ctx->plan);
    const ConstPrintingPlan source(plan);
    std::vector<Tower> towers;
    for (TowerFootprint &footprint : footprints) {
        Tower tower{std::move(footprint), UINT32_MAX, {}};
        // The plan's object instances own the tower when they are its object's instance.
        for (uint32_t id = 0; id < source.object_instance_count() && tower.footprint.object != nullptr; ++id) {
            const PrintingObjectInstance owner = source.object_instance(id);
            if (owner.has_object() && owner.object().handle() == tower.footprint.object &&
                owner.object_instance_idx() == tower.footprint.instance_idx)
                tower.owner_ids.push_back(id);
        }
        tower.separate = tower.footprint.config.get(k_separate_key).get_bool();
        towers.push_back(std::move(tower));
    }
    if (std::any_of(towers.begin(), towers.end(), [](const Tower &tower) { return tower.separate; }))
        check_separate_filament(print_config, source);
    add_forced_visits(plan, towers, m_forced_visit);
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
            validate_anchor(source, request.incoming, nozzle_count);
        if (request.has_outgoing)
            validate_anchor(source, request.outgoing, nozzle_count);
        if (request.has_incoming && request.has_outgoing &&
            (request.incoming.group_idx != request.outgoing.group_idx || request.incoming.layer_group_idx != request.outgoing.layer_group_idx))
            throw std::invalid_argument("Both wipe parts must share a layer group.");
        collected.tower_index = choose_tower(source, request, towers);
        if (collected.tower_index != no_tower && towers[collected.tower_index].separate && request.has_outgoing)
            throw std::runtime_error("Separate filament sections of the wipe tower cannot print an outgoing filament "
                                     "(ramming or unload) in the section of the incoming one.");
    }
    collector.requests.erase(std::remove_if(collector.requests.begin(), collector.requests.end(),
        [](const CollectedRequest &request) { return request.tower_index == no_tower; }), collector.requests.end());
    if (collector.requests.empty()) {
        clean_forced_visits(plan, m_forced_visit);
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
        candidates[t] = tower_layers(print_config, source, tower, t, collector.requests);
        std::vector<LayoutLayerInput> inputs;
        for (const TowerLayer &layer : candidates[t])
            inputs.push_back(layer.input);
        // get_float() gives a percent as a ratio: 100% is 1. A tower with separate sections has no cone.
        layouts.push_back(layout_tower(tower.footprint.width, tower.footprint.config.get("wipe_tower_extra_spacing").get_float(),
            print_config.get("wipe_tower_no_sparse_layers").get_bool(),
            tower.separate ? 0. : tower.footprint.config.get("wipe_tower_cone_angle").get_float(), inputs));
        if (tower.separate)
            place_sections(towers[t], layouts.back(), source, collector.requests);
    }
    for (uint32_t t = 0; t < towers.size(); ++t) {
        if (layouts[t].layers.empty())
            continue;
        Tower &tower = towers[t];
        std::string name = "Wipe tower";
        // The host numbers the instance among all the model instances; the SDK only gives the printable ones.
        if (tower.footprint.object != nullptr)
            name += " " + Object(tower.footprint.object).name() + " #" + std::to_string(tower.footprint.instance_idx + 1);
        const StoredPolygon outline = make_polygon(run->plugin_storage, tower.footprint.polygon);
        tower.catalog_id = plan.add_synthetic_object_instance(name, outline);
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
                parts.push_back({request.outgoing, collected.tower_index, PartOrder::Band, layer->print_z, std::move(outgoing)});
            if (request.has_incoming)
                parts.push_back({request.incoming, collected.tower_index, PartOrder::Band, layer->print_z, std::move(incoming)});
        }
    }
    // Maintenance geometry is generated only after every service GENERATE call has completed;
    // a floating layer holds only its band and receives none.
    for (uint32_t t = 0; t < towers.size(); ++t) {
        if (towers[t].separate) {
            throw_if_cancelled(run);
            append_section_maintenance(parts, run->plugin_storage, print_config, source, towers[t], t, layouts[t],
                                       candidates[t], collector.requests);
            continue;
        }
        for (size_t l = 0; l < layouts[t].layers.size(); ++l) {
            throw_if_cancelled(run);
            const LayoutLayer &layer = layouts[t].layers[l];
            if (layer.floating)
                continue;
            const TowerLayer &candidate = candidates[t][layer.source_index];
            const c_wipe_tower_anchor anchor = maintenance_anchor(source, towers[t], candidate, collector.requests, t);
            const extruder_index_t tool = source.group(anchor.group_idx).layer_group(anchor.layer_group_idx).tool_group(anchor.tool_group_idx).extruder_id();
            const MaintenanceArea area{0., layer.depth, layer.used, true, l == 0, 0., layer.depth, layouts[t].layers.front().depth};
            parts.push_back({anchor, t, PartOrder::SharedMaintenance, layer.print_z,
                maintenance_root(run->plugin_storage, print_config, towers[t], layouts[t], l, tool, area)});
        }
    }
    std::stable_sort(parts.begin(), parts.end(), [](const GeneratedPart &left, const GeneratedPart &right) {
        const c_wipe_tower_anchor &a = left.anchor, &b = right.anchor;
        return std::make_tuple(a.group_idx, a.layer_group_idx, a.tool_group_idx, a.extrusion_idx, left.order) <
               std::make_tuple(b.group_idx, b.layer_group_idx, b.tool_group_idx, b.extrusion_idx, right.order);
    });
    for (std::vector<GeneratedPart>::reverse_iterator it = parts.rbegin(); it != parts.rend(); ++it) {
        throw_if_cancelled(run);
        GeneratedPart &part = *it;
        const Tower &tower = towers[part.tower];
        transform_root(part.root.mutable_view(), 3.141592653589793238 * tower.footprint.rotation / 180.,
            {scale_i(tower.footprint.origin_x), scale_i(tower.footprint.origin_y)});
        part.root.disable_sort().disable_reverse();
        PrintingGroup group = plan.group(part.anchor.group_idx);
        const coord_t z_offset = coord_t(std::llround(scale_d(part.print_z))) -
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
    clean_forced_visits(plan, m_forced_visit);
}

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

bool first_owner_anchor(const ConstPrintingLayerGroup &layer, const Tower &tower, c_wipe_tower_anchor &anchor)
{
    if (tower.footprint.object == nullptr && layer.tool_group_count() != 0) {
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
        if (towers[t].footprint.object == nullptr)
            global = t;
        if (std::find(towers[t].owner_ids.begin(), towers[t].owner_ids.end(), owner) != towers[t].owner_ids.end())
            return t;
    }
    return global;
}

void check_separate_filament(const Config &print_config, const ConstPrintingPlan &plan)
{
    if (print_config.get("single_extruder_multi_material").get_bool())
        throw std::runtime_error("Separate filament sections of the wipe tower cannot be used with a single extruder "
                                 "multi-material printer: every filament goes through the same nozzle.");
    const ConfigOption ramming = print_config.get("filament_multitool_ramming");
    for (uint32_t g = 0; g < plan.group_count(); ++g)
        for (uint32_t l = 0; l < plan.group(g).layer_group_count(); ++l) {
            const ConstPrintingLayerGroup layer = plan.group(g).layer_group(l);
            for (uint32_t t = 0; t < layer.tool_group_count(); ++t) {
                const extruder_index_t tool = layer.tool_group(t).extruder_id();
                if (tool >= 0 && ramming.get_bool(uint32_t(tool)))
                    throw std::runtime_error("Separate filament sections of the wipe tower cannot be used with "
                                             "filament_multitool_ramming (extruder " + std::to_string(tool + 1) + ").");
            }
        }
}

void add_forced_visits(PrintingPlan &plan, std::vector<Tower> &towers, const PluginPropertyKey<ForcedVisit> &marker)
{
    const PlanLayer absent{UINT32_MAX, UINT32_MAX};
    std::vector<PlanLayer> last_change(towers.size(), absent);
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
    // A section lasts until the last layer where its tool prints something of the tower's owners,
    // and at most until the tower's last tool change, where the tower stops.
    for (uint32_t t = 0; t < towers.size(); ++t) {
        Tower &tower = towers[t];
        if (!tower.separate || last_change[t] == absent)
            continue;
        for (uint32_t g = 0; g < plan.group_count(); ++g)
            for (uint32_t l = 0; l < plan.group(g).layer_group_count() && PlanLayer{g, l} <= last_change[t]; ++l) {
                const ConstPrintingLayerGroup layer = plan.group(g).layer_group(l);
                for (uint32_t v = 0; v < layer.tool_group_count(); ++v) {
                    const ConstPrintingToolGroup visit = layer.tool_group(v);
                    bool owned = false;
                    for (uint32_t e = 0; e < visit.extrusion_count() && !owned; ++e)
                        owned = tower.footprint.object == nullptr ||
                            std::find(tower.owner_ids.begin(), tower.owner_ids.end(),
                                      visit.extrusion(e).printing_catalog_id()) != tower.owner_ids.end();
                    if (owned)
                        tower.section_ends[visit.extruder_id()] = {g, l};
                }
            }
    }
    previous = EXTRUDER_ID_UNKNOWN;
    for (uint32_t g = 0; g < plan.group_count(); ++g)
        for (uint32_t l = 0; l < plan.group(g).layer_group_count(); ++l) {
            PrintingLayerGroup layer = plan.group(g).layer_group(l);
            for (uint32_t t = 0; t < towers.size(); ++t) {
                const Tower &tower = towers[t];
                c_wipe_tower_anchor owner{g, l, 0, 0};
                if (last_change[t] == absent || PlanLayer{g, l} > last_change[t] ||
                    !first_owner_anchor(ConstPrintingLayerGroup(layer), tower, owner))
                    continue;
                // A shared tower forces its contour tool; a separate tower forces every tool whose section
                // is still active, so that each section is printed on every layer by its own filament.
                std::vector<extruder_index_t> requested_tools;
                if (tower.separate) {
                    for (const auto &[tool, end] : tower.section_ends)
                        if (PlanLayer{g, l} <= end)
                            requested_tools.push_back(tool);
                } else {
                    const int32_t contour_tool = tower.footprint.config.get("wipe_tower_extruder").get_int();
                    if (contour_tool != 0 && tower.footprint.config.get("wipe_tower_extruder_toolchange").get_bool())
                        requested_tools.push_back(extruder_index_t(contour_tool - 1));
                }
                for (const extruder_index_t requested : requested_tools) {
                    bool exists = false;
                    for (uint32_t v = 0; v < layer.tool_group_count(); ++v)
                        exists = exists || layer.tool_group(v).extruder_id() == requested;
                    if (exists)
                        continue;
                    extruder_index_t next = EXTRUDER_ID_UNKNOWN;
                    for (uint32_t ng = g; ng < plan.group_count() && next == EXTRUDER_ID_UNKNOWN; ++ng)
                        for (uint32_t nl = ng == g ? l + 1 : 0; nl < plan.group(ng).layer_group_count() && next == EXTRUDER_ID_UNKNOWN; ++nl)
                            if (plan.group(ng).layer_group(nl).tool_group_count() != 0)
                                next = plan.group(ng).layer_group(nl).tool_group(0).extruder_id();
                    // Insert where the visit adds the fewest tool changes; ties go to the earliest position.
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
                        throw std::runtime_error("Cannot insert a forced tower visit.");
                }
            }
            if (layer.tool_group_count() != 0)
                previous = layer.tool_group(layer.tool_group_count() - 1).extruder_id();
        }
}
void clean_forced_visits(PrintingPlan &plan, const PluginPropertyKey<ForcedVisit> &marker)
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

double max_layer_height_from_nozzle(const Config &print_config, const extruder_index_t extruder)
{
    const double z_step = print_config.get("z_step").get_float();
    const double minimum = check_z_step(
        min_layer_height_mm(print_config.computed_float_or_default("min_layer_height", extruder, 0.)), z_step);
    const double maximum = print_config.computed_float_or_default("max_layer_height", extruder, 0.);
    const bool enabled = print_config.get("max_layer_height").is_enabled(uint32_t(extruder));
    const double nozzle = print_config.get("nozzle_diameter").get_float(uint32_t(extruder));
    return check_z_step(std::max(minimum, max_layer_height_mm(maximum, enabled, nozzle)), z_step);
}

double check_z_step(const double value, const double z_step)
{
    if (z_step <= EPSILON)
        return value;
    // Integer arithmetic on 1e-8 mm units avoids the drift of a floating-point modulo.
    const uint64_t value_units = uint64_t(value * 100000000. + 0.5);
    const uint64_t step_units = uint64_t(z_step * 100000000. + 0.5);
    return double(((value_units + step_units / 2) / step_units) * step_units) / 100000000.;
}

std::vector<TowerLayer> tower_layers(const Config &print_config, const ConstPrintingPlan &plan, const Tower &tower,
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
            for (uint32_t tool = 0; tool < source.tool_group_count(); ++tool)
                max_height = std::min(max_height, max_layer_height_from_nozzle(print_config, source.tool_group(tool).extruder_id()));
            for (size_t r = 0; r < requests.size(); ++r) {
                const CollectedRequest &request = requests[r];
                const c_wipe_tower_anchor &anchor = request.request.has_incoming ? request.request.incoming : request.request.outgoing;
                if (request.tower_index != tower_idx || anchor.group_idx != g || anchor.layer_group_idx != l)
                    continue;
                const extruder_index_t extruder = source.tool_group(anchor.tool_group_idx).extruder_id();
                const double nozzle = print_config.get("nozzle_diameter").get_float(uint32_t(extruder));
                const double width = tower.footprint.config.get("wipe_tower_extrusion_width").get_effective_value(nozzle);
                layer.input.bands.push_back({r, request.request.purge_volume, width > 0. ? width : nozzle, nozzle,
                    request.request.floating != 0});
            }
            layers.push_back(std::move(layer));
        }
    for (TowerLayer &layer : layers)
        layer.input.max_height = max_height;
    return layers;
}

bool root_has_content(const StoredExtrusionEntity &root)
{
    return root.has_local_points() || root.child_count() != 0 || root.property_count() != 0;
}

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

void append_visit_speed_command(storage_handle *storage, PrintingToolGroup tool,
                                const c_extrusion_special_command code, const double value)
{
    StoredExtrusionEntity command(storage);
    command.get_or_add(EPropertySpecialCommand::key).set(code, value);
    command.disable_sort().disable_reverse();
    if (!tool.events().append_before_move(command.mutable_view()).valid())
        throw std::runtime_error("Cannot append a tower speed command to the incoming visit.");
}

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
    const int32_t contour_tool = tower.footprint.config.get("wipe_tower_extruder").get_int();
    if (contour_tool > 0) {
        const ConstPrintingLayerGroup source = plan.group(layer.group).layer_group(layer.layer);
        for (uint32_t t = 0; t < source.tool_group_count(); ++t)
            if (source.tool_group(t).extruder_id() == contour_tool - 1)
                return {layer.group, layer.layer, t, 0};
    }
    return first;
}

void place_sections(Tower &tower, TowerLayout &layout, const ConstPrintingPlan &plan,
                    const std::vector<CollectedRequest> &requests)
{
    std::vector<std::vector<SectionBandInput>> inputs;
    double line_width = 0.;
    for (const LayoutLayer &layer : layout.layers) {
        inputs.emplace_back();
        for (const LayoutBand &band : layer.bands) {
            inputs.back().push_back({request_tool(plan, requests[band.request_index].request), band.y1 - band.y0});
            line_width = std::max(line_width, band.line_width);
        }
    }
    // Neighbouring purge areas keep room for the perimeters of both sections, one line spacing
    // apart; the line width is a safe bound of the spacing on every layer.
    const int32_t perimeters = tower.footprint.config.get(k_perimeters_key).get_int();
    tower.sections = layout_sections(inputs, double(std::max(1, 2 * perimeters - 1)) * line_width);
    for (size_t l = 0; l < layout.layers.size(); ++l)
        for (size_t b = 0; b < layout.layers[l].bands.size(); ++b) {
            LayoutBand &band = layout.layers[l].bands[b];
            const double height = band.y1 - band.y0;
            band.y0 = tower.sections.band_y0[l][b];
            band.y1 = band.y0 + height;
        }
}

void append_section_maintenance(std::vector<GeneratedPart> &parts, storage_handle *storage, const Config &print_config,
                                const ConstPrintingPlan &plan, const Tower &tower, const uint32_t tower_idx,
                                const TowerLayout &layout, const std::vector<TowerLayer> &candidates,
                                const std::vector<CollectedRequest> &requests)
{
    const std::vector<FilamentSection> &sections = tower.sections.sections;
    if (sections.empty())
        return;
    const double tower_depth = sections.back().y0 + sections.back().length;
    for (size_t l = 0; l < layout.layers.size(); ++l) {
        const LayoutLayer &layer = layout.layers[l];
        if (layer.floating)
            continue;
        const TowerLayer &candidate = candidates[layer.source_index];
        const PlanLayer here{candidate.group, candidate.layer};
        // A section is printed up to its end; a tool that only reaches the tower through a purge
        // request has no end, and its section is printed where it purges.
        const auto active = [&](const FilamentSection &section) {
            const std::map<extruder_index_t, PlanLayer>::const_iterator end = tower.section_ends.find(extruder_index_t(section.tool));
            return end != tower.section_ends.end() ? here <= end->second : tower.sections.used[l].count(section.tool) != 0;
        };
        // The first-layer brim surrounds every section printed on that layer.
        bool brim = l == 0;
        double brim_y0 = tower_depth, brim_y1 = 0.;
        for (const FilamentSection &section : sections)
            if (active(section)) {
                brim_y0 = std::min(brim_y0, section.y0);
                brim_y1 = std::max(brim_y1, section.y0 + section.length);
            }
        for (const FilamentSection &section : sections) {
            if (!active(section))
                continue;
            // The maintenance follows the first band of the section's tool on this layer, in the same visit;
            // without band, it starts the tool's first visit of the layer.
            c_wipe_tower_anchor anchor{UINT32_MAX, 0, 0, 0};
            for (const LayoutBand &band : layer.bands) {
                const c_wipe_tower_request &request = requests[band.request_index].request;
                if (request_tool(plan, request) == section.tool) {
                    anchor = request.incoming;
                    break;
                }
            }
            if (anchor.group_idx == UINT32_MAX) {
                const ConstPrintingLayerGroup source = plan.group(candidate.group).layer_group(candidate.layer);
                for (uint32_t v = 0; v < source.tool_group_count() && anchor.group_idx == UINT32_MAX; ++v)
                    if (source.tool_group(v).extruder_id() == section.tool)
                        anchor = {candidate.group, candidate.layer, v, 0};
            }
            if (anchor.group_idx == UINT32_MAX)
                continue;
            const std::map<int32_t, double>::const_iterator used = tower.sections.used[l].find(section.tool);
            const MaintenanceArea area{section.y0, section.y0 + section.length,
                                       section.y0 + (used == tower.sections.used[l].end() ? 0. : used->second),
                                       false, brim, brim_y0, brim_y1, tower_depth};
            parts.push_back({anchor, tower_idx, PartOrder::SectionMaintenance, layer.print_z,
                             maintenance_root(storage, print_config, tower, layout, l, section.tool, area)});
            brim = false;
        }
    }
}

extruder_index_t request_tool(const ConstPrintingPlan &plan, const c_wipe_tower_request &request)
{
    const c_wipe_tower_anchor &anchor = request.has_incoming ? request.incoming : request.outgoing;
    return plan.group(anchor.group_idx).layer_group(anchor.layer_group_idx).tool_group(anchor.tool_group_idx).extruder_id();
}
StoredExtrusionEntity maintenance_root(storage_handle *storage, const Config &print_config, const Tower &tower,
                                       const TowerLayout &layout, const size_t layer_idx, const extruder_index_t extruder,
                                       const MaintenanceArea &area)
{
    const Config &config = tower.footprint.config;
    const double width = tower.footprint.width;
    const LayoutLayer &layer = layout.layers[layer_idx];
    const double nozzle = print_config.get("nozzle_diameter").get_float(uint32_t(extruder));
    const double requested_width = config.get("wipe_tower_extrusion_width").get_effective_value(nozzle);
    const TowerFlow flow = TowerFlow::from_width(float(requested_width > 0. ? requested_width : nozzle),
        float(nozzle), float(layer.height));
    const double tower_speed = print_config.get("wipe_tower_speed").get_float();
    double speed = tower_speed > 0. ? tower_speed : 80.;
    if (layer_idx == 0) {
        // The first layer speed caps the tower speed, then its minimum floors it, like object paths.
        const c_float_or_percent first = config.get("first_layer_speed").get_float_or_percent();
        if (first.value > 0.)
            speed = std::min(speed, config.get("first_layer_speed").get_effective_value(speed));
        speed = std::max(speed, config.get("first_layer_min_speed").get_float());
    }
    StoredExtrusionEntity root(storage);
    const std::vector<c_point> rectangle{new_scale(0., area.y0), new_scale(width, area.y0), new_scale(width, area.y1),
                                         new_scale(0., area.y1)};
    StoredPolygon envelope = make_polygon(storage, rectangle);
    const double rectangle_area = envelope.area();
    const double radius = layout.cone_base_radius + layer.depth / 2. - layout.cone_slope * layer.print_z;
    if (area.cone && layout.cone_base_radius > 0.) {
        std::vector<c_point> points = rectangle;
        for (size_t n = 0; n < 80; ++n) {
            const double angle = 2. * 3.141592653589793238 * double(n) / 80.;
            points.push_back(new_scale(width / 2. + radius * std::cos(angle) / layout.cone_scale,
                layer.depth / 2. + radius * std::sin(angle)));
        }
        const StoredPolygon cone = make_polygon(storage, points);
        envelope = convex_hull(storage, cone);
    }
    // The first perimeter is the band rectangle, then the cone ring when there is a cone; each
    // further perimeter is one line spacing outside the envelope. Zero perimeters prints none of them.
    const int32_t perimeters = config.get(k_perimeters_key).get_int();
    const size_t outer_loops = perimeters > 1 ? size_t(perimeters - 1) : 0;
    // Append the closed rings of an outline offset outward by the given number of line spacings.
    const auto append_offset = [&](const StoredPolygon &outline, const size_t spacings) {
        const StoredPolygonCollection rings = clipper_offset(ClipperOperand::from_polygon(storage, outline),
            scale_d(double(spacings) * flow.spacing())).to_polygon_collection();
        for (const Polygon &ring : rings) {
            std::vector<c_point> points = ring.points();
            points.push_back(points.front());
            append_path(storage, root.mutable_view(), points, flow, speed);
        }
    };
    if (area.brim && tower.footprint.brim_width > 0.) {
        // The brim starts outside the last perimeter and is printed from the outside in. It surrounds
        // the cone envelope of a shared tower, or every section of a tower with separate sections.
        const StoredPolygon outline = area.cone ? make_polygon(storage, envelope.points()) :
            make_polygon(storage, {new_scale(0., area.brim_y0), new_scale(width, area.brim_y0),
                                   new_scale(width, area.brim_y1), new_scale(0., area.brim_y1)});
        const size_t loops = size_t((tower.footprint.brim_width + flow.spacing() / 2.) / flow.spacing());
        for (size_t i = loops; i > 0; --i)
            append_offset(outline, outer_loops + i);
    }
    if (perimeters > 0) {
        std::vector<c_point> perimeter = rectangle;
        perimeter.push_back(perimeter.front());
        append_path(storage, root.mutable_view(), perimeter, flow, speed);
        if (envelope.area() > rectangle_area + 1.) {
            std::vector<c_point> ring = envelope.points();
            ring.push_back(ring.front());
            append_path(storage, root.mutable_view(), ring, flow, speed);
        }
    }
    for (size_t i = 1; i <= outer_loops; ++i)
        append_offset(envelope, i);
    StoredExPolygonCollection fill = clipper_offset(ClipperOperand::from_polygon(storage, envelope),
        -double(flow.scaled_spacing()) / 3.).to_expolygon_collection();
    if (area.used_y1 > area.y0) {
        const StoredPolygon used = make_polygon(storage, {new_scale(0., area.y0), new_scale(width, area.y0),
            new_scale(width, area.used_y1), new_scale(0., area.used_y1)});
        fill = clipper_diff(ClipperOperand::from_expolygons(storage, fill),
            ClipperOperand::from_polygon(storage, used)).to_expolygon_collection();
    }
    raw_infill_pattern_params params = {};
    params.extrusion_role = RAW_EXTRUSION_ROLE_WIPE_TOWER;
    params.flow.width = flow.scaled_width();
    params.flow.height = flow.scaled_height();
    params.flow.spacing = flow.scaled_spacing();
    params.flow.nozzle_diameter = scale_i(nozzle);
    params.flow.spacing_ratio = 1.;
    params.flow.mm3_per_mm = flow.mm3_per_mm();
    params.flow_mult = 1.f;
    params.connection = C_INFILL_CONNECTION_CONNECTED;
    params.angle = 3.141592653589793238 / 4.;
    params.bridge_angle = -1.;
    params.layer_id = uint32_t(layer_idx);
    params.layer_height = layer.height;
    params.z = layer.print_z;
    params.fill_resolution = scale_i(0.01);
    // The fill pattern is centred on the tower rectangle; a cone envelope may extend beyond it.
    params.object_bounding_box = {{0, 0}, {scale_i(width), scale_i(area.pattern_y1)}};
    params.spacing = flow.spacing();
    params.density = 1.f;
    // The first layer is always dense; wipe_tower_only_solid makes every layer dense.
    const bool dense = layer_idx == 0 || config.get(k_only_solid_key).get_bool();
    if (!dense) {
        // Grid divides density between two sweeps. Limit cell diagonals, not just their perpendicular pitch.
        // The extra spacing percent is read as a ratio.
        const double nominal_pitch = 20. * flow.spacing() * config.get("wipe_tower_extra_spacing").get_float();
        const double bridging = config.get("wipe_tower_bridging").get_float();
        const double pitch = bridging > 0. ? std::min(nominal_pitch, bridging / std::sqrt(2.)) : nominal_pitch;
        params.density = float(std::min(1., 2. * flow.spacing() / pitch));
    }
    raster::FillMonotonic solid;
    raster::FillGrid sparse;
    raster::FillRectilinear &filler = dense ? static_cast<raster::FillRectilinear &>(solid) : sparse;
    raster::configure_filler(filler, params, storage);
    for (const ExPolygon &polygon : fill) {
        const raster_work::Polylines lines = filler.fill_surface(polygon, params);
        for (const raster_work::Polyline &line : lines) {
            std::vector<c_point> points;
            for (const raster_work::Point &point : line.points)
                points.push_back({coord_t(point.x()), coord_t(point.y())});
            if (points.size() >= 2)
                append_path(storage, root.mutable_view(), points, flow, speed);
        }
    }
    root.disable_sort().disable_reverse();
    return root;
}

void append_path(storage_handle *storage, MutableExtrusionEntity root, const std::vector<c_point> &points,
                 const TowerFlow &flow, const double speed)
{
    StoredExtrusionEntity path(storage);
    path.set_points(points);
    path.get_or_add(EPropertyAttributes::key).extrusion_role(RAW_EXTRUSION_ROLE_WIPE_TOWER)
        .width(flow.width).height(flow.height).mm3_per_mm(flow.mm3_per_mm());
    path.get_or_add(EPropertySpeed::key).speed_mm_per_s = float(speed);
    path.disable_sort().disable_reverse();
    if (root.append_child_move(path.mutable_view()) == UINT32_MAX)
        throw std::runtime_error("Cannot append tower maintenance path.");
}

void transform_root(MutableExtrusionEntity root, const double angle, const c_point origin)
{
    if (root.has_local_points() && (!root.rotate(angle) || !root.translate(origin)))
        throw std::runtime_error("Cannot transform tower extrusion to platter coordinates.");
    for (uint32_t idx = 0; idx < root.child_count(); ++idx)
        transform_root(root.child_mutable(idx), angle, origin);
}

c_point new_scale(const double x, const double y)
{
    return {coord_t(std::round(scale_d(x))), coord_t(std::round(scale_d(y)))};
}

StoredPolygon make_polygon(storage_handle *storage, const std::vector<c_point> &points)
{
    StoredPolygon polygon(storage);
    polygon.insert_array(0, points.data(), uint32_t(points.size()));
    return polygon;
}
} // namespace
} // namespace nematx_wipetower

// Host ABI entry point. Register a static instance so it stays alive for every callback.
extern "C" SLIC3R_PLUGIN_API void register_plugin(orchestrator_handle *orchestrator)
{
    static nematx_wipetower::WipeTowerManager manager(orchestrator);
    orchestrator_register_plugin(orchestrator, manager.c_instance());
}

// Keep this header last so its ABI registration includes every public API header above.
#include "libslic3r/Api/plugin/c/slic3r_plugin_register_version.h"
