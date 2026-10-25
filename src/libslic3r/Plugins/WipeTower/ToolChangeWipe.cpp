///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* ToolChangeWipe.hpp defines visit, volume and speed rules. Callbacks borrow the
   print and plan only for one phase. EDIT adds the empty tool visits needed by the
   single-extruder multi-material (SEMM) sequences, COLLECT reports value anchors,
   GENERATE moves local paths into manager-owned roots, leaving selection,
   placement and Z offsets to the manager and downstream plugins.

   The band of one request is filled by one connected zigzag of X lines (the
   "purge path"). The outgoing part of a request, when present, uses the start of
   that path for the ramming of the old filament; the incoming part uses the rest
   for the purge of the new one. Each leaf carries explicit flow and speed. The
   purge speed ramps from wipe_tower_wipe_starting_speed to wipe_tower_speed over
   four lines, and the incoming filament purge factor multiplies all of it:
   filament_max_wipe_tower_speed as a percentage, or, when that value is not
   positive, a fallback by filament type (soluble 35 %, PVA 60 % below 0.8 mm of
   tower and 80 % above, SCAFF 35 %, FLEX 35 %, others 100 %). Ramming, the SEMM
   unload and load and the tower maintenance keep their own speeds.

   In SEMM, the unload and load also travel along a "work line" inside the band:
   the line one band line above the end of the ramming, capped at the band top,
   or the band entry when nothing was rammed. That line is not printed yet, so
   the filament of the cooling and loading moves is spread over it instead of
   forming a blob at one position, like WipeTower v1. Each such move keeps the
   configured filament speed: its duration is |E| / speed, its XY speed is the
   travelled X distance over that duration, and a positive machine ceiling
   shortens the travel to ceiling * duration without ever slowing the filament.
   The ceiling is the smallest positive normal-mode X or Y machine feedrate,
   because the tower may be rotated, or 100 mm/s when neither is positive;
   machine_limits_usage = ignore leaves the travel unshortened. The purge then
   resumes at the path cursor, and the firmware inserts the connecting travel
   between these non-contiguous leaves.
   Filament moves that stay in place (the first loading part, cooling-tube pulls,
   skinnydip, pauses, parking) remain point-less leaves carrying an E-only
   EXTRUSION special command and their own speed, so the firmware emits them
   where the nozzle stands, inside the tower.

   The swap is the retraction itself: the unload and load collections carry the
   EPropertyModifier disable_retraction flag, which CreateRetraction honours by
   cancelling a boundary whose source or target leaf disables retraction. So no
   retraction or unretraction is added inside the swap (from the end of the
   ramming to the tool change, and from the load to the purge cursor), the
   tool-change retraction is skipped because the boundary leaves are swap
   leaves, the travel retraction from the object to the tower before the
   ramming stays, and an outgoing-only final unload gets no terminal
   retraction.

   During a SEMM swap the unload also adds an entry heater event for the
   incoming filament temperature (first_layer_temperature on the first layer
   of the plan, temperature above it) without waiting, because nothing adds a
   normal heater event afterwards and the shared SEMM heater keeps it in
   effect after the tool change.

   Pressure advance is a request carried by the two roots. The outgoing root
   asks for zero while it rams and unloads, because those moves push or pull
   filament without printing; the request is made only when
   filament_pressure_advance is enabled for the outgoing tool, so a value
   written by the filament G-code is left alone otherwise. The incoming root
   asks for the incoming filament value when it is enabled, because no other
   step requests the normal pressure advance: that is what the load and the
   purge print with, and in a multi-nozzle printer the rest of the visit keeps
   it.

   Wipe into infill/objects measures what the incoming tool prints first: for
   every change A->B, COLLECT walks the printable leaves at the start of B's
   visit in print order and adds the volume of the consecutive eligible ones,
   stopping at the first printable leaf that is not eligible. A leaf is eligible
   when its object has wipe_into_objects, or when a region of its island has
   wipe_into_infill and the leaf is sparse infill (INTERNAL_INFILL) whose
   perimeters are already printed: with wipe_into_infill alone and no
   infill_first, the island's perimeter tool must be the incoming tool or print
   earlier in the same layer. Nothing is moved or reassigned to another tool by
   the wipe tower, and nothing is measured when either filament is soluble. The
   measured volume is removed from the tower purge, down to the minimal purge of
   the incoming tool. */
#include "ToolChangeWipe.hpp"
#include "WipeTowerLayout.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_wipe_tower_wipe.h"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PluginPropertyKey.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Print.hpp"

namespace slic3r_api::WipeTower {
namespace {
const raw_used_config_key k_used_keys[] = {
    {"wiping_volumes_matrix", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"wiping_volumes_extruders", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_minimal_purge_on_wipe_tower", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"nozzle_diameter", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"wipe_tower_speed", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"wipe_tower_wipe_starting_speed", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_max_wipe_tower_speed", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_type", RAW_CO_VECTOR_STRING, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_soluble", RAW_CO_VECTOR_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"single_extruder_multi_material", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"single_extruder_multi_material_priming", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_ramming_parameters", RAW_CO_VECTOR_STRING, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_multitool_ramming", RAW_CO_VECTOR_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_multitool_ramming_volume", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_multitool_ramming_flow", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"cooling_tube_retraction", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"cooling_tube_length", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"parking_pos_retraction", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"extra_loading_move", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"high_current_on_filament_swap", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_loading_speed", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_loading_speed_start", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_unloading_speed", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_unloading_speed_start", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_cooling_moves", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_cooling_initial_speed", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_cooling_final_speed", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_enable_toolchange_temp", RAW_CO_VECTOR_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_toolchange_temp", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"temperature", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"first_layer_temperature", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_enable_toolchange_part_fan", RAW_CO_VECTOR_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_toolchange_part_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_use_skinnydip", RAW_CO_VECTOR_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_use_fast_skinnydip", RAW_CO_VECTOR_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_skinnydip_distance", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_dip_insertion_speed", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_dip_extraction_speed", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_melt_zone_pause", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_cooling_zone_pause", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_toolchange_delay", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_pressure_advance", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"machine_max_feedrate_x", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"machine_max_feedrate_y", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"machine_limits_usage", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"wipe_into_infill", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"wipe_into_objects", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"infill_first", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"perimeter_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
};

// Speed used for filament moves whose configured speed is not positive (mm/s of filament).
const double k_fallback_filament_speed = 10.;

// One step of a ramming profile: a volume extruded at a constant volumetric speed.
struct RammingStep {
    double volume;          // mm3
    double volumetric_speed; // mm3/s
};

// The zigzag purge path of one band, consumed from its start by successive leaves.
struct PurgePath {
    std::vector<c_vec2d> points;
    // line_end[i] is true when points[i] ends an X line (the end of a natural leaf).
    std::vector<bool> line_end;
    // Current position: on the segment points[segment] -> points[segment + 1], at distance offset from its start.
    size_t segment = 0;
    double offset = 0.;
};

class ToolChangeWipe final : public PluginBase {
public:
    // Borrow the host for process-lifetime service callbacks.
    explicit ToolChangeWipe(orchestrator_handle *orchestrator);
private:
    // Return the stable service identifier.
    const char *id_impl() const noexcept override;
    // Return the display name.
    const char *name_impl() const noexcept override;
    // Select the manager-driven geometry service step.
    slicing_step_t step_impl() const noexcept override;
    // Declare no service dependencies.
    const char *const *dependencies_impl() const noexcept override;
    // Allow this wipe to coexist with other wipe services.
    const char *exclusive_group_impl() const noexcept override;
    // Use the default service priority.
    int32_t priority_impl() const noexcept override;
    // Copy the typed option dependencies to an optional caller array.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    // Add SEMM tool visits, collect tool-change requests or fill the supplied band.
    void run_impl(const plugin_run_context *run) const override;
};

// Add the SEMM priming visits and the empty outgoing visits needed at layer starts.
void edit_plan(const Slic3r::PrintConfig &config, PrintingPlan plan, const plugin_run_context *run);
// Return true when the change away from tool needs an outgoing part (ramming and/or unload).
bool needs_outgoing(const Slic3r::PrintConfig &config, extruder_index_t tool);
// Return the volume of the consecutive eligible leaves at the start of one incoming visit.
double consecutive_wiped_volume(const ConstPrintingLayerGroup &layer, uint32_t visit_index, extruder_index_t incoming);
// Add the volume of the consecutive eligible leaves of a borrowed tree in print order.
bool measure_eligible_leaves(const Slic3r::ExtrusionEntity &entity, const Slic3r::ExtrusionAttributes *inherited,
                             bool eligible_all, bool eligible_sparse, double &volume);
// Report one request per real tool change, plus the final SEMM unload.
void collect_requests(const Slic3r::PrintConfig &config, const ConstPrintingPlan &plan,
                      const run_ctx_wipe_tower_wipe &ctx, const plugin_run_context *run);
// Resolve a pair's matrix/component/minimum purge volume.
double purge_volume(const Slic3r::PrintConfig &config, extruder_index_t from, extruder_index_t to);
// Return the factor applied to the purge speed of the incoming filament on the tower.
double purge_speed_factor(const Slic3r::PrintConfig &config, extruder_index_t tool, double print_z);
// Return the ramming profile of a tool leaving the nozzle; empty when it is not rammed.
std::vector<RammingStep> ramming_steps(const Slic3r::PrintConfig &config, extruder_index_t tool);
// Sum the volume of a ramming profile.
double ramming_volume(const std::vector<RammingStep> &steps);
// Return true when the band prints at the Z of the first layer of the plan.
bool is_first_plan_layer(const ConstPrintingPlan &plan, double print_z);
// Add a toolchange heater entry event, and its part fan when enabled, to a collection.
void request_toolchange_temperature(StoredExtrusionEntity &entity, const Slic3r::PrintConfig &config,
                                    size_t idx, bool wait);
// Add an incoming-filament heater entry event without waiting to a collection.
void request_incoming_temperature(StoredExtrusionEntity &entity, const Slic3r::PrintConfig &config,
                                  extruder_index_t tool, bool first_layer);
// Fill the band of one request: ramming and unload in the outgoing root, load and purge in the incoming root.
void generate(const Slic3r::PrintConfig &config, const ConstPrintingPlan &plan,
              const run_ctx_wipe_tower_wipe &ctx, const plugin_run_context *run);
// Build the zigzag purge path of a band from its entry point.
PurgePath purge_path(const c_wipe_tower_band &band, size_t lines);
// Advance along the purge path by length (or to the next X line end), returning the covered points.
std::vector<c_vec2d> take_path(PurgePath &path, double length, bool stop_at_line_end);
// Append a geometric leaf with the given role, flow and speed to a root.
void append_leaf(storage_handle *storage, extrusion_entity_handle *root, const std::vector<c_vec2d> &points,
                 int32_t role, const Slic3r::Flow &flow, double speed);
// Return the SEMM cooling/loading XY speed ceiling, or 0 when machine limits are ignored.
double xy_speed_cap(const Slic3r::PrintConfig &config);
// Append one filament move of distance mm at filament speed mm/s toward target_x along the current
// work line, shortening the travel to a positive XY ceiling and moving position to the leaf end.
void append_x_move(storage_handle *storage, extrusion_entity_handle *root, c_vec2d &position,
                   double target_x, double distance, double speed, double cap);
// Append the SEMM unload sequence of the old filament (cooling tube, cooling moves, skinnydip, parking).
void append_unload(storage_handle *storage, extrusion_entity_handle *root, const Slic3r::PrintConfig &config,
                   extruder_index_t tool, extruder_index_t incoming, bool first_layer, const c_wipe_tower_band &band,
                   double work_y, double cap, c_vec2d &position);
// Append the SEMM load sequence of the new filament.
void append_load(storage_handle *storage, extrusion_entity_handle *root, const Slic3r::PrintConfig &config,
                 extruder_index_t tool, const c_wipe_tower_band &band, double work_y, double cap,
                 c_vec2d &position);
// Append a point-less leaf carrying one special command, with an optional filament speed.
void append_command(storage_handle *storage, extrusion_entity_handle *root, c_extrusion_special_command code,
                    double value, double speed = -1.);
// Append an E-only filament move of distance mm at speed mm/s; zero distances are skipped.
void append_filament_move(storage_handle *storage, extrusion_entity_handle *root, double distance, double speed);

// Initialize plugin metadata against a borrowed nonnull host; no ownership changes and no failure.
ToolChangeWipe::ToolChangeWipe(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}
// Return a borrowed constant identifier without allocation or mutation.
const char *ToolChangeWipe::id_impl() const noexcept { return "wipe_tower.wipe.toolchange.default"; }
// Return a borrowed display name without allocation or mutation.
const char *ToolChangeWipe::name_impl() const noexcept { return "Default tool-change wipe"; }
// Return the service step, whose option reads invalidate its manager step.
slicing_step_t ToolChangeWipe::step_impl() const noexcept { return WIPE_TOWER_WIPE; }
// Return no dependencies; the manager dispatches this service explicitly.
const char *const *ToolChangeWipe::dependencies_impl() const noexcept { return nullptr; }
// Return an empty selector so other wipe services remain independently active.
const char *ToolChangeWipe::exclusive_group_impl() const noexcept { return ""; }
// Return constant scheduling priority without effects or failure.
int32_t ToolChangeWipe::priority_impl() const noexcept { return 0; }
// Copy fixed descriptors to a sufficiently sized optional array; strings stay module-owned and no allocation occurs.
int32_t ToolChangeWipe::used_config_keys(raw_used_config_key *keys) const noexcept
{
    const int32_t count = static_cast<int32_t>(sizeof(k_used_keys) / sizeof(k_used_keys[0]));
    if (keys != nullptr)
        std::copy(k_used_keys, k_used_keys + count, keys);
    return count;
}

// Borrow a valid phase payload and dispatch it: EDIT changes only tool visits, COLLECT only reports, GENERATE only
// fills the lent roots. Cancellation, invalid geometry/config, reporting and allocation failures throw without
// retaining callback pointers.
void ToolChangeWipe::run_impl(const plugin_run_context *run) const
{
    const run_ctx_wipe_tower_wipe *ctx = plugin_ctx_as_wipe_tower_wipe(run);
    if (ctx == nullptr || ctx->print == nullptr || ctx->plan == nullptr)
        throw std::invalid_argument("Tool-change wipe requires a Print and PrintingPlan.");
    const Slic3r::Print &print = *reinterpret_cast<const Slic3r::Print *>(ctx->print);
    const Slic3r::PrintConfig &config = print.config();
    if (ctx->phase == C_WIPE_TOWER_EDIT) {
        if (ctx->mutable_plan == nullptr)
            throw std::invalid_argument("Tool-change wipe EDIT requires a mutable plan.");
        edit_plan(config, PrintingPlan(ctx->mutable_plan), run);
        return;
    }
    const ConstPrintingPlan plan(ctx->plan);
    if (ctx->phase == C_WIPE_TOWER_COLLECT) {
        if (ctx->report_request == nullptr)
            throw std::invalid_argument("Tool-change wipe COLLECT requires a reporter.");
        collect_requests(config, plan, *ctx, run);
        return;
    }
    if (ctx->phase != C_WIPE_TOWER_GENERATE || ctx->request == nullptr || ctx->band == nullptr || ctx->exit == nullptr)
        throw std::invalid_argument("Tool-change wipe GENERATE requires a request, band and exit.");
    generate(config, plan, *ctx, run);
}

// Walk the borrowed mutable plan twice. With SEMM priming, insert at the start of the first printed layer one empty
// visit per other used tool (ascending ids), so each of them is loaded and purged once before the first visit. Then,
// for every change away from a tool that needs an outgoing part where the new tool opens its layer group, insert an
// empty visit of the old tool at the start of that layer group: the ramming/unload of the old filament is printed at
// the Z of the layer where the change happens. Insertions are collected first and applied afterwards, each at index
// 0 of a distinct layer group, so collected positions stay valid. Unresolved visits are ignored.
void edit_plan(const Slic3r::PrintConfig &config, PrintingPlan plan, const plugin_run_context *run)
{
    const size_t tool_count = config.nozzle_diameter.size();
    const bool semm = config.single_extruder_multi_material.value;
    if (semm && config.single_extruder_multi_material_priming.value) {
        std::vector<bool> used(tool_count, false);
        extruder_index_t first = EXTRUDER_ID_UNKNOWN;
        uint32_t first_group = 0, first_layer = 0;
        for (uint32_t g = 0; g < plan.group_count(); ++g)
            for (uint32_t l = 0; l < plan.group(g).layer_group_count(); ++l) {
                const PrintingLayerGroup layer = plan.group(g).layer_group(l);
                for (uint32_t t = 0; t < layer.tool_group_count(); ++t) {
                    const extruder_index_t tool = layer.tool_group(t).extruder_id();
                    if (tool < 0 || size_t(tool) >= tool_count)
                        continue;
                    used[size_t(tool)] = true;
                    if (first == EXTRUDER_ID_UNKNOWN) {
                        first = tool;
                        first_group = g;
                        first_layer = l;
                    }
                }
            }
        if (first != EXTRUDER_ID_UNKNOWN) {
            PrintingLayerGroup layer = plan.group(first_group).layer_group(first_layer);
            uint32_t position = 0;
            for (size_t tool = 0; tool < tool_count; ++tool)
                if (used[tool] && extruder_index_t(tool) != first) {
                    const uint32_t appended = layer.tool_group_count();
                    layer.append_tool_group(int64_t(tool));
                    if (!layer.move_tool_group(appended, position++))
                        throw std::runtime_error("Cannot insert a priming tool visit.");
                }
        }
    }
    struct Insertion { uint32_t group; uint32_t layer; extruder_index_t tool; };
    std::vector<Insertion> insertions;
    extruder_index_t previous = EXTRUDER_ID_UNKNOWN;
    for (uint32_t g = 0; g < plan.group_count(); ++g)
        for (uint32_t l = 0; l < plan.group(g).layer_group_count(); ++l) {
            throw_if_cancelled(run);
            const PrintingLayerGroup layer = plan.group(g).layer_group(l);
            bool first_visit = true;
            for (uint32_t t = 0; t < layer.tool_group_count(); ++t) {
                const extruder_index_t tool = layer.tool_group(t).extruder_id();
                if (tool < 0 || size_t(tool) >= tool_count)
                    continue;
                if (first_visit && previous != EXTRUDER_ID_UNKNOWN && previous != tool && needs_outgoing(config, previous))
                    insertions.push_back({g, l, previous});
                first_visit = false;
                previous = tool;
            }
        }
    for (const Insertion &insertion : insertions) {
        PrintingLayerGroup layer = plan.group(insertion.group).layer_group(insertion.layer);
        const uint32_t appended = layer.tool_group_count();
        layer.append_tool_group(int64_t(insertion.tool));
        if (!layer.move_tool_group(appended, 0))
            throw std::runtime_error("Cannot insert an outgoing tool visit.");
    }
}

// SEMM unloads every filament; a multi-nozzle printer rams only the tools with filament_multitool_ramming. Reads
// borrowed configuration only.
bool needs_outgoing(const Slic3r::PrintConfig &config, const extruder_index_t tool)
{
    return config.single_extruder_multi_material.value || config.filament_multitool_ramming.get_at(size_t(tool));
}

// Volume of the consecutive eligible leaves at the start of the incoming visit, in print order: the
// leaves of its PrintingExtrusions, depth first, whole leaves only. A leaf is eligible when the object
// of its extrusion's region island has wipe_into_objects, or when a region of that island has
// wipe_into_infill and the leaf is sparse infill; with wipe_into_infill alone and no infill_first in
// the island, the perimeters must already be printed, so the island's perimeter tool must be the
// incoming tool or have a visit before it in the same layer. The walk stops at the first printable
// leaf that is not eligible. Reads borrowed views only.
double consecutive_wiped_volume(const ConstPrintingLayerGroup &layer, const uint32_t visit_index,
                                const extruder_index_t incoming)
{
    const ConstPrintingToolGroup visit = layer.tool_group(visit_index);
    double volume = 0.;
    for (uint32_t e = 0; e < visit.extrusion_count(); ++e) {
        const ConstPrintingExtrusion extrusion = visit.extrusion(e);
        bool eligible_all = false;
        bool eligible_sparse = false;
        const LayerRegionIsland island = extrusion.region_island();
        if (island.valid() && island.region_count() > 0) {
            // wipe_into_objects is an object option: every leaf of the extrusion is then eligible.
            eligible_all = island.region(0).layer().object().config().bool_or_default("wipe_into_objects", false);
            if (!eligible_all) {
                // wipe_into_infill is a region option: one region with it enables the sparse infill.
                bool into_infill = false;
                bool infill_first = false;
                for (uint32_t r = 0; r < island.region_count(); ++r) {
                    into_infill = into_infill ||
                        island.region(r).print_region().config().bool_or_default("wipe_into_infill", false);
                    infill_first = infill_first ||
                        island.region(r).print_region().config().bool_or_default("infill_first", false);
                }
                // Perimeters first, like WipeTower v1: the island's perimeter tool is its first
                // region's configured one-based perimeter_extruder, not the island's own extruder_id,
                // which is the extruder of its bucket (infill, ironing, ...). A missing or invalid
                // perimeter extruder keeps the leaf ineligible.
                bool perimeters_first = true;
                if (into_infill && !infill_first) {
                    const int32_t configured =
                        island.region(0).print_region().config().int_or_default("perimeter_extruder", 1);
                    const extruder_index_t perimeter_tool = configured > 0 ?
                        extruder_index_t(configured - 1) : EXTRUDER_ID_UNKNOWN;
                    perimeters_first = perimeter_tool == incoming;
                    if (!perimeters_first && perimeter_tool != EXTRUDER_ID_UNKNOWN)
                        for (uint32_t t = 0; t < visit_index && !perimeters_first; ++t)
                            perimeters_first = layer.tool_group(t).extruder_id() == perimeter_tool;
                }
                eligible_sparse = into_infill && perimeters_first;
            }
        }
        if (!measure_eligible_leaves(*reinterpret_cast<const Slic3r::ExtrusionEntity *>(extrusion.root().handle()),
                                     nullptr, eligible_all, eligible_sparse, volume))
            break;
    }
    return volume;
}

// Attributes are inherited from the nearest ancestor that has them. A printable leaf (points and a
// positive inherited mm3_per_mm) adds its length times mm3_per_mm when it is eligible; the walk stops
// (returning false) at the first printable leaf that is not, and skips leaves that carry no material.
bool measure_eligible_leaves(const Slic3r::ExtrusionEntity &entity, const Slic3r::ExtrusionAttributes *inherited,
                             const bool eligible_all, const bool eligible_sparse, double &volume)
{
    if (const Slic3r::ExtrusionAttributes *attributes = entity.get_property<Slic3r::ExtrusionAttributes>())
        inherited = attributes;
    if (entity.child_count() == 0) {
        if (inherited == nullptr || !(inherited->mm3_per_mm > 0.) || !entity.has_polyline())
            return true;
        const bool eligible = eligible_all ||
            (eligible_sparse && inherited->role == uint16_t(RAW_EXTRUSION_ROLE_INTERNAL_INFILL));
        if (!eligible)
            return false;
        volume += Slic3r::unscaled(entity.length()) * inherited->mm3_per_mm;
        return true;
    }
    for (size_t idx = 0; idx < entity.child_count(); ++idx)
        if (!measure_eligible_leaves(entity.child(idx), inherited, eligible_all, eligible_sparse, volume))
            return false;
    return true;
}

// Walk the borrowed plan in print order and report, for every real change A->B between consecutive concrete visits
// (across layer and printing groups, empty visits included), one request anchored at the start of B's visit. When A
// needs an outgoing part and A's visit directly precedes B's in the same layer group, the request also anchors the
// outgoing part at the end of A's visit and adds A's ramming volume to the purge volume. The volume of the eligible
// object geometry that B prints first is removed from the purge, down to the minimal purge of B; nothing is measured
// when either filament is soluble. With SEMM and at least one reported change, a last outgoing-only floating request
// unloads the filament left in the nozzle at the end of the plan, so a tower that stopped lower does not grow to reach
// it; a print without any change reports nothing and lets the end G-code unload. Unresolved visits are skipped;
// reporter failures throw.
void collect_requests(const Slic3r::PrintConfig &config, const ConstPrintingPlan &plan,
                      const run_ctx_wipe_tower_wipe &ctx, const plugin_run_context *run)
{
    const size_t tool_count = config.nozzle_diameter.size();
    extruder_index_t previous = EXTRUDER_ID_UNKNOWN;
    c_wipe_tower_anchor previous_end = {};
    bool previous_adjacent = false;
    uint64_t id = 0;
    for (uint32_t g = 0; g < plan.group_count(); ++g)
        for (uint32_t l = 0; l < plan.group(g).layer_group_count(); ++l) {
            throw_if_cancelled(run);
            const ConstPrintingLayerGroup layer = plan.group(g).layer_group(l);
            previous_adjacent = false;
            for (uint32_t t = 0; t < layer.tool_group_count(); ++t) {
                const ConstPrintingToolGroup visit = layer.tool_group(t);
                const extruder_index_t tool = visit.extruder_id();
                // An unresolved or unconfigured visit cannot be purged; it is left to the
                // plain tool change and does not reset the previous concrete tool.
                if (tool < 0 || size_t(tool) >= tool_count) {
                    previous_adjacent = false;
                    continue;
                }
                if (previous != EXTRUDER_ID_UNKNOWN && previous != tool) {
                    double volume = purge_volume(config, previous, tool);
                    // WipeTower v1 never wipes into an object with a soluble filament.
                    if (!config.filament_soluble.get_at(size_t(previous)) &&
                        !config.filament_soluble.get_at(size_t(tool)))
                        volume = std::max(volume - consecutive_wiped_volume(layer, t, tool),
                                          config.filament_minimal_purge_on_wipe_tower.get_at(size_t(tool)));
                    c_wipe_tower_request request{++id, {g, l, t, 0}, {}, 1, 0, volume, 0};
                    if (previous_adjacent && needs_outgoing(config, previous)) {
                        request.outgoing = previous_end;
                        request.has_outgoing = 1;
                        request.purge_volume += ramming_volume(ramming_steps(config, previous));
                    }
                    if (!ctx.report_request(ctx.report_context, &request))
                        throw std::runtime_error("Cannot report tool-change wipe request.");
                }
                previous = tool;
                previous_end = {g, l, t, visit.extrusion_count()};
                previous_adjacent = true;
            }
        }
    // No reported change means nothing was purged, so no tower grows and no final unload is
    // reported; the end G-code unloads the filament, like the legacy tower.
    if (config.single_extruder_multi_material.value && id > 0) {
        // The final unload is floating: it may be printed above a tower that stopped lower.
        const c_wipe_tower_request request{++id, {}, previous_end, 0, 1,
            ramming_volume(ramming_steps(config, previous)), 1};
        if (!ctx.report_request(ctx.report_context, &request))
            throw std::runtime_error("Cannot report the final unload request.");
    }
}

// Read borrowed vectors for concrete configured indices, use a printer-sized square matrix or paired components and floor by incoming minimum; absent sources use both minima, invalid volumes throw and no state changes.
double purge_volume(const Slic3r::PrintConfig &config, const extruder_index_t from, const extruder_index_t to)
{
    const std::vector<double> &matrix = config.wiping_volumes_matrix.get_values();
    const size_t side = size_t(std::sqrt(double(matrix.size())));
    double volume;
    if (side == config.nozzle_diameter.size() && side * side == matrix.size())
        volume = matrix[size_t(from) * side + size_t(to)];
    else {
        const std::vector<double> &components = config.wiping_volumes_extruders.get_values();
        if (size_t(from) * 2 < components.size() && size_t(to) * 2 + 1 < components.size())
            volume = components[size_t(from) * 2] + components[size_t(to) * 2 + 1];
        else
            volume = config.filament_minimal_purge_on_wipe_tower.get_at(from);
    }
    volume = std::max(volume, config.filament_minimal_purge_on_wipe_tower.get_at(to));
    if (!std::isfinite(volume) || volume < 0.)
        throw std::invalid_argument("Tool-change purge volume must be finite and nonnegative.");
    return volume;
}

// Purge speed factor of the incoming filament: filament_max_wipe_tower_speed is a percentage of the purge
// speed, and a value that is not positive falls back to the filament type, each rule replacing the previous
// one like WipeTower v1: soluble 35 %, PVA 60 % below 0.8 mm of tower and 80 % above, SCAFF 35 %, FLEX 35 %,
// any other material 100 %. Reads borrowed configuration only.
double purge_speed_factor(const Slic3r::PrintConfig &config, const extruder_index_t tool, const double print_z)
{
    const size_t idx = size_t(tool);
    const double configured = config.filament_max_wipe_tower_speed.get_at(idx);
    if (configured > 0.)
        return configured / 100.;
    double factor = 1.;
    if (config.filament_soluble.get_at(idx))
        factor = 0.35;
    std::string material = config.filament_type.get_at(idx);
    std::transform(material.begin(), material.end(), material.begin(),
        [](const unsigned char c) { return char(std::toupper(c)); });
    if (material == "PVA")
        factor = print_z < 0.80 ? 0.60 : 0.80;
    if (material == "SCAFF")
        factor = 0.35;
    if (material == "FLEX")
        factor = 0.35;
    return factor;
}

// SEMM: parse filament_ramming_parameters ("<width %> <step %> v1 v2 ...| <spline points>"): every v is a volumetric
// speed in mm3/s held for 0.25 s; the two leading percentages and the part after '|' are ignored, the ramming lines
// use the band flow. Multi-nozzle with filament_multitool_ramming: one step of the configured volume at the configured
// flow. Otherwise, or when nothing positive is configured, the profile is empty. Malformed numbers end the profile.
std::vector<RammingStep> ramming_steps(const Slic3r::PrintConfig &config, const extruder_index_t tool)
{
    std::vector<RammingStep> steps;
    if (config.single_extruder_multi_material.value) {
        const std::string &text = config.filament_ramming_parameters.get_at(size_t(tool));
        std::istringstream stream(text.substr(0, text.find('|')));
        double width_percent = 0., step_percent = 0., speed = 0.;
        if (!(stream >> width_percent >> step_percent))
            return steps;
        while (stream >> speed)
            if (std::isfinite(speed) && speed > 0.)
                steps.push_back({speed * 0.25, speed});
    } else if (config.filament_multitool_ramming.get_at(size_t(tool))) {
        const double volume = config.filament_multitool_ramming_volume.get_at(size_t(tool));
        const double flow = config.filament_multitool_ramming_flow.get_at(size_t(tool));
        if (std::isfinite(volume) && volume > 0. && std::isfinite(flow) && flow > 0.)
            steps.push_back({volume, flow});
    }
    return steps;
}

// Sum the step volumes in mm3; an empty profile gives 0.
double ramming_volume(const std::vector<RammingStep> &steps)
{
    double volume = 0.;
    for (const RammingStep &step : steps)
        volume += step.volume;
    return volume;
}

// Return true when the band prints at the Z of the first layer of the plan; a plan without a layer
// counts as the first layer. Reads borrowed views only.
bool is_first_plan_layer(const ConstPrintingPlan &plan, const double print_z)
{
    double first = print_z;
    bool found = false;
    for (uint32_t group = 0; group < plan.group_count(); ++group)
        for (uint32_t layer = 0; layer < plan.group(group).layer_group_count(); ++layer) {
            const double z = Slic3r::unscaled(plan.group(group).layer_group(layer).print_z());
            if (!found || z < first) {
                first = z;
                found = true;
            }
        }
    return !found || print_z <= first + 1e-6;
}

// Count the band lines exactly like the manager reservation (same rounded Flow and line spacing), build the zigzag
// and lay it out: the outgoing root receives the ramming steps from the start of the path, then the SEMM unload; the
// incoming root receives the SEMM load, then the rest of the path cut at every X line end, with the first four lines
// ramping from the starting speed to wipe_tower_speed and the whole ramp multiplied by the incoming filament purge
// factor (its filament_max_wipe_tower_speed percentage, or a fallback by filament type when that value is 0). The
// ramming, the SEMM unload and load and the tower maintenance keep their own speeds. The manager owns the
// speed-ratio save and restore commands that bracket the request. The flow is the one of the incoming tool, or of the
// outgoing tool for an outgoing-only request. The outgoing root asks for no pressure advance while it rams and
// unloads, and the incoming root asks for the incoming filament value; each request is made only when
// filament_pressure_advance is enabled for that tool. In SEMM, the unload and load travel along the band's work line: the line
// one band line above the end of the ramming, capped at the band top, or the band entry when nothing was rammed. That
// line is not printed yet, so the cooling and loading filament spreads over it instead of forming a blob, and the purge
// later resumes at the path cursor; the firmware inserts the travel between these non-contiguous leaves. Their XY speed
// is the travelled distance over the duration imposed by E, shortened to the machine ceiling when one applies.
// *ctx.exit receives the last point laid (the entry when nothing is laid). Missing roots or a band too shallow for the
// computed lines throw.
void generate(const Slic3r::PrintConfig &config, const ConstPrintingPlan &plan,
              const run_ctx_wipe_tower_wipe &ctx, const plugin_run_context *run)
{
    const c_wipe_tower_request &request = *ctx.request;
    const c_wipe_tower_band &band = *ctx.band;
    if ((request.has_incoming && band.incoming_root == nullptr) || (request.has_outgoing && band.outgoing_root == nullptr))
        throw std::invalid_argument("Tool-change wipe requires the roots of its enabled parts.");
    *ctx.exit = band.entry;
    const extruder_index_t incoming = request.has_incoming ?
        plan.group(request.incoming.group_idx).layer_group(request.incoming.layer_group_idx)
            .tool_group(request.incoming.tool_group_idx).extruder_id() : EXTRUDER_ID_UNKNOWN;
    const extruder_index_t outgoing = request.has_outgoing ?
        plan.group(request.outgoing.group_idx).layer_group(request.outgoing.layer_group_idx)
            .tool_group(request.outgoing.tool_group_idx).extruder_id() : EXTRUDER_ID_UNKNOWN;
    const extruder_index_t flow_tool = request.has_incoming ? incoming : outgoing;
    const Slic3r::Flow flow = Slic3r::Flow::new_from_width(float(band.extrusion_width),
        float(config.nozzle_diameter.get_at(size_t(flow_tool))), float(band.layer_height), 1.f, false);
    const double width = band.max.x - band.min.x;
    size_t lines = 0;
    if (request.purge_volume > 0.) {
        // The manager reserved the band with this pitch and the same rounded Flow, so the line
        // count below fills it exactly.
        lines = wipe_band_line_count(request.purge_volume, width, flow.mm3_per_mm(), band.line_spacing);
        if (band.max.y - band.min.y + 1e-6 < double(lines) * band.line_spacing)
            throw std::invalid_argument("Tool-change wipe band is too shallow for its rounded flow.");
    }
    PurgePath path = purge_path(band, lines);
    const bool semm = config.single_extruder_multi_material.value;
    const double cap = semm ? xy_speed_cap(config) : 0.;
    storage_handle *storage = run->plugin_storage;
    c_vec2d last = band.entry;
    // The work line starts at the band entry and moves one band line above the end of the ramming
    // when there is one, staying inside the reserved band.
    double work_y = band.entry.y;
    if (request.has_outgoing) {
        // Ramming and unloading move filament without printing, so the outgoing root asks for no
        // pressure advance when the outgoing filament enables the option; an unset value leaves
        // whatever the filament G-code wrote in place.
        if (config.filament_pressure_advance.is_enabled(int32_t(outgoing)))
            MutableExtrusionEntity(band.outgoing_root).get_or_add(EPropertySpeed::key).pressure_advance(0.f);
        bool rammed = false;
        for (const RammingStep &step : ramming_steps(config, outgoing)) {
            throw_if_cancelled(run);
            const std::vector<c_vec2d> points = take_path(path, step.volume / flow.mm3_per_mm(), false);
            if (points.size() < 2)
                continue;
            append_leaf(storage, band.outgoing_root, points, RAW_EXTRUSION_ROLE_WIPE_TOWER_RAMMING, flow,
                        step.volumetric_speed / flow.mm3_per_mm());
            last = points.back();
            rammed = true;
        }
        if (rammed)
            work_y = std::min(last.y + band.line_spacing, band.max.y);
        if (semm)
            append_unload(storage, band.outgoing_root, config, outgoing,
                          request.has_incoming ? incoming : EXTRUDER_ID_UNKNOWN,
                          is_first_plan_layer(plan, band.print_z), band, work_y, cap, last);
        append_command(storage, band.outgoing_root, C_EXTRUSION_SPECIAL_COMMAND_FLUSH_PLANNER_QUEUE, 0.);
    }
    if (request.has_incoming) {
        // Nothing else requests the normal pressure advance, so the incoming root asks for the
        // incoming filament value: the load and the purge print with it, and in a multi-nozzle
        // printer the rest of the visit keeps it.
        if (config.filament_pressure_advance.is_enabled(int32_t(incoming)))
            MutableExtrusionEntity(band.incoming_root).get_or_add(EPropertySpeed::key)
                .pressure_advance(float(config.filament_pressure_advance.get_at(size_t(incoming))));
        if (semm)
            append_load(storage, band.incoming_root, config, incoming, band, work_y, cap, last);
        const double main_speed = config.wipe_tower_speed.value > 0. ? config.wipe_tower_speed.value : 80.;
        const double configured_start = config.wipe_tower_wipe_starting_speed.get_effective_value(main_speed);
        const double start_speed = configured_start > 0. ? std::min(configured_start, main_speed) : main_speed;
        const double purge_factor = purge_speed_factor(config, incoming, band.print_z);
        for (size_t i = 0;; ++i) {
            throw_if_cancelled(run);
            const std::vector<c_vec2d> points = take_path(path, 0., true);
            if (points.size() < 2)
                break;
            const double speed = start_speed + (main_speed - start_speed) * double(std::min<size_t>(i, 3)) / 3.;
            append_leaf(storage, band.incoming_root, points, RAW_EXTRUSION_ROLE_WIPE_TOWER_WIPE, flow,
                        speed * purge_factor);
            last = points.back();
        }
    }
    *ctx.exit = last;
}

// Lines alternate from the side of the entry (left when the entry is in the left half). The path starts at the entry
// when it is not already the first line start, so a chained entry gives a short connector. Line i lies at
// y = band.min.y + i * band.line_spacing. The returned cursor is at the start of the path.
PurgePath purge_path(const c_wipe_tower_band &band, const size_t lines)
{
    PurgePath path;
    if (lines == 0)
        return path;
    const bool enter_right = band.entry.x > (band.min.x + band.max.x) / 2.;
    for (size_t i = 0; i < lines; ++i) {
        const bool right = enter_right != (i % 2 != 0);
        const double y = band.min.y + double(i) * band.line_spacing;
        const c_vec2d begin{right ? band.max.x : band.min.x, y};
        const c_vec2d end{right ? band.min.x : band.max.x, y};
        if (i == 0 && (band.entry.x != begin.x || band.entry.y != begin.y)) {
            path.points.push_back(band.entry);
            path.line_end.push_back(false);
        }
        path.points.push_back(begin);
        path.line_end.push_back(false);
        path.points.push_back(end);
        path.line_end.push_back(true);
    }
    return path;
}

// Return the points from the cursor to the point at the given length along the path (stop_at_line_end false), or to
// the next X line end (stop_at_line_end true), and move the cursor there. A cut inside a segment adds the interpolated
// point. At the end of the path fewer points (possibly none) are returned.
std::vector<c_vec2d> take_path(PurgePath &path, double length, const bool stop_at_line_end)
{
    std::vector<c_vec2d> out;
    if (path.segment + 1 >= path.points.size())
        return out;
    const auto lerp = [](const c_vec2d &a, const c_vec2d &b, double t) {
        return c_vec2d{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
    };
    const c_vec2d &first_a = path.points[path.segment];
    const c_vec2d &first_b = path.points[path.segment + 1];
    const double first_length = std::hypot(first_b.x - first_a.x, first_b.y - first_a.y);
    out.push_back(first_length > 0. ? lerp(first_a, first_b, path.offset / first_length) : first_a);
    while (path.segment + 1 < path.points.size()) {
        const c_vec2d &a = path.points[path.segment];
        const c_vec2d &b = path.points[path.segment + 1];
        const double segment_length = std::hypot(b.x - a.x, b.y - a.y);
        const double remaining = segment_length - path.offset;
        if (!stop_at_line_end && length < remaining - 1e-9) {
            path.offset += length;
            out.push_back(lerp(a, b, path.offset / segment_length));
            return out;
        }
        length -= remaining;
        out.push_back(b);
        ++path.segment;
        path.offset = 0.;
        if (stop_at_line_end ? path.line_end[path.segment] : length <= 1e-9)
            return out;
    }
    return out;
}

// Copy the local millimeter points as scaled points into a new non-sortable, non-reversible leaf with the role,
// rounded flow attributes and explicit speed, then move it to the end of the borrowed root. Append failures throw.
void append_leaf(storage_handle *storage, extrusion_entity_handle *root, const std::vector<c_vec2d> &points,
                 const int32_t role, const Slic3r::Flow &flow, const double speed)
{
    std::vector<c_point> scaled;
    for (const c_vec2d &point : points)
        scaled.push_back({Slic3r::scale_i(point.x), Slic3r::scale_i(point.y)});
    StoredExtrusionEntity leaf(storage);
    leaf.set_points(scaled);
    leaf.get_or_add(EPropertyAttributes::key).extrusion_role(role)
        .width(flow.width()).height(flow.height()).mm3_per_mm(flow.mm3_per_mm());
    leaf.get_or_add(EPropertySpeed::key).speed_mm_per_s = float(speed);
    leaf.disable_sort().disable_reverse();
    if (MutableExtrusionEntity(root).append_child_move(leaf.mutable_view()) == UINT32_MAX)
        throw std::runtime_error("Cannot append a tool-change wipe leaf.");
}

// Return the SEMM cooling and loading XY speed ceiling, or 0 when no ceiling applies. Machine limits set to ignore
// never shorten the travel. Otherwise both normal-mode axis feedrates cap the move, because the tower may be
// rotated, and 100 mm/s covers a missing or non-positive limit.
double xy_speed_cap(const Slic3r::PrintConfig &config)
{
    if (config.machine_limits_usage.value == Slic3r::MachineLimitsUsage::Ignore)
        return 0.;
    double cap = 0.;
    const Slic3r::ConfigOptionFloats *const axes[] = {&config.machine_max_feedrate_x, &config.machine_max_feedrate_y};
    for (const Slic3r::ConfigOptionFloats *axis : axes) {
        const std::vector<double> &values = axis->get_values();
        if (!values.empty() && std::isfinite(values.front()) && values.front() > 0.)
            cap = cap > 0. ? std::min(cap, values.front()) : values.front();
    }
    return cap > 0. ? cap : 100.;
}

// Append one filament move of distance mm at filament speed mm/s while the nozzle travels along its work line from
// position toward target_x, like WipeTower v1's load_move_x_advanced: the move lasts |distance| / speed, the XY speed
// is the travelled X distance over that duration, and a positive cap shortens the travel to cap * duration so the
// filament speed is never reduced. The leaf is geometric and carries EXTRUSION(distance) with a zero flow, so the
// firmware spreads the filament over the segment without dimension markers; a zero X distance becomes a point-less
// E-only leaf in place. position moves to the leaf end.
void append_x_move(storage_handle *storage, extrusion_entity_handle *root, c_vec2d &position,
                   const double target_x, const double distance, const double speed, const double cap)
{
    if (!std::isfinite(distance) || std::abs(distance) < 1e-6)
        return;
    const double filament_speed = std::isfinite(speed) && speed > 0. ? speed : k_fallback_filament_speed;
    const double duration = std::abs(distance) / filament_speed;
    const double direction = target_x > position.x ? 1. : (target_x < position.x ? -1. : 0.);
    double travel = std::abs(target_x - position.x);
    if (cap > 0. && travel > cap * duration)
        travel = cap * duration;
    if (direction == 0. || travel < 1e-6) {
        append_filament_move(storage, root, distance, filament_speed);
        return;
    }
    const c_vec2d end{position.x + direction * travel, position.y};
    std::vector<c_point> points{{Slic3r::scale_i(position.x), Slic3r::scale_i(position.y)},
                                {Slic3r::scale_i(end.x), Slic3r::scale_i(end.y)}};
    StoredExtrusionEntity leaf(storage);
    leaf.set_points(points);
    // A zero flow marks a pure filament move: no width, height or mm3 per mm, so the firmware
    // emits no dimension markers and spreads EXTRUSION(distance) over the segment.
    leaf.get_or_add(EPropertyAttributes::key).extrusion_role(RAW_EXTRUSION_ROLE_WIPE_TOWER)
        .width(0.f).height(0.f).mm3_per_mm(0.);
    leaf.get_or_add(EPropertySpeed::key).speed_mm_per_s = float(travel / duration);
    leaf.get_or_add(EPropertySpecialCommand::key).set(C_EXTRUSION_SPECIAL_COMMAND_EXTRUSION, distance);
    leaf.disable_sort().disable_reverse();
    if (MutableExtrusionEntity(root).append_child_move(leaf.mutable_view()) == UINT32_MAX)
        throw std::runtime_error("Cannot append a filament move along the band.");
    position = end;
}

// Add one heater entry event with the requested target and optional wait, then add the configured part-fan request; idx must select configured filament data, configuration is borrowed, and no entity ownership is transferred.
void request_toolchange_temperature(StoredExtrusionEntity &entity, const Slic3r::PrintConfig &config,
                                    const size_t idx, const bool wait)
{
    entity.get_or_add(EPropertyHeater::key).set_tool(
        int16_t(config.filament_toolchange_temp.get_at(idx)), EXTRUDER_ID_UNKNOWN, wait);
    if (config.filament_enable_toolchange_part_fan.get_at(idx))
        entity.get_or_add(EPropertySpeed::key).fan_speed(
            float(std::clamp(config.filament_toolchange_part_fan_speed.get_at(idx), 0, 100)));
}

// Add an incoming-filament heater entry event without waiting; tool must select configured filament data, the first plan layer uses first_layer_temperature and later layers use temperature, and no entity ownership is transferred.
void request_incoming_temperature(StoredExtrusionEntity &entity, const Slic3r::PrintConfig &config,
                                  const extruder_index_t tool, const bool first_layer)
{
    entity.get_or_add(EPropertyHeater::key).set_tool(int16_t(first_layer ?
        config.first_layer_temperature.get_at(size_t(tool)) : config.temperature.get_at(size_t(tool))));
}

// Port of the SEMM unload of WipeTower2 (and of the Prusa wipe tower it comes from), applied after the ramming:
//  - fast pull into the cooling tube: 15 mm at filament_unloading_speed_start, then the rest of
//    cooling_tube_retraction + cooling_tube_length / 2 in 70/20/10 % parts at 100/50/30 % of filament_unloading_speed;
//  - filament_cooling_moves back-and-forth filament moves of cooling_tube_length, their speed ramping from the
//    initial to the final cooling speed; each one travels along the work line toward the band edge farthest from
//    the cooling start X and back toward that X, so the filament spreads instead of forming a blob;
//  - optional skinnydip: dip of filament_skinnydip_distance, melt-zone pause, extraction, cooling-zone pause (ms);
//  - filament_toolchange_delay (s), then the move to the parking position, so the total pull equals
//    parking_pos_retraction (the sign error of WipeTower2 on this last move is not reproduced).
// Append borrowed configuration's SEMM unload phases to caller-owned storage
// and update the caller's borrowed position. Valid band geometry is required,
// generated children become storage-owned, and geometry/configuration failures
// propagate. The unload is split into successive sub-collections so each part
// carries its own heater event. With filament_enable_toolchange_temp, the pulls
// wait for filament_toolchange_temp (normal mode) or the cooling moves run while
// it is requested and the wait comes after them (fast skinnydip), with the
// toolchange part fan when enabled; the rest of the unload runs at the incoming
// filament temperature, added without waiting. A heater event remains in
// effect until another event or external G-code changes it, so the incoming
// target persists after the tool change in SEMM. Children are appended to the root, and the caller's
// position follows the cooling moves on the work line so the load continues from there.
// The unload collection disables retraction, because the unload is itself the retraction of the swap: CreateRetraction
// then adds no retract/unretract pair inside it or at the tool-change boundary.
void append_unload(storage_handle *storage, extrusion_entity_handle *root, const Slic3r::PrintConfig &config,
                   const extruder_index_t tool, const extruder_index_t incoming, const bool first_layer,
                   const c_wipe_tower_band &band, const double work_y, const double cap, c_vec2d &position)
{
    const size_t idx = size_t(tool);
    const bool toolchange_temp = config.filament_enable_toolchange_temp.get_at(idx);
    const bool fast_skinnydip = config.filament_use_fast_skinnydip.get_at(idx);
    const bool incoming_known = incoming != EXTRUDER_ID_UNKNOWN;
    // The cooling moves happen on the work line; the firmware connects the ramming end to it.
    position.y = work_y;
    StoredExtrusionEntity unload(storage);
    unload.disable_sort().disable_reverse();
    unload.get_or_add(EPropertyModifier::key).set_disable_retraction();
    extrusion_entity_handle *unload_handle = unload.mutable_handle();
    append_command(storage, unload_handle, C_EXTRUSION_SPECIAL_COMMAND_DISABLE_PREVIEW, 0.);
    const double tube_retraction = config.cooling_tube_retraction.value;
    const double tube_length = config.cooling_tube_length.value;
    // Pulls: in normal mode they wait for the toolchange temperature first, otherwise they run without
    // requesting one and the previous state stays in effect.
    StoredExtrusionEntity pulls(storage);
    pulls.disable_sort().disable_reverse();
    extrusion_entity_handle *pulls_handle = pulls.mutable_handle();
    if (toolchange_temp && !fast_skinnydip) {
        request_toolchange_temperature(pulls, config, idx, true);
    }
    if (tube_retraction != 0. || tube_length != 0.) {
        const double total = tube_retraction + tube_length / 2. - 15.;
        const double speed = config.filament_unloading_speed.get_at(idx);
        append_filament_move(storage, pulls_handle, -15., config.filament_unloading_speed_start.get_at(idx));
        append_filament_move(storage, pulls_handle, -0.70 * total, speed);
        append_filament_move(storage, pulls_handle, -0.20 * total, 0.5 * speed);
        append_filament_move(storage, pulls_handle, -0.10 * total, 0.3 * speed);
    }
    if (MutableExtrusionEntity(unload_handle).append_child_move(pulls.mutable_view()) == UINT32_MAX)
        throw std::runtime_error("Cannot append the filament unload pulls.");
    // Cooling moves and skinnydip: in fast skinnydip mode they request the toolchange temperature (the heater
    // moves toward it during the cooling moves) and wait for it after them, otherwise they run at the incoming
    // filament temperature.
    StoredExtrusionEntity swap(storage);
    swap.disable_sort().disable_reverse();
    extrusion_entity_handle *swap_handle = swap.mutable_handle();
    if (toolchange_temp && fast_skinnydip)
        request_toolchange_temperature(swap, config, idx, false);
    else if (incoming_known)
        request_incoming_temperature(swap, config, incoming, first_layer);
    const int moves = config.filament_cooling_moves.get_at(idx);
    if (moves > 0) {
        const double initial = config.filament_cooling_initial_speed.get_at(idx);
        const double increment = (config.filament_cooling_final_speed.get_at(idx) - initial) / (2. * moves - 1.);
        const double start_x = position.x;
        const double far_x = start_x - band.min.x < band.max.x - start_x ? band.max.x : band.min.x;
        for (int i = 0; i < moves; ++i) {
            append_x_move(storage, swap_handle, position, far_x, tube_length, initial + increment * 2. * i, cap);
            append_x_move(storage, swap_handle, position, start_x, -tube_length, initial + increment * (2. * i + 1.), cap);
        }
    }
    if (toolchange_temp && fast_skinnydip)
        append_command(storage, swap_handle, C_EXTRUSION_SPECIAL_COMMAND_WAIT_FOR_TEMP, 0.);
    if (config.filament_use_skinnydip.get_at(idx)) {
        const double dip = config.filament_skinnydip_distance.get_at(idx);
        append_filament_move(storage, swap_handle, dip, config.filament_dip_insertion_speed.get_at(idx));
        append_command(storage, swap_handle, C_EXTRUSION_SPECIAL_COMMAND_PAUSE, config.filament_melt_zone_pause.get_at(idx));
        append_filament_move(storage, swap_handle, -dip, config.filament_dip_extraction_speed.get_at(idx));
        append_command(storage, swap_handle, C_EXTRUSION_SPECIAL_COMMAND_PAUSE, config.filament_cooling_zone_pause.get_at(idx));
    }
    if (MutableExtrusionEntity(unload_handle).append_child_move(swap.mutable_view()) == UINT32_MAX)
        throw std::runtime_error("Cannot append the filament unload cooling moves.");
    // The toolchange delay and the parking move run at the incoming filament temperature.
    StoredExtrusionEntity parking(storage);
    parking.disable_sort().disable_reverse();
    extrusion_entity_handle *parking_handle = parking.mutable_handle();
    if (incoming_known)
        request_incoming_temperature(parking, config, incoming, first_layer);
    append_command(storage, parking_handle, C_EXTRUSION_SPECIAL_COMMAND_PAUSE,
                   1000. * config.filament_toolchange_delay.get_at(idx));
    append_filament_move(storage, parking_handle,
                         -(config.parking_pos_retraction.value - tube_retraction - tube_length / 2.), 33.);
    if (MutableExtrusionEntity(unload_handle).append_child_move(parking.mutable_view()) == UINT32_MAX)
        throw std::runtime_error("Cannot append the filament unload parking.");
    append_command(storage, unload_handle, C_EXTRUSION_SPECIAL_COMMAND_ENABLE_PREVIEW, 0.);
    if (MutableExtrusionEntity(root).append_child_move(unload.mutable_view()) == UINT32_MAX)
        throw std::runtime_error("Cannot append the filament unload.");
}

// Port of the SEMM load of WipeTower2: when parking_pos_retraction or extra_loading_move is set, push the filament
// back by their sum in 20/70/10 % parts at filament_loading_speed_start, filament_loading_speed and 10 % of it, with
// the high extruder current (750 then 550) around it when high_current_on_filament_swap is set. The first part stays
// in place (E-only); the fast part travels along the work line toward the band edge farthest from the current X and
// the last part returns toward that X, so the filament spreads instead of forming a blob. Children are appended to
// the root inside one collection that disables retraction, because the load is itself the restore of the swap:
// CreateRetraction then adds no retract/unretract pair inside it or at the tool-change boundary. The caller's
// position follows the travelling parts.
void append_load(storage_handle *storage, extrusion_entity_handle *root, const Slic3r::PrintConfig &config,
                 const extruder_index_t tool, const c_wipe_tower_band &band, const double work_y, const double cap,
                 c_vec2d &position)
{
    const size_t idx = size_t(tool);
    const double distance = config.parking_pos_retraction.value + config.extra_loading_move.value;
    if (distance == 0.)
        return;
    // The load moves happen on the work line; the firmware connects the nozzle position to it.
    position.y = work_y;
    StoredExtrusionEntity load(storage);
    load.disable_sort().disable_reverse();
    load.get_or_add(EPropertyModifier::key).set_disable_retraction();
    extrusion_entity_handle *load_handle = load.mutable_handle();
    const bool high_current = config.high_current_on_filament_swap.value;
    if (high_current)
        append_command(storage, load_handle, C_EXTRUSION_SPECIAL_COMMAND_EXTRUDER_CURRENT, 750.);
    append_command(storage, load_handle, C_EXTRUSION_SPECIAL_COMMAND_DISABLE_PREVIEW, 0.);
    const double speed = config.filament_loading_speed.get_at(idx);
    append_filament_move(storage, load_handle, 0.2 * distance, config.filament_loading_speed_start.get_at(idx));
    const double start_x = position.x;
    const double far_x = start_x - band.min.x < band.max.x - start_x ? band.max.x : band.min.x;
    append_x_move(storage, load_handle, position, far_x, 0.7 * distance, speed, cap);
    append_x_move(storage, load_handle, position, start_x, 0.1 * distance, 0.1 * speed, cap);
    append_command(storage, load_handle, C_EXTRUSION_SPECIAL_COMMAND_ENABLE_PREVIEW, 0.);
    if (high_current)
        append_command(storage, load_handle, C_EXTRUSION_SPECIAL_COMMAND_EXTRUDER_CURRENT, 550.);
    if (MutableExtrusionEntity(root).append_child_move(load.mutable_view()) == UINT32_MAX)
        throw std::runtime_error("Cannot append the filament load.");
}

// Pauses (PAUSE) with a non-positive duration are skipped. A positive speed is stored as the leaf speed (mm/s of
// filament for E-only moves). The leaf has no point, so the firmware emits it at the current nozzle position.
void append_command(storage_handle *storage, extrusion_entity_handle *root, const c_extrusion_special_command code,
                    const double value, const double speed)
{
    if (code == C_EXTRUSION_SPECIAL_COMMAND_PAUSE && !(value > 0.))
        return;
    StoredExtrusionEntity command(storage);
    command.get_or_add(EPropertySpecialCommand::key).set(code, value);
    if (speed > 0.)
        command.get_or_add(EPropertySpeed::key).speed_mm_per_s = float(speed);
    command.disable_sort().disable_reverse();
    if (MutableExtrusionEntity(root).append_child_move(command.mutable_view()) == UINT32_MAX)
        throw std::runtime_error("Cannot append a tool-change command.");
}

// A non-positive or non-finite configured speed falls back to k_fallback_filament_speed, so the move keeps a valid
// feed rate.
void append_filament_move(storage_handle *storage, extrusion_entity_handle *root, const double distance, const double speed)
{
    if (!std::isfinite(distance) || std::abs(distance) < 1e-6)
        return;
    append_command(storage, root, C_EXTRUSION_SPECIAL_COMMAND_EXTRUSION, distance,
                   std::isfinite(speed) && speed > 0. ? speed : k_fallback_filament_speed);
}
} // namespace

// Register one process-lifetime service, lending its C instance to the host; registration failures are reported by the host and callback storage stays host-owned.
void register_tool_change_wipe_plugin(orchestrator_handle *orchestrator)
{
    static ToolChangeWipe plugin(orchestrator);
    orchestrator_register_plugin(orchestrator, plugin.c_instance());
}
} // namespace slic3r_api::WipeTower
