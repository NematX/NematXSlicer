///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
/*
Short-layer slowdown plugin (`layer_extrusion_edit.speed.slowdown`)
===================================================================

Purpose
-------
A layer that prints too fast cannot cool: the next layer lands on hot plastic.
When a layer would last less than `slowdown_below_layer_time` seconds, this
plugin lowers the speed of its extrusions so that the layer lasts that long, if
possible, never going under `min_print_speed` nor beyond `max_speed_reduction`.
It replaces the slowdown that the legacy CoolingBuffer applied on parsed G-code,
and it is on by default (the default threshold is 5 s), as in the legacy slicer.

Vocabulary
----------
- leaf / sheet: an extrusion entity that owns segments (a path). All its
  segments share one effective speed. One leaf is the equivalent of one legacy
  `_EXTRUDE_SET_SPEED` ... `_EXTRUDE_END` block, which is the unit the legacy
  code slowed down.
- unassigned time: fixed stationary deposits and synthetic travels between
  leaves or entering the layer. It counts in the duration but is never slowed.
- RESOLVED duration: a PrintingLayerTimeProperty whose origin is RESOLVED. It
  says "this is the layer duration with the final speeds"; FanCooling prefers
  it over its own estimate, so the fan follows the duration AFTER the slowdown.

How it works
------------
1. setup() (serialized, reads the whole plan): if no extruder of the plan has a
   positive threshold, the plugin stays inactive. Otherwise it measures every
   layer once with PrintingPlanTimeEstimator::estimate_detailed(). This must be
   done here because the position entering a layer depends on the previous
   layer. The measurement is cached per layer.
2. run() (parallel, one layer at a time): walks the layer's trees in the same
   order as the estimator to pair every leaf with its measured duration and to
   decide whether the leaf is slowable (role table below). The plain numbers go
   to SlowdownSolver::solve(); the new speeds are written back with
   EPropertySpeedFieldEditor (only `speed_mm_per_s` changes, acceleration and
   pressure advance of an existing property are kept), and the layer duration
   computed by the solver is published as RESOLVED.

The plugin never splits, merges, reorders or moves anything: it only writes the
speed field of existing leaves, so the position entering the next layer is
unchanged. A layer where no extruder has a positive threshold is left alone and
publishes nothing, so FanCooling keeps estimating it.

Priority 16: after the host's speed editors (DefaultSpeed 0,
DefaultAcceleration 10, DefaultTravelDeceleration 15) and before
the first fan policy (20). A plugin that changes speeds after this priority makes
the RESOLVED duration wrong.

    setup_impl(): cache ordered timing without editing the plan
        `-- estimate_detailed(): separate geometry from fixed deposits and gaps
    run_impl(): edit only the current layer's speeds and duration
        |-- collect_layer() -> collect_tree(): pair geometric leaves with timing
        |-- SlowdownSolver::solve(): equalize fastest speeds within tool bounds
        `-- write_speeds(): preserve all fields except speed_mm_per_s

The host owns the provider and plan; cached details own numbers, not tree views.
The cache is rebuilt in serialized setup and read by disjoint layer callbacks.
Invalid geometry, changed leaf layout and cancellation propagate as errors.
The numeric solver uses current durations between tool thresholds and caps each
gain at its maximum duration; these differ from the legacy CoolingBuffer model.

Slowable roles (copied from the legacy `cooldown_marker_init`): perimeter,
external perimeter, internal/solid/top solid infill, ironing, internal bridge
infill, thin wall, gap fill, skirt/brim, support, support interface. Overhang
perimeters, bridge infill, wipe tower, milling, travels, wipes and any unknown
role keep their speed but still count in the duration. (The legacy table also
listed "Custom", which only exists for custom G-code and never as a plan leaf.)
*/
#include "CoolingSlowdown.hpp"

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <map>
#include <set>
#include <stdexcept>
#include <vector>

#include "ExtrusionProcessParameterHelpers.hpp"
#include "SlowdownSolver.hpp"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingLayerTimeProperty.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanTimeEstimator.hpp"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace CoolingSlowdownPlugin {
namespace {
using namespace ProcessParameterHelpers;

// Runs after the host speed editors (max 15) and before the fan policies (20).
constexpr int32_t k_priority = 16;
// Defaults of the settings, used only if an option is missing from the config.
constexpr double k_default_min_print_speed = 10.0;
// Percent options are read as ratios (90 % reads 0.9).
constexpr double k_default_max_speed_reduction = 0.9;

const char *const k_no_dependencies[] = {nullptr};
const raw_used_config_key k_used[] = {
    {"max_speed_reduction", RAW_CO_VECTOR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"min_print_speed", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"slowdown_below_layer_time", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"travel_speed", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
};

// Raw roles whose speed may be lowered (see the header comment). Roles are
// compared as raw values: a value that is not in this table stays fixed, even
// when it carries a modifier bit of a listed role (for example a perimeter
// that is also marked solid), and any unknown value is safe (no assertion).
// The legacy table also listed "Custom" (custom G-code, never a plan leaf) and
// the AroundHoles infill travels (no role of their own in v3: they are plain
// infill leaves and follow the infill rule), so neither needs an entry.
const raw_extrusion_role k_slowable_roles[] = {
    RAW_EXTRUSION_ROLE_PERIMETER, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER, RAW_EXTRUSION_ROLE_INTERNAL_INFILL,
    RAW_EXTRUSION_ROLE_SOLID_INFILL, RAW_EXTRUSION_ROLE_TOP_SOLID_INFILL, RAW_EXTRUSION_ROLE_IRONING_INFILL,
    RAW_EXTRUSION_ROLE_INTERNAL_BRIDGE_INFILL, RAW_EXTRUSION_ROLE_THIN_WALL, RAW_EXTRUSION_ROLE_GAP_FILL,
    RAW_EXTRUSION_ROLE_SKIRT, RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL, RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE
};

// Where a slowable leaf lives, to write its new speed after solving.
struct LeafTarget
{
    MutableExtrusionEntity leaf;
    size_t extruder_slot = 0;
    size_t line_idx = 0;
};

// Everything gathered from one layer before the solver runs.
struct LayerLines
{
    SlowdownSolver::Layer solver_layer;
    std::vector<LeafTarget> targets;
    // True when at least one extruder of the layer has a positive threshold.
    bool any_threshold = false;
};

// State shared by the recursive tree walk of collect_tree().
struct TreeWalk
{
    LayerLines &lines;
    const PrintingLayerTimeDetail &detail;
    size_t extruder_slot = 0;
    uint32_t tool_idx = 0;
    uint32_t extrusion_idx = 0;
    // Rank of the next leaf of the current extrusion.
    uint32_t next_rank = 0;
    // Index of the next unread entry of detail.leaves.
    size_t next_leaf = 0;
};

class CoolingSlowdown : public PluginBase {
public:
    // Retain the timing key owned by this host; no plan view is stored here.
    explicit CoolingSlowdown(orchestrator_handle *orch) : PluginBase(orch),
        m_time(printing_layer_time_property_key(orch)) {}
private:
    // Return this composable provider's stable identity.
    const char *id_impl() const noexcept override { return "layer_extrusion_edit.speed.slowdown"; }
    // Return borrowed display metadata.
    const char *name_impl() const noexcept override { return "Short-layer slowdown"; }
    // Composes with the speed editors: it only post-processes their result.
    const char *exclusive_group_impl() const noexcept override { return ""; }
    // Describe the speed-only transformation and its timing result.
    const char *description_impl() const noexcept override {
        return "Lower print speeds on layers shorter than slowdown_below_layer_time and publish the resulting layer duration.";
    }
    // Edit ordered plan clones, not the source toolpaths.
    slicing_step_t step_impl() const noexcept override { return STEP_LAYER_EXTRUSION_EDIT; }
    // Compose through priority and public timing rather than provider IDs.
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    // Complete speed resolution before fan policy timing is consumed.
    int32_t priority_impl() const noexcept override { return k_priority; }
    // Copy borrowed input declarations, or return their count for null output.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override {
        if (keys) std::copy(std::begin(k_used), std::end(k_used), keys);
        return int32_t(std::size(k_used));
    }
    // All the settings are defined by the host (FFFPrintConfig).
    int32_t defined_config_keys(const char **) const noexcept override { return 0; }
    // Host-owned definitions require no additional initialization.
    void inilialize_impl(storage_handle *) const override {}
    // Return the borrowed per-tree progress format.
    const char *progress_message_format_impl() const noexcept override { return "Slowing down short layers: %u / %u trees"; }
    // Clear stale data and cache a whole-plan estimate when any threshold is enabled.
    // Borrow the plan read-only; propagate cancellation and timing-model errors.
    void setup_impl(const plugin_run_context *run, uint32_t) const override;
    // Count this layer's extrusion trees only when setup enabled the provider.
    void setup_run_impl(const plugin_run_context *run) const override;
    // Pair this layer's geometric leaves, solve, and publish RESOLVED even when
    // no speed changes. Disabled layers publish nothing. Borrowed handles do not
    // escape the call; layout mismatches, invalid inputs and cancellation throw.
    void run_impl(const plugin_run_context *run) const override;
    PluginPropertyKey<PrintingLayerTimeProperty> m_time;
    // Written by setup() only, read by the parallel runs.
    mutable bool m_active = false;
    mutable std::vector<std::vector<PrintingLayerTimeDetail>> m_details;
};

// Threshold of one extruder, 0 when disabled or missing.
double threshold_seconds(const Config &config, extruder_index_t extruder_id);
// True when at least one tool group of the plan has a positive threshold.
bool plan_has_threshold(const Config &config, const PrintingPlan &plan, const plugin_run_context *run);
// Gather one layer in estimator order, merging repeated tool visits by sorted
// extruder ID so equal thresholds resolve deterministically. Missing vector
// entries use the option's default; invalid timing layout and cancellation throw.
LayerLines collect_layer(const Config &config, PrintingLayerGroup &layer,
                         const PrintingLayerTimeDetail &detail, const plugin_run_context *run);
// Pair geometric nodes in estimator order (children, then own segments), without
// editing the plan. Only childless slowable nodes become borrowed write targets.
// Count/rank mismatches throw; no structural edit may occur between measurement,
// pairing and writing, so the targets remain valid for this layer callback.
void collect_tree(MutableExtrusionEntity entity, const EffectiveTreeState &parent, TreeWalk &walk);
// Table lookup of k_slowable_roles.
bool is_slowable_role(raw_extrusion_role role);
// Write only slowed leaf speeds. Preserve other direct process fields, or use
// unset fields on a new property so inherited acceleration/pressure remain intact.
void write_speeds(const LayerLines &lines, const SlowdownSolver::Result &result);

double threshold_seconds(const Config &config, extruder_index_t extruder_id)
{
    return config.vector_float_or_default("slowdown_below_layer_time", uint32_t(extruder_id), 0.0);
}

bool plan_has_threshold(const Config &config, const PrintingPlan &plan, const plugin_run_context *run)
{
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        const ConstPrintingGroup group = plan.group(group_idx);
        for (uint32_t idx = 0; idx < group.layer_group_count(); ++idx) {
            throw_if_cancelled(run);
            const ConstPrintingLayerGroup layer = group.layer_group(idx);
            for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx)
                if (threshold_seconds(config, layer.tool_group(tool_idx).extruder_id()) > 0.0)
                    return true;
        }
    }
    return false;
}

bool is_slowable_role(raw_extrusion_role role)
{
    return std::find(std::begin(k_slowable_roles), std::end(k_slowable_roles), role) != std::end(k_slowable_roles);
}

void collect_tree(MutableExtrusionEntity entity, const EffectiveTreeState &parent, TreeWalk &walk)
{
    const EffectiveTreeState state = effective_state(entity, parent);
    for (uint32_t idx = 0; idx < entity.child_count(); ++idx)
        collect_tree(entity.child_mutable(idx), state, walk);
    if (entity.segment_count() == 0)
        return;

    // The estimator numbered its leaves with the same rule; a mismatch means the
    // plan changed between setup() and run(), which must never go unnoticed.
    if (walk.next_leaf >= walk.detail.leaves.size())
        throw std::runtime_error("Slowdown found a leaf that was not measured.");
    const PrintingLeafTime &measured = walk.detail.leaves[walk.next_leaf++];
    if (measured.tool_idx != walk.tool_idx || measured.extrusion_idx != walk.extrusion_idx ||
        measured.leaf_rank != walk.next_rank++)
        throw std::runtime_error("Slowdown measurements do not match the plan layout.");

    // A node that also has children is not a path of its own: writing a speed
    // on it would change its children, so its time stays fixed.
    SlowdownSolver::Line line;
    line.seconds = measured.duration_seconds;
    line.speed_mm_per_s = measured.speed_mm_per_s;
    line.adjustable = entity.child_count() == 0 && is_slowable_role(state.attributes.extrusion_role());
    SlowdownSolver::Extruder &extruder = walk.lines.solver_layer.extruders[walk.extruder_slot];
    extruder.lines.push_back(line);
    if (line.adjustable)
        walk.lines.targets.push_back(LeafTarget{entity, walk.extruder_slot, extruder.lines.size() - 1});
}

LayerLines collect_layer(const Config &config, PrintingLayerGroup &layer,
                         const PrintingLayerTimeDetail &detail, const plugin_run_context *run)
{
    LayerLines lines;
    lines.solver_layer.unassigned_seconds = detail.unattributed_seconds;

    // One solver extruder per extruder id (a tool can be visited several times
    // in a layer); ids are sorted so that equal thresholds resolve identically.
    std::set<extruder_index_t> ids;
    for (uint32_t idx = 0; idx < layer.tool_group_count(); ++idx)
        ids.insert(layer.tool_group(idx).extruder_id());
    std::map<extruder_index_t, size_t> slots;
    for (extruder_index_t id : ids) {
        SlowdownSolver::Extruder extruder;
        extruder.threshold_seconds = threshold_seconds(config, id);
        extruder.min_print_speed = config.vector_float_or_default("min_print_speed", uint32_t(id), k_default_min_print_speed);
        extruder.max_speed_reduction = config.vector_percent_or_default(
            "max_speed_reduction", uint32_t(id), k_default_max_speed_reduction);
        lines.any_threshold = lines.any_threshold || extruder.threshold_seconds > 0.0;
        slots.emplace(id, lines.solver_layer.extruders.size());
        lines.solver_layer.extruders.push_back(std::move(extruder));
    }

    TreeWalk walk{lines, detail};
    for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
        PrintingToolGroup tool = layer.tool_group(tool_idx);
        walk.tool_idx = tool_idx;
        walk.extruder_slot = slots.at(tool.extruder_id());
        for (uint32_t extrusion = 0; extrusion < tool.extrusion_count(); ++extrusion) {
            throw_if_cancelled(run);
            walk.extrusion_idx = extrusion;
            walk.next_rank = 0;
            collect_tree(tool.extrusion(extrusion).mutable_root(), EffectiveTreeState{}, walk);
        }
    }
    if (walk.next_leaf != detail.leaves.size())
        throw std::runtime_error("Slowdown measured leaves that the plan no longer contains.");
    return lines;
}

void write_speeds(const LayerLines &lines, const SlowdownSolver::Result &result)
{
    // The editor keeps an existing EPropertySpeed (acceleration, pressure
    // advance) and only replaces its speed; a leaf that inherited its speed
    // gets a new property with the other fields unset (-1).
    EPropertySpeedFieldEditor editor(EPropertySpeedField::Speed);
    for (const LeafTarget &target : lines.targets) {
        const SlowdownSolver::LineResult &line = result.lines[target.extruder_slot][target.line_idx];
        if (line.slowed)
            editor.set_value(target.leaf, line.speed_mm_per_s);
    }
}

void CoolingSlowdown::setup_impl(const plugin_run_context *run, uint32_t) const
{
    throw_if_cancelled(run);
    m_active = false;
    m_details.clear();
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run);
    if (!ctx || !ctx->print || !ctx->plan) return;
    const Print print(ctx->print);
    const PrintingPlan plan(ctx->plan);
    const Config config = print.config();
    // Nothing to do (and no time spent estimating) when every threshold is 0.
    if (!plan_has_threshold(config, plan, run)) return;

    // Measure the whole plan once: the entry position of a layer depends on the
    // previous layers, so this cannot be done inside the parallel runs.
    std::vector<PrintingLayerTimeDetail> details = PrintingPlanTimeEstimator(config).estimate_detailed(plan);
    m_details.resize(plan.group_count());
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx)
        m_details[group_idx].resize(plan.group(group_idx).layer_group_count());
    for (PrintingLayerTimeDetail &detail : details) {
        throw_if_cancelled(run);
        m_details.at(detail.group_idx).at(detail.layer_group_idx) = std::move(detail);
    }
    m_active = true;
}

void CoolingSlowdown::setup_run_impl(const plugin_run_context *run) const
{
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run);
    if (m_active && ctx && ctx->layer_group)
        progress().add_max(ProcessParameterHelpers::extrusion_tree_count(PrintingLayerGroup(ctx->layer_group)));
}

void CoolingSlowdown::run_impl(const plugin_run_context *run) const
{
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run);
    if (!m_active || !ctx || !ctx->print || !ctx->layer_group) return;
    throw_if_cancelled(run);
    const Config config = Print(ctx->print).config();
    PrintingLayerGroup layer(ctx->layer_group);
    LayerLines lines = collect_layer(config, layer, m_details.at(ctx->group_idx).at(ctx->layer_group_idx), run);
    for (uint32_t idx = 0; idx < ProcessParameterHelpers::extrusion_tree_count(layer); ++idx)
        progress().increment();
    // Layers where every threshold is 0 are neither slowed nor given a duration.
    if (!lines.any_threshold) return;

    const SlowdownSolver::Result result = SlowdownSolver::solve(lines.solver_layer);
    write_speeds(lines, result);
    PrintingLayerTimeProperty &time = m_time.get_or_add(layer.properties());
    time.duration_seconds = result.total_seconds;
    time.origin = RAW_PRINTING_LAYER_TIME_ORIGIN_RESOLVED;
    time.validate();
}
} // namespace

// Registers the single plugin instance with the orchestrator; the instance
// lives for the whole process, as for the other built-in plugins.
void register_cooling_slowdown_plugin(orchestrator_handle *orch)
{
    static CoolingSlowdown plugin(orch);
    orchestrator_register_plugin(orch, plugin.c_instance());
}
}}} // namespace slic3r_api::LayerExtrusionEdit::CoolingSlowdownPlugin
