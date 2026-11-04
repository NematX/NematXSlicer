///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "FixedDirectionSeamPlacer.hpp"
#include "SeamInitializationCancellation.hpp"
#include "SeamPlacementConstraints.hpp"

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/SeamPlacerViews.hpp"

/*
Fixed-direction seam placement
==============================

Without applicable constraints this provider chooses a stored loop vertex.
Painted and volume constraints restrict eligible line/arc portions. The
configured compass direction defines the primary projection. A clockwise
perpendicular projection breaks ties. Analytic constrained candidates follow
portion/tree order and sorted curve-parameter order for exact ties; original
tree/point order applies to the select_vertex() fallback. Constraint boundaries
and directional extrema provide extra candidates on true segments and arcs; no
chord approximation is used. Blockers override enforcers. Free portions are
the second choice; fully blocked loops fall back to stored vertices.

The direction is read from the LayerRegion passed with every request, so one
immutable session serves a complete PrintingPlan even when its regions use
different settings. The provider ignores the incoming tool position because
its policy is geometric rather than travel based.
*/

/*
The provider has a preparation path and a request path:

    FixedDirectionSeamPlacer::run_impl()
    `-- FixedDirectionSession::initialize()
        `-- SeamPlacementConstraints::initialize()

    FixedDirectionSession::place_seam()
    |-- read the regional enum
    |-- portions() and recheck rounded candidates
    |-- compare status and directional projections
    `-- select_vertex() if no constrained candidate survives
*/
namespace slic3r_api { namespace SeamPlacement { namespace FixedDirectionPlugin {
namespace {

const char *const k_plugin_id = "seam_placer.fixed";
const char *const k_group_id = "seam_placer_plugin";
const char *const k_direction_key = "seam_fixed_direction";
const char *const k_no_dependencies[] = { nullptr };

/*
The array order is part of the runtime contract: ConfigOptionEnum stores the
zero-based index, so it must remain aligned with FixedDirection and
direction_vector(). The strings are the stable serialized profile values.
*/
const key_value_string_pair_t k_direction_values[] = {
    { "rear", "Rear (+Y)" },
    { "front", "Front (-Y)" },
    { "right", "Right (+X)" },
    { "left", "Left (-X)" },
    { "rear_right", "Rear right (+Y +X)" },
    { "rear_left", "Rear left (+Y -X)" },
    { "front_right", "Front right (-Y +X)" },
    { "front_left", "Front left (-Y -X)" }
};

const raw_used_config_key k_used_config_keys[] = {
    { k_direction_key, RAW_CO_ENUM, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT }
};

enum class FixedDirection : int32_t {
    Rear = 0,
    Front,
    Right,
    Left,
    RearRight,
    RearLeft,
    FrontRight,
    FrontLeft
};

struct DirectionVector {
    // Integer signs represent a compass direction in scaled XY coordinates.
    int32_t x;
    int32_t y;
};

struct SelectedVertex {
    /* Updated during one depth-first traversal; strict comparisons preserve ties. */
    bool valid = false;
    c_point point = {};
    long double primary = -std::numeric_limits<long double>::infinity();
    long double secondary = -std::numeric_limits<long double>::infinity();
};

class FixedDirectionSession;
class FixedDirectionSeamPlacer;

/* Convert one enum index into its primary projection vector, defaulting to Rear. */
DirectionVector direction_vector(int32_t direction);

/*
Compare every stored vertex recursively and retain the deterministic best.

The traversal visits local points before children and children by index. Since
exact ties do not replace selected, this order is the final stable tie-break.
*/
void select_vertex(const ExtrusionEntity &entity,
                   DirectionVector direction,
                   SelectedVertex &selected);

class FixedDirectionSession final : public SeamPlacerSession
{
public:
    explicit FixedDirectionSession(
        storage_handle *storage,
        const plugin_run_context *run_context = nullptr) :
        m_storage(storage), m_run_context(run_context) {}
    /* Prepare immutable constraint caches before publishing the session. */
    void initialize(const PrintingPlan &plan,
                    const std::vector<PrintRegion> &print_regions) override;

    /* Every region uses the same fixed-direction policy rank. */
    float get_priority(const ExtrusionEntity &loop,
                       const LayerRegion &layer_region) const override;

    /* Select the directional best admissible point without changing geometry. */
    c_point place_seam(const ExtrusionEntity &loop,
                       const LayerRegion &layer_region,
                       c_point start_position) const override;
private:
    storage_handle *m_storage;
    const plugin_run_context *m_run_context;
    SeamPlacementConstraints m_constraints;
};

class FixedDirectionSeamPlacer final : public PluginBase
{
public:
    static FixedDirectionSeamPlacer &instance(orchestrator_handle *orchestrator);
    explicit FixedDirectionSeamPlacer(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

private:
    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    const char *exclusive_group_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    // Return the fixed execution priority for seam-placement jobs.
    int32_t priority_impl() const noexcept override;
    // Return the fixed rank among seam-placement alternatives.
    int32_t exclusive_group_priority_impl() const noexcept override;
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    int32_t defined_config_keys(const char **keys) const noexcept override;
    void inilialize_impl(storage_handle *storage) const override;
    void run_impl(const plugin_run_context *run_context) const override;
};

DirectionVector direction_vector(const int32_t direction)
{
    // Configuration stores a zero-based enum index. Invalid values use the
    // documented Rear direction so the result remains usable.
    /* Keep this table aligned with k_direction_values and FixedDirection. */
    static const std::array<DirectionVector, 8> directions = {{
        { 0, 1 }, { 0, -1 }, { 1, 0 }, { -1, 0 },
        { 1, 1 }, { -1, 1 }, { 1, -1 }, { -1, -1 }
    }};
    if (direction < 0 || direction >= static_cast<int32_t>(directions.size()))
        return directions[static_cast<size_t>(FixedDirection::Rear)];
    return directions[static_cast<size_t>(direction)];
}

void select_vertex(const ExtrusionEntity &entity,
                   const DirectionVector direction,
                   SelectedVertex &selected)
{
    // Compare every local point before visiting children. Strict comparisons
    // make traversal order the final deterministic tie-break.
    // Rotating (x, y) clockwise gives (y, -x). It supplies the documented
    // secondary direction, for example Rear (+Y) breaks ties toward Right.
    const DirectionVector clockwise = { direction.y, -direction.x };
    for (uint32_t point_idx = 0; point_idx < entity.point_count(); ++point_idx) {
        const c_point point = entity.point(point_idx);
        const long double primary = static_cast<long double>(point.x) * direction.x +
                                    static_cast<long double>(point.y) * direction.y;
        const long double secondary = static_cast<long double>(point.x) * clockwise.x +
                                      static_cast<long double>(point.y) * clockwise.y;
        if (!selected.valid || primary > selected.primary ||
            (primary == selected.primary && secondary > selected.secondary)) {
            selected.valid = true;
            selected.point = point;
            selected.primary = primary;
            selected.secondary = secondary;
        }
    }

    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        select_vertex(entity.child(child_idx), direction, selected);
}

void FixedDirectionSession::initialize(const PrintingPlan &plan,
                                       const std::vector<PrintRegion> &)
{
    // Initialization copies all shapes and releases temporary storage before
    // queries can run concurrently. Only borrowed plan identities are retained.
    if (!plan.valid())
        throw std::invalid_argument("Fixed-direction seam placement needs a valid PrintingPlan.");
    InitializationCancellation::check(m_run_context);
    m_constraints.initialize(plan, m_storage, m_run_context);
    m_storage = nullptr;
    m_run_context = nullptr;
}

float FixedDirectionSession::get_priority(const ExtrusionEntity &,
                                          const LayerRegion &) const
{
    // Fixed direction has no preference between overlapping regions. The
    // router therefore keeps the source island's deterministic region order.
    return 50.f;
}

c_point FixedDirectionSession::place_seam(const ExtrusionEntity &loop,
                                          const LayerRegion &layer_region,
                                          const c_point) const
{
    // Candidate points are reclassified after integer rounding because a
    // rounded boundary point can change its constraint status.
    // The incoming position is intentionally unnamed: unlike travel-based
    // policies, this provider must return the same vertex from any approach.
    if (!loop.is_loop())
        throw std::invalid_argument(
            "Fixed-direction seam placement needs a non-empty fixed continuous closed loop.");
    if (!layer_region.valid())
        throw std::invalid_argument(
            "Fixed-direction seam placement needs a LayerRegion with a PrintRegion.");

    // LayerRegion references the source PrintRegion whose Config contains the
    // effective per-region value. No configuration is cached in the session,
    // so different regions may use different directions in the same plan.
    const Config config = layer_region.print_region().config();
    const int32_t configured_direction = config.enum_or_default(
        k_direction_key, int32_t(FixedDirection::Rear));
    SelectedVertex selected;
    const DirectionVector direction = direction_vector(configured_direction);
    const SeamPlacementConstraints::Result constraints = m_constraints.portions(loop, layer_region);
    SeamPlacementConstraints::Status best_status = SeamPlacementConstraints::Status::Blocked;
    if (constraints.relevant) {
        for (const SeamPlacementConstraints::Portion &portion : constraints.portions) {
            if (portion.status == SeamPlacementConstraints::Status::Blocked) continue;
            const SeamPlacementConstraints::Curve &curve = portion.curve;
            const double length = curve.radius > 0 ? std::abs(curve.radius * curve.sweep) :
                std::hypot(double(curve.segment.point_b.x) - curve.segment.point_a.x,
                           double(curve.segment.point_b.y) - curve.segment.point_a.y);
            const double inset = length > 0 ? std::min((portion.end - portion.begin) / 2, 2 / length) : 0;
            const double begin = portion.begin + (portion.include_begin ? 0 : inset);
            const double end = portion.end - (portion.include_end ? 0 : inset);
            // Inward alternatives survive integer rounding at closed boundaries.
            std::vector<double> candidates = {begin, end, (begin + end) / 2,
                std::min(end, begin + inset), std::max(begin, end - inset)};
            if (curve.radius > 0) {
                const double extremum = curve.parameter(std::atan2(double(direction.y), double(direction.x)));
                if (extremum >= begin && extremum <= end) candidates.push_back(extremum);
            }
            std::sort(candidates.begin(), candidates.end());
            for (double t : candidates) {
                const c_point point = curve.point(t);
                if (m_constraints.classify_point(loop, point, portion.paint_margin) != portion.status) continue;
                const long double primary = static_cast<long double>(point.x) * direction.x +
                                            static_cast<long double>(point.y) * direction.y;
                const long double secondary = static_cast<long double>(point.x) * direction.y -
                                              static_cast<long double>(point.y) * direction.x;
                if (!selected.valid || portion.status > best_status ||
                    (portion.status == best_status && (primary > selected.primary ||
                    (primary == selected.primary && secondary > selected.secondary)))) {
                    selected = {true, point, primary, secondary};
                    best_status = portion.status;
                }
            }
        }
    }
    if (!selected.valid) select_vertex(loop, direction, selected);
    if (!selected.valid)
        throw std::runtime_error("Fixed-direction seam placement found no stored loop vertex.");
    return selected.point;
}

FixedDirectionSeamPlacer &FixedDirectionSeamPlacer::instance(orchestrator_handle *orchestrator)
{
    /* Plugin metadata is process-wide; per-plan mutable state lives in sessions. */
    static FixedDirectionSeamPlacer plugin(orchestrator);
    return plugin;
}

const char *FixedDirectionSeamPlacer::id_impl() const noexcept { return k_plugin_id; }
const char *FixedDirectionSeamPlacer::name_impl() const noexcept { return "Fixed direction"; }

const char *FixedDirectionSeamPlacer::description_impl() const noexcept
{
    return "Places seams in a fixed direction, respecting painted seam constraints and inside seam volumes.";
}

const char *FixedDirectionSeamPlacer::exclusive_group_impl() const noexcept { return k_group_id; }
slicing_step_t FixedDirectionSeamPlacer::step_impl() const noexcept { return SEAM_PLACER; }
const char *const *FixedDirectionSeamPlacer::dependencies_impl() const noexcept { return k_no_dependencies; }
int32_t FixedDirectionSeamPlacer::priority_impl() const noexcept { return 600; }
int32_t FixedDirectionSeamPlacer::exclusive_group_priority_impl() const noexcept { return 30; }

int32_t FixedDirectionSeamPlacer::used_config_keys(raw_used_config_key *keys) const noexcept
{
    // Reporting the read dependency lets the orchestrator invalidate ordering
    // when a region changes its fixed direction.
    if (keys != nullptr)
        keys[0] = k_used_config_keys[0];
    return int32_t(std::size(k_used_config_keys));
}

int32_t FixedDirectionSeamPlacer::defined_config_keys(const char **keys) const noexcept
{
    /* Declare ownership of the option definition registered during initialize. */
    if (keys != nullptr)
        keys[0] = k_direction_key;
    return 1;
}

void FixedDirectionSeamPlacer::inilialize_impl(storage_handle *) const
{
    // Register the enum definition before exposing its setting in the GUI.
    // Register a regional enum so each LayerRegion can resolve its own value.
    // STEP_ORDERING is the first plan stage that consumes seam placement here.
    raw_config_option_def def = raw_config_option_def_init();
    def.opt_key = k_direction_key;
    def.type = RAW_CO_ENUM;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.gui_type = RAW_GUI_TYPE_SELECT_CLOSE;
    def.printer_technology = RAW_PT_FFF;
    def.label = "Fixed seam direction";
    def.full_label = "Fixed seam direction";
    def.category = RAW_OPTION_CATEGORY_PERIMETER;
    def.invalidates_step = STEP_ORDERING;
    def.tooltip = "Choose the platter direction for seam placement within painted or volume constraints.";
    def.mode = RAW_CONFIG_OPTION_MODE_SIM_ADV_EXP | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "rear";
    def.enum_def.value_label_pairs.items = k_direction_values;
    def.enum_def.value_label_pairs.count = uint32_t(std::size(k_direction_values));
    if (orchestrator_create_option_def(m_orchestrator, &def) != OPTION_DEF_ERROR_OK)
        throw std::runtime_error("Cannot register seam_fixed_direction.");

    // Insert only the plugin-owned setting into the existing Seam group. The
    // legacy seam_position setting remains a separate G-code-era control.
    orchestrator_add_ui_fragment(
        m_orchestrator,
        "print.ui",
        k_plugin_id,
        "page:Perimeters & Shell\n"
        "group:Seam\n"
        "setting:seam_fixed_direction\n",
        1);
}

void FixedDirectionSeamPlacer::run_impl(const plugin_run_context *run_context) const
{
    // A SEAM_PLACER run creates and publishes one initialized session; actual
    // placement is performed later through its service callbacks.
    // SEAM_PLACER run() is a factory invocation, not a placement request. It
    // initializes one session and transfers its ownership into the payload;
    // ordering consumers will take and eventually destroy that instance.
    run_ctx_seam_placer *context = plugin_ctx_as_seam_placer(run_context);
    if (context == nullptr || context->plan == nullptr ||
        (context->region_count != 0 && context->regions == nullptr))
        throw std::invalid_argument("Fixed-direction seam placement received an invalid run context.");
    std::vector<PrintRegion> print_regions;
    print_regions.reserve(context->region_count);
    for (uint32_t region_idx = 0; region_idx < context->region_count; ++region_idx)
        print_regions.emplace_back(context->regions[region_idx]);
    InitializationCancellation::check(run_context);
    context->instance = make_seam_placer_instance(
        std::unique_ptr<SeamPlacerSession>(new FixedDirectionSession(
            run_context->plugin_storage, run_context)),
        PrintingPlan(context->plan), print_regions);
}

} // namespace

void register_fixed_direction_seam_placer_plugin(orchestrator_handle *orchestrator)
{
    /* Registration exposes the singleton; session creation remains per run(). */
    orchestrator_register_plugin(
        orchestrator, FixedDirectionSeamPlacer::instance(orchestrator).c_instance());
}

}}} // namespace slic3r_api::SeamPlacement::FixedDirectionPlugin
