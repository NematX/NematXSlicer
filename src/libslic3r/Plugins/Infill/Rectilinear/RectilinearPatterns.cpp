///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "RectilinearPatterns.hpp"

#include <array>
#include <memory>
#include <vector>

#include "RectilinearFills.hpp"
#include "RectilinearSawtooth.hpp"
#include "RectilinearWithPerimeter.hpp"

#include "libslic3r/Api/plugin/c/slic3r_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Plugins/Infill/PatternPluginHelpers.hpp"

/*
Rectilinear pattern plugins
===========================

The ordinary rectilinear patterns use the ported FillRectilinear family
(RectilinearFills.hpp). Support Base uses the same rasterizer with its
dedicated support connector, and Rectilinear-with-perimeter uses its own
helper. Sawtooth uses the same XY filler with a publisher for its Z-profiled
extrusion tree. The plugin boundary passes the surface API view, the raw
infill recipe, and publishes the generated paths.

Monotonic patterns keep their path order and direction: their group is
published without the sortable / reversible flags, so the extrusion step only
inserts travels between the paths. Every other pattern exposes its paths to the
ordering engine.
*/

namespace slic3r_api { namespace Infill {
namespace {

using namespace PatternHelpers;

namespace rect_work = PluginCommon::polygon_work;
namespace rect_fills = RectilinearPatternsPlugin::fills;
namespace recti = RectilinearPatternsPlugin::recti;
namespace sawtooth = RectilinearPatternsPlugin::sawtooth;

const char *k_no_dependencies[] = { nullptr };
const char *const k_no_lists[] = { nullptr };

// Option keys whose pattern list this plugin wants to join. Each pattern
// declares the lists it is valid for; the host ignores unknown keys, so a
// pattern never needs to know whether a list already exists.
const char *const k_all_five[] = {
    "fill_pattern", "top_fill_pattern", "bottom_fill_pattern", "solid_fill_pattern", "bridge_fill_pattern", nullptr
};
const char *const k_sparse_top_bottom_solid[] = {
    "fill_pattern", "top_fill_pattern", "bottom_fill_pattern", "solid_fill_pattern", nullptr
};
const char *const k_top_bottom_solid[] = {
    "top_fill_pattern", "bottom_fill_pattern", "solid_fill_pattern", nullptr
};
const char *const k_top_only[] = { "top_fill_pattern", nullptr };
const char *const k_sparse_only[] = { "fill_pattern", nullptr };

enum class PatternKind
{
    Rectilinear,
    AlignedRectilinear,
    Monotonic,
    Grid,
    Triangles,
    Stars,
    Cubic,
    ScatteredRectilinear,
    Sawtooth,
    SupportBase,
    RectilinearWithPerimeter
};

struct PatternDefinition
{
    const char *id;
    const char *name;
    const char *description;
    int32_t priority;
    PatternKind kind;
    const char *const *list_keys;
};

// Declaration order defines the familiar config UI order. The explicit
// priority keeps the order stable with the patterns that are still legacy.
const std::array<PatternDefinition, 11> k_patterns = {{
    { "rectilinear", "Rectilinear", "Classic rectilinear infill pattern.", 0, PatternKind::Rectilinear, k_all_five },
    { "monotonic", "Monotonic", "Monotonic solid infill pattern.", 1, PatternKind::Monotonic, k_all_five },
    { "alignedrectilinear", "Aligned Rectilinear", "Rectilinear infill aligned across layers.", 2,
      PatternKind::AlignedRectilinear, k_sparse_top_bottom_solid },
    { "grid", "Grid", "Grid infill pattern.", 3, PatternKind::Grid, k_sparse_only },
    { "triangles", "Triangles", "Triangular infill pattern.", 4, PatternKind::Triangles, k_sparse_only },
    { "stars", "Stars", "Star-shaped triangular infill pattern.", 5, PatternKind::Stars, k_sparse_only },
    { "cubic", "Cubic", "Cubic infill pattern.", 6, PatternKind::Cubic, k_sparse_only },
    { "rectiwithperimeter", "Rectilinear with Perimeter", "Rectilinear infill with perimeter-aware filling.", 20,
      PatternKind::RectilinearWithPerimeter, k_no_lists },
    { "scatteredrectilinear", "Scattered Rectilinear", "Scattered rectilinear infill pattern.", 21,
      PatternKind::ScatteredRectilinear, k_sparse_only },
    { "sawtooth", "Sawtooth", "Sawtooth rectilinear infill pattern.", 22,
      PatternKind::Sawtooth, k_top_only },
    { "supportbase", "Support Base", "Sparse support base pattern.", 25,
      PatternKind::SupportBase, k_no_lists },
}};

std::unique_ptr<rect_fills::FillRectilinear> make_pattern(PatternKind kind)
{
    switch (kind) {
    case PatternKind::Rectilinear:              return std::make_unique<rect_fills::FillRectilinear>();
    case PatternKind::AlignedRectilinear:       return std::make_unique<rect_fills::FillAlignedRectilinear>();
    case PatternKind::Monotonic:                return std::make_unique<rect_fills::FillMonotonic>();
    case PatternKind::Grid:                     return std::make_unique<rect_fills::FillGrid>();
    case PatternKind::Triangles:                return std::make_unique<rect_fills::FillTriangles>();
    case PatternKind::Stars:                    return std::make_unique<rect_fills::FillStars>();
    case PatternKind::Cubic:                    return std::make_unique<rect_fills::FillCubic>();
    case PatternKind::ScatteredRectilinear:     return std::make_unique<rect_fills::FillScatteredRectilinear>();
    case PatternKind::Sawtooth:                 return std::make_unique<rect_fills::FillRectilinear>();
    case PatternKind::SupportBase:              return std::make_unique<rect_fills::FillSupportBase>();
    case PatternKind::RectilinearWithPerimeter: return std::make_unique<rect_fills::FillRectilinear>();
    }
    return nullptr;
}

void publish_pattern_paths(const run_ctx_infill_pattern &ctx, storage_handle *storage,
                           const rect_work::Polylines &lines, const raw_infill_pattern_params &params,
                           bool keep_path_order)
{
    StoredExtrusionEntity group(storage);
    if (!keep_path_order)
        group.set_flags(group.flags() | RAW_EXTRUSION_FLAG_SORTABLE | RAW_EXTRUSION_FLAG_REVERSIBLE);

    for (const rect_work::Polyline &line : lines) {
        if (line.points.size() < 2)
            continue;
        StoredExtrusionEntity path = make_path(storage, line.points, params);
        group.append_child_move(path.mutable_view());
    }

    // Adding the first child makes an empty API entity sortable by default.
    if (keep_path_order) {
        group.disable_sort();
        group.disable_reverse();
    }

    if (!group.empty()) {
        MutableExtrusionEntity output(ctx.output);
        output.append_child_move(group.mutable_view());
    }
}

class RectilinearPattern : public PluginBase
{
public:
    RectilinearPattern(orchestrator_handle *orch, const PatternDefinition &definition)
        : PluginBase(orch), m_definition(definition) {}

private:
    const char *id_impl() const noexcept override { return m_definition.id; }
    const char *name_impl() const noexcept override { return m_definition.name; }
    const char *description_impl() const noexcept override { return m_definition.description; }
    slicing_step_t step_impl() const noexcept override { return INFILL_PATTERN; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    int32_t priority_impl() const noexcept override { return m_definition.priority; }
    const char *progress_message_format_impl() const noexcept override { return "Generating infill pattern: %u / %u"; }

    void setup_run_impl(const plugin_run_context *run_ctx) const override
    {
        throw_if_cancelled(run_ctx);
        const run_ctx_infill_pattern *ctx = plugin_ctx_as_infill_pattern(run_ctx);
        if (run_ctx != nullptr && run_ctx->plugin_storage != nullptr && ctx != nullptr && ctx->surface != nullptr &&
            ctx->params != nullptr && ctx->output != nullptr)
            progress().add_max(1);
    }

    void inilialize_impl(storage_handle *) const override
    {
        for (const char *const *key = m_definition.list_keys; *key != nullptr; ++key) {
            key_value_string_pair_t entry = { m_definition.id, m_definition.name };
            key_value_string_pair_array_t entries = { &entry, 1 };

            raw_option_def_patch patch = raw_option_def_patch_init();
            patch.kind = RAW_OPTION_DEF_PATCH_ADD_ENUM_VALUES;
            patch.opt_key = *key;
            patch.enum_values = entries;
            orchestrator_modify_option_def(m_orchestrator, &patch);
        }
    }

    void run_impl(const plugin_run_context *run_ctx) const override
    {
        throw_if_cancelled(run_ctx);
        const run_ctx_infill_pattern *ctx = plugin_ctx_as_infill_pattern(run_ctx);
        if (ctx == nullptr || ctx->surface == nullptr || ctx->params == nullptr || ctx->output == nullptr ||
            run_ctx == nullptr || run_ctx->plugin_storage == nullptr)
            return;

        const raw_infill_pattern_params &raw = *ctx->params;

        // The ported algorithms use this plugin storage for transient Clipper operations.
        const slic3r_api::ExPolygon expolygon = Surface(ctx->surface).expolygon();

        std::unique_ptr<rect_fills::FillRectilinear> pattern = make_pattern(m_definition.kind);
        if (pattern == nullptr)
            return;
        rect_fills::configure_filler(*pattern, raw, run_ctx->plugin_storage);


        throw_if_cancelled(run_ctx);
        if (m_definition.kind == PatternKind::RectilinearWithPerimeter) {
            StoredExtrusionEntity root = recti::RectiWithPerimeter::fill_with_perimeter(
                run_ctx->plugin_storage, expolygon, *pattern, raw);
            if (!root.empty()) {
                MutableExtrusionEntity output(ctx->output);
                output.append_child_move(root.mutable_view());
            }
            throw_if_cancelled(run_ctx);
            progress().increment();
            return;
        }

        const rect_work::Polylines lines = pattern->fill_surface(expolygon, raw);
        throw_if_cancelled(run_ctx);

        if (m_definition.kind == PatternKind::Sawtooth) {
            sawtooth::publish_sawtooth(*ctx, run_ctx->plugin_storage, lines, raw, pattern->angle);
            progress().increment();
            return;
        }

        publish_pattern_paths(*ctx, run_ctx->plugin_storage, lines, raw, pattern->no_sort());
        progress().increment();
    }

    PatternDefinition m_definition;
};

std::vector<std::unique_ptr<RectilinearPattern>> &pattern_instances(orchestrator_handle *orch)
{
    static std::vector<std::unique_ptr<RectilinearPattern>> plugins;
    if (plugins.empty()) {
        plugins.reserve(k_patterns.size());
        for (const PatternDefinition &definition : k_patterns)
            plugins.emplace_back(new RectilinearPattern(orch, definition));
    }
    return plugins;
}

} // namespace

void register_rectilinear_pattern_plugins(orchestrator_handle *orch)
{
    for (const std::unique_ptr<RectilinearPattern> &plugin : pattern_instances(orch))
        orchestrator_register_plugin(orch, plugin->c_instance());
}

}} // namespace slic3r_api::Infill
