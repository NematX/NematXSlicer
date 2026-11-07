#include <catch2/catch.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepGenerateInfill.hpp"
#include "libslic3r/Steps/StepGeneratePerimeter.hpp"
#include "libslic3r/Steps/StepSurfaceGeneration.hpp"
#include "libslic3r/Surface.hpp"

/*
Simple infill pattern plugins
=============================

Honeycomb, 3D honeycomb, Hilbert curve, Archimedean chords and octagram spiral
are standalone INFILL_PATTERN plugins. These tests exercise STEP_INFILL end to
end: they build a sliced cube, run perimeter and surface generation, then run
the infill step with only the plugins it needs.

The registration case also covers line, concentric and the rectilinear family,
including grid, triangles, stars, cubic and the hidden supportbase provider.
Generation cases inspect published infill for grid, line and concentric. A
recipe modifier redirects sparse non-bridge recipes to supportbase and checks
both pattern resolution and publication in the SUPPORT bucket.

Hole cases replace the sliced layers by ring cross-sections and sample the
published leaf-path segments against their fill surfaces; joining paths across
the empty hole must be detected. Honeycomb cases compare sorted interior points
between unchanged and shifted layer cross-sections, so boundary clipping and
path order do not hide a lattice shift. ScopedActivePlugins restores the prior
plugin selection after each restricted generation run.

The test cases have distinct preparation and inspection phases:

    Registration case
    `-- initialize the runtime; look up each listed provider and check INFILL_PATTERN

    Generation and supportbase cases
    |-- prepare_cube_print() with the requested sparse pattern
    |-- activate the required perimeter, surface, infill and pattern plugins
    |-- run_until_infill()
    |   |-- clear and run StepGeneratePerimeter
    |   |-- clear and run StepSurfaceGeneration
    |   `-- clear and run StepGenerateInfill
    |       `-- SupportBaseSelector modifies sparse recipes in the supportbase case
    `-- after return, count_infill_paths() or count_support_paths()
        `-- count_paths_in_bucket() measures shallow output in INFILLS or SUPPORT

    Hole cases
    |-- prepare_cube_print(), then replace_all_layer_islands() with a square-hole ring
    |-- select the tested pattern and run_until_infill()
    |-- require more than one counted infill item
    |-- count_infill_paths_crossing_holes()
    |   |-- walk extrusion children recursively to their printable leaves
    |   |-- sample each segment at its ends and quarter intervals
    |   `-- count paths with a sample outside all region-island fill surfaces
    `-- require that crossing count to be zero

    Honeycomb alignment cases
    |-- prepare reference and shifted-second-layer cross-sections separately
    |-- run_until_infill() for each prepared print
    |-- interior_infill_points() after each run
    |   `-- collect and sort interior leaf-path points from layer one
    `-- require enough reference points and equal point sets
*/

using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

namespace {

// Count one path per ungrouped entity and the direct children of each logical group in the selected bucket.
// This shallow count does not recurse through nested groups or count all leaf paths.
// INFILLS and SUPPORT checks use it to require output; hole cases require more than
// one counted item, then inspect leaf-path geometry separately for crossings.
size_t count_paths_in_bucket(const Print &print, ExtrusionRole role);

constexpr const char *SUPPORT_BASE_SELECTOR = "tests.infill.supportbase_selector";
const char *k_no_dependencies[] = { nullptr };

class SupportBaseSelector final : public slic3r_api::PluginBase
{
public:
    explicit SupportBaseSelector(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

    void reset()
    {
        m_run_count.store(0);
        m_supportbase_resolution_count.store(0);
        m_resolution_mismatch_count.store(0);
    }
    uint32_t run_count() const { return m_run_count.load(); }
    uint32_t supportbase_resolution_count() const { return m_supportbase_resolution_count.load(); }
    uint32_t resolution_mismatch_count() const { return m_resolution_mismatch_count.load(); }

private:
    const char *id_impl() const noexcept override { return SUPPORT_BASE_SELECTOR; }
    const char *name_impl() const noexcept override { return "Support base selector"; }
    const char *description_impl() const noexcept override
    {
        return "Redirects sparse recipes to the hidden support base pattern.";
    }
    slicing_step_t step_impl() const noexcept override { return INFILL_SURFACE_RECIPE_MODIFIER; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    int32_t priority_impl() const noexcept override { return 0; }

    void run_impl(const plugin_run_context *run_ctx) const override
    {
        const run_ctx_infill_surface_recipe_modifier *ctx =
            plugin_ctx_as_infill_surface_recipe_modifier(run_ctx);
        if (ctx == nullptr || ctx->params == nullptr || ctx->generate_infill_ctx == nullptr ||
            ctx->resolve_pattern_id == nullptr)
            return;

        // Support-base connection is defined only for sparse, non-bridge recipes.
        if (ctx->params->flow.is_bridge != 0 || ctx->params->density >= 0.9999f)
            return;

        ctx->params->anchor_length = 2.f;
        ctx->params->anchor_length_max = 5.f;
        const infill_pattern_runtime_id resolved_pattern_id =
            ctx->resolve_pattern_id(ctx->generate_infill_ctx, "supportbase");
        const char *resolved_plugin_id = ctx->pattern_plugin_id != nullptr ?
            ctx->pattern_plugin_id(ctx->generate_infill_ctx, resolved_pattern_id) : nullptr;
        if (resolved_plugin_id != nullptr && std::string(resolved_plugin_id) == "supportbase")
            ++m_supportbase_resolution_count;
        else
            ++m_resolution_mismatch_count;

        ctx->params->pattern_id = resolved_pattern_id;
        ctx->params->extrusion_role = RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL;
        ++m_run_count;
    }

    mutable std::atomic_uint32_t m_run_count{ 0 };
    mutable std::atomic_uint32_t m_supportbase_resolution_count{ 0 };
    mutable std::atomic_uint32_t m_resolution_mismatch_count{ 0 };
};

SupportBaseSelector &support_base_selector()
{
    Orchestrator &orchestrator = Orchestrator::instance();
    static SupportBaseSelector selector(reinterpret_cast<orchestrator_handle *>(&orchestrator));
    if (orchestrator.get_plugin(SUPPORT_BASE_SELECTOR) == nullptr)
        REQUIRE(orchestrator.register_plugin(selector.c_instance()));
    return selector;
}

// Local copy of the perimeter test scope helper so the infill test can restrict
// the active plugin set without touching the shared helper.
class ScopedActivePlugins
{
public:
    explicit ScopedActivePlugins(std::initializer_list<const char *> plugin_ids) :
        m_orchestrator(Orchestrator::instance())
    {
        m_previous_active_plugins.reserve(m_orchestrator.active_plugins().size());
        for (Plugin *plugin : m_orchestrator.active_plugins())
            m_previous_active_plugins.push_back(plugin);

        m_orchestrator.clear_active_plugins();
        for (const char *plugin_id : plugin_ids)
            REQUIRE(m_orchestrator.set_plugin_active(plugin_id, true));
    }

    ~ScopedActivePlugins()
    {
        m_orchestrator.clear_active_plugins();
        for (Plugin *plugin : m_previous_active_plugins)
            m_orchestrator.set_plugin_active(plugin, true);
    }

private:
    Orchestrator &m_orchestrator;
    std::vector<Plugin *> m_previous_active_plugins;
};

DynamicPrintConfig pattern_test_config(const char *pattern_id)
{
    return perimeter_config({
        { "fill_pattern", pattern_id },
        { "fill_density", "20%" },
        { "top_solid_layers", "0" },
        { "bottom_solid_layers", "0" },
        { "solid_infill_every_layers", "0" },
        { "infill_every_layers", "1" }
    });
}

void run_until_infill(PreparedPerimeterPrint &prepared)
{
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepGeneratePerimeter::clear(prepared.print);
    Steps::StepGeneratePerimeter::run_step(orchestrator, prepared.print);
    Steps::StepSurfaceGeneration::clear(prepared.print);
    Steps::StepSurfaceGeneration::run_step(orchestrator, prepared.print);
    Steps::StepGenerateInfill::clear(prepared.print);
    Steps::StepGenerateInfill::run_step(orchestrator, prepared.print);
}

size_t count_paths_in_bucket(const Print &print, const ExtrusionRole role)
{
    size_t paths = 0;
    for (const PrintObject &object : print.objects())
        for (const Layer &layer : object.layers())
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region_island : island.regions_islands()) {
                    if (!region_island.has_extrusion(role))
                        continue;
                    for (const ExtrusionEntity *entity : region_island.extrusion(role).entities())
                        paths += entity->is_collection() ? entity->child_count() : 1;
                }
    return paths;
}

size_t count_infill_paths(const Print &print)
{
    return count_paths_in_bucket(print, LayerRegionIsland::INFILLS);
}

size_t count_support_paths(const Print &print)
{
    return count_paths_in_bucket(print, LayerRegionIsland::SUPPORT);
}

// Count infill paths that leave every fill surface of their region island. A
// path merged across a hole crosses the hole interior, so one of its sampled
// points is no longer contained by any fill surface expolygon.
size_t count_infill_paths_crossing_holes(const Print &print)
{
    size_t crossing = 0;
    for (const PrintObject &object : print.objects())
        for (const Layer &layer : object.layers())
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region_island : island.regions_islands()) {
                    if (!region_island.has_extrusion(LayerRegionIsland::INFILLS))
                        continue;
                    const SurfaceCollection &surfaces = region_island.fill_surfaces();
                    if (surfaces.empty())
                        continue;

                    const auto point_is_filled = [&surfaces](const Point &point) {
                        for (const Surface &surface : surfaces)
                            if (surface.expolygon.contains(point))
                                return true;
                        return false;
                    };
                    // Only leaves hold printable geometry. Walking children
                    // until is_leaf() keeps composite nodes out of collect_points()
                    // which would join separate child paths into fake segments.
                    const auto visit = [&crossing, &point_is_filled](
                                           const ExtrusionEntity &entity, const auto &self) -> void {
                        if (!entity.is_leaf()) {
                            for (const ExtrusionEntityUPtr &child : entity.children())
                                if (child != nullptr)
                                    self(*child, self);
                            return;
                        }

                        Points points;
                        entity.collect_points(points);
                        for (size_t idx = 0; idx + 1 < points.size(); ++idx) {
                            const Point &from = points[idx];
                            const Point &to = points[idx + 1];
                            for (const double ratio : { 0.0, 0.25, 0.5, 0.75, 1.0 }) {
                                const Point sample(
                                    coord_t(std::llround(double(from.x()) +
                                                         (double(to.x()) - double(from.x())) * ratio)),
                                    coord_t(std::llround(double(from.y()) +
                                                         (double(to.y()) - double(from.y())) * ratio)));
                                if (!point_is_filled(sample)) {
                                    ++crossing;
                                    return;
                                }
                            }
                        }
                    };

                    for (const ExtrusionEntity *entity :
                         region_island.extrusion(LayerRegionIsland::INFILLS).entities())
                        if (entity != nullptr)
                            visit(*entity, visit);
                }
    return crossing;
}

// Collect the infill points of one layer that sit well inside the printed area,
// so clipping at the layer boundary cannot hide the lattice alignment. The
// points are sorted to compare them as a set, independently of path ordering.
std::vector<std::pair<coord_t, coord_t>> interior_infill_points(const Print &print, size_t layer_idx)
{
    REQUIRE(print.objects().size() == 1);
    const Layer &layer = print.object(0).layer(layer_idx);
    const coord_t interior_limit = scale_i(8.5);

    std::vector<std::pair<coord_t, coord_t>> points;
    for (const LayerSliceIsland &island : layer.islands())
        for (const LayerRegionIsland &region_island : island.regions_islands()) {
            if (!region_island.has_extrusion(LayerRegionIsland::INFILLS))
                continue;

            const auto visit = [&points, interior_limit](const ExtrusionEntity &entity, const auto &self) -> void {
                if (!entity.is_leaf()) {
                    for (const ExtrusionEntityUPtr &child : entity.children())
                        if (child != nullptr)
                            self(*child, self);
                    return;
                }

                Points entity_points;
                entity.collect_points(entity_points);
                for (const Point &point : entity_points)
                    if (point.x() > -interior_limit && point.x() < interior_limit &&
                        point.y() > -interior_limit && point.y() < interior_limit)
                        points.push_back({ point.x(), point.y() });
            };

            for (const ExtrusionEntity *entity : region_island.extrusion(LayerRegionIsland::INFILLS).entities())
                if (entity != nullptr)
                    visit(*entity, visit);
        }

    std::sort(points.begin(), points.end());
    return points;
}

} // namespace

TEST_CASE("Simple infill pattern plugins register as infill pattern providers",
          "[plugins][infill][patterns]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    for (const char *plugin_id : { "line", "concentric", "honeycomb", "3dhoneycomb", "hilbertcurve", "archimedeanchords",
                                   "octagramspiral", "rectilinear", "alignedrectilinear", "monotonic",
                                   "grid", "triangles", "stars", "cubic",
                                   "scatteredrectilinear", "supportbase" }) {
        INFO(plugin_id);
        const Plugin *plugin = orchestrator.get_plugin(plugin_id);
        REQUIRE(plugin != nullptr);
        CHECK(plugin->get_step() == INFILL_PATTERN);
    }
}

TEST_CASE("Support base resolves through STEP_INFILL and generates extrusion",
          "[plugins][infill][patterns][supportbase]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    SupportBaseSelector &selector = support_base_selector();
    selector.reset();

    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, pattern_test_config("grid"));

    ScopedActivePlugins active({
        SIMPLE_PERIMETER_GENERATOR,
        INITIAL_TYPED_SURFACE_BUILDER,
        "infill.generator.default",
        "grid",
        "rectilinear",
        "supportbase",
        SUPPORT_BASE_SELECTOR
    });
    run_until_infill(prepared);

    CHECK(selector.run_count() > 0);
    CHECK(selector.supportbase_resolution_count() == selector.run_count());
    CHECK(selector.resolution_mismatch_count() == 0);
    CHECK(count_support_paths(prepared.print) > 0);
}

TEST_CASE("Simple infill patterns generate extrusion through STEP_INFILL",
          "[plugins][infill][patterns]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const char *pattern_id = GENERATE("grid", "line", "concentric");
    INFO(pattern_id);

    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, pattern_test_config(pattern_id));

    ScopedActivePlugins active({
        SIMPLE_PERIMETER_GENERATOR,
        INITIAL_TYPED_SURFACE_BUILDER,
        "infill.generator.default",
        pattern_id,
        "rectilinear"
    });
    run_until_infill(prepared);

    // The plugin produced extrusion; a failure would leave the bucket empty.
    CHECK(count_infill_paths(prepared.print) > 0);
}

TEST_CASE("Simple infill patterns never extrude across a surface hole",
          "[plugins][infill][patterns]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const char *pattern_id = GENERATE("gyroid", "hilbertcurve", "honeycomb", "rectilinear", "monotonic", "line", "concentric");
    INFO(pattern_id);

    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, pattern_test_config(pattern_id));

    // Give every layer the same ring cross-section. The clipping then splits the
    // generated paths at the square hole, and a merge across it would extrude
    // straight through the void.
    replace_all_layer_islands(prepared, rectangle_with_hole_expolygon());

    ScopedActivePlugins active({
        SIMPLE_PERIMETER_GENERATOR,
        INITIAL_TYPED_SURFACE_BUILDER,
        "infill.generator.default",
        pattern_id,
        "rectilinear"
    });
    run_until_infill(prepared);

    CHECK(count_infill_paths(prepared.print) > 1);
    CHECK(count_infill_paths_crossing_holes(prepared.print) == 0);
}

TEST_CASE("Honeycomb patterns keep their lattice aligned across layers",
          "[plugins][infill][patterns]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const char *pattern_id = GENERATE("honeycomb", "3dhoneycomb");
    INFO(pattern_id);

    // The legacy honeycomb patterns anchor their lattice to the world frame, not
    // to the cross-section of the layer. Moving the cross-section of one layer
    // must therefore not move the infill inside the part.
    const ExPolygon full_area = rectangle_expolygon(-10., -10., 10., 10.);
    const ExPolygon shifted_area = rectangle_expolygon(-9.7, -10., 10.3, 10.);

    const auto layer_one_infill = [pattern_id, &full_area](const ExPolygon &second_layer_area) {
        PreparedPerimeterPrint prepared;
        prepare_cube_print(prepared, pattern_test_config(pattern_id));

        const size_t layer_count = prepared.print.object(0).layer_count();
        REQUIRE(layer_count > 1);
        std::vector<ExPolygon> areas(layer_count, full_area);
        areas[1] = second_layer_area;
        replace_all_layer_islands(prepared, areas);

        ScopedActivePlugins active({
            SIMPLE_PERIMETER_GENERATOR,
            INITIAL_TYPED_SURFACE_BUILDER,
            "infill.generator.default",
            pattern_id,
            "rectilinear"
        });
        run_until_infill(prepared);
        return interior_infill_points(prepared.print, 1);
    };

    const std::vector<std::pair<coord_t, coord_t>> reference = layer_one_infill(full_area);
    const std::vector<std::pair<coord_t, coord_t>> shifted = layer_one_infill(shifted_area);

    REQUIRE(reference.size() > 10);
    CHECK(shifted == reference);
}
