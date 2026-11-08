#include <catch2/catch.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <utility>
#include <vector>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/c/slic3r_data_tree.h"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionProperty.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepGenerateInfill.hpp"
#include "libslic3r/Steps/StepGeneratePerimeter.hpp"
#include "libslic3r/Steps/StepPostInfillGeneration.hpp"
#include "libslic3r/Steps/StepSurfaceGeneration.hpp"
#include "libslic3r/Surface.hpp"

/*
Post-infill full fill volume correction
=======================================

STEP_INFILL stamps the raw fill recipe and the free area to fill on the root of
a solid surface subtree when enforce_full_fill_volume is enabled. These tests
exercise the stamp and the STEP_POST_INFILL pass that turns it into a flow
multiplier, the replacement of the legacy Fill::fill_surface_extrusion()
correction.
*/

using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

namespace {

// Local copy of the perimeter test scope helper so this test can restrict the
// active plugin set without touching the shared helper.
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
        for (const char *plugin_id : plugin_ids) {
            if (!m_orchestrator.set_plugin_active(plugin_id, true)) {
                // Restore the previous set before reporting: a failed
                // activation happens while this object is still under
                // construction, so its destructor would not run and the
                // partially cleared plugin set would leak into later tests.
                restore_previous_plugins();
                FAIL("Cannot activate test plugin " << plugin_id);
            }
        }
    }

    ~ScopedActivePlugins() { restore_previous_plugins(); }

private:
    void restore_previous_plugins()
    {
        m_orchestrator.clear_active_plugins();
        for (Plugin *plugin : m_previous_active_plugins)
            m_orchestrator.set_plugin_active(plugin, true);
        m_previous_active_plugins.clear();
    }

    Orchestrator &m_orchestrator;
    std::vector<Plugin *> m_previous_active_plugins;
};

const char *const k_full_fill_volume_corrector = "infill.post_process.full_fill_volume";
const char *const k_post_infill_gap_fill = "infill.post_process.gap_fill";

using ConfigOverride = std::pair<std::string, std::string>;

DynamicPrintConfig full_fill_volume_config(
    const char *pattern_id,
    const char *fill_density,
    bool enforce_full_fill_volume,
    std::initializer_list<ConfigOverride> extra_overrides = {})
{
    // top_solid_layers and bottom_solid_layers are inert in this pipeline: the
    // top/bottom to sparse degradation lives in
    // LayerRegion::prepare_fill_surfaces(), which the plugin surface-generation
    // path does not call. The initial typed surface builder still marks the
    // first layer as bottom-solid and the last layer as top-solid, which is
    // what makes solid surfaces available here.
    DynamicPrintConfig config = perimeter_config({
        { "fill_pattern", pattern_id },
        { "fill_density", fill_density },
        { "top_solid_layers", "0" },
        { "bottom_solid_layers", "0" },
        { "solid_infill_every_layers", "0" },
        { "infill_every_layers", "1" },
        { "enforce_full_fill_volume", enforce_full_fill_volume ? "1" : "0" }
    });
    for (const ConfigOverride &entry : extra_overrides)
        config.set_deserialize_strict(entry.first, entry.second);
    return config;
}


struct StampedRootVolume
{
    uint64_t surface_id = 0;
    double target_mm3 = 0.;
    double volume_mm3 = 0.;
    // Legacy normalization the corrector applies to the measured volume:
    // spacing / width_eff, or 1 when the recipe keeps the full line overlap.
    double normalization = 1.;
    bool solid_surface = false;
};

struct FullFillVolumeRun
{
    size_t root_count = 0;
    std::vector<StampedRootVolume> stamped_roots;
};

// Volume the corrector aims for while its ratio stays inside the failsafe
// bounds. The plugin compares the raw target with the normalized measurement,
// so the reached volume is target / normalization (identical to the raw target
// at 100% overlap).
double corrected_target_mm3(const StampedRootVolume &root)
{
    return root.target_mm3 / root.normalization;
}

// The two runs of one test slice the same surfaces, so pair the roots by their
// source surface id instead of relying on a stable traversal order.
const StampedRootVolume *find_stamped_root(const FullFillVolumeRun &run, uint64_t surface_id)
{
    for (const StampedRootVolume &root : run.stamped_roots)
        if (root.surface_id == surface_id)
            return &root;
    return nullptr;
}

double stamped_target_mm3(const FullFillVolumeRun &run)
{
    double total = 0.;
    for (const StampedRootVolume &root : run.stamped_roots)
        total += root.target_mm3;
    return total;
}

double stamped_volume_mm3(const FullFillVolumeRun &run)
{
    double total = 0.;
    for (const StampedRootVolume &root : run.stamped_roots)
        total += root.volume_mm3;
    return total;
}

// Same failsafe bounds as the legacy Fill correction and the plugin.
double clamp_flow_mult(double mult)
{
    return std::min(1.3, std::max(0.8, mult));
}

FullFillVolumeRun collect_full_fill_volume_run(const Print &print)
{
    FullFillVolumeRun run;
    for (const PrintObject &object : print.objects())
        for (const Layer &layer : object.layers())
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region_island : island.regions_islands()) {
                    if (!region_island.has_extrusion(LayerRegionIsland::INFILLS))
                        continue;
                    for (const ExtrusionEntity *entity :
                         region_island.extrusion(LayerRegionIsland::INFILLS).entities()) {
                        if (entity == nullptr)
                            continue;
                        ++run.root_count;

                        const ExtrusionPropertyInfill *property =
                            entity->get_property<ExtrusionPropertyInfill>();
                        if (property == nullptr || property->area_to_fill_mm2 <= 0.)
                            continue;
                        const double target = property->area_to_fill_mm2 * unscaled(property->height) *
                                              double(property->flow_mult);
                        // The plugin normalizes the measured volume the same way
                        // the legacy correction did (FillBase.cpp:314-318).
                        double normalization = 1.;
                        const double height_mm = unscaled(property->height);
                        const double spacing_mm = unscaled(property->spacing);
                        if (property->spacing_ratio != 1.f && spacing_mm > 0.) {
                            const double width_eff_mm =
                                spacing_mm - height_mm * (1. - 0.25 * PI) *
                                                 (1. - double(property->spacing_ratio));
                            if (width_eff_mm > 0.)
                                normalization = spacing_mm / width_eff_mm;
                        }

                        StampedRootVolume root;
                        root.surface_id = property->source_surface_id;
                        root.target_mm3 = target;
                        root.volume_mm3 = entity->total_volume();
                        root.normalization = normalization;
                        root.solid_surface =
                            (property->surface_type & RAW_SURFACE_TYPE_DENS_SOLID) != 0;
                        run.stamped_roots.push_back(root);
                    }
                }
    return run;
}

// Prepare the slab of every case: one bottom solid layer, one sparse layer and
// one top solid layer, the smallest print that has both stamped solid roots and
// an unstamped sparse root. A 14 x 14 mm section with a single perimeter leaves
// a well defined free area, so the stamped target and the measured extrusion are
// both stable. Every extra layer or wider section would only add pattern work:
// in Debug, a full-density Hilbert curve already takes about two seconds per
// surface. Tests that need a coverage extreme pass their own cross-section.
void prepare_slab(PreparedPerimeterPrint &prepared, const DynamicPrintConfig &config, const ExPolygon *area)
{
    // The first layer is 1 mm high and the next ones 0.3 mm, the largest height for the 0.4 mm nozzle.
    prepare_box_print(prepared, config, 1.6);
    REQUIRE(prepared.print.object(0).layer_count() == 3);
    replace_all_layer_islands(prepared, area != nullptr ? *area : rectangle_expolygon(-7., -7., 7., 7.));
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

void run_until_post_infill(PreparedPerimeterPrint &prepared)
{
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepPostInfillGeneration::clear(prepared.print);
    Steps::StepPostInfillGeneration::run_step(orchestrator, prepared.print);
}

// Prepare one print and run the infill step only.
FullFillVolumeRun run_full_fill_volume_case(
    const char *pattern_id,
    const char *fill_density,
    bool enforce_full_fill_volume,
    std::initializer_list<ConfigOverride> extra_overrides = {},
    const ExPolygon *area = nullptr)
{
    PreparedPerimeterPrint prepared;
    prepare_slab(prepared, full_fill_volume_config(pattern_id, fill_density, enforce_full_fill_volume, extra_overrides), area);

    ScopedActivePlugins active({
        SIMPLE_PERIMETER_GENERATOR,
        INITIAL_TYPED_SURFACE_BUILDER,
        "infill.generator.default",
        pattern_id,
        k_post_infill_gap_fill
    });
    run_until_infill(prepared);

    return collect_full_fill_volume_run(prepared.print);
}

// Measure the same print before and after one post-infill pass. The two
// measurements must describe the same surfaces: surface ids are only valid
// inside one print, so the runs cannot be compared across two independent
// slices.
struct FullFillVolumeComparison
{
    FullFillVolumeRun before;
    FullFillVolumeRun after;
};

FullFillVolumeComparison run_full_fill_volume_comparison(
    const char *pattern_id,
    const char *fill_density,
    bool enforce_full_fill_volume,
    const char *post_plugin,
    std::initializer_list<ConfigOverride> extra_overrides = {},
    const ExPolygon *area = nullptr)
{
    PreparedPerimeterPrint prepared;
    prepare_slab(prepared, full_fill_volume_config(pattern_id, fill_density, enforce_full_fill_volume, extra_overrides), area);

    ScopedActivePlugins active({
        SIMPLE_PERIMETER_GENERATOR,
        INITIAL_TYPED_SURFACE_BUILDER,
        "infill.generator.default",
        pattern_id,
        post_plugin
    });
    run_until_infill(prepared);

    FullFillVolumeComparison comparison;
    comparison.before = collect_full_fill_volume_run(prepared.print);
    run_until_post_infill(prepared);
    comparison.after = collect_full_fill_volume_run(prepared.print);
    return comparison;
}

// Run one infill pass followed by two post-infill passes on the same tree and
// return the measurements after each post pass.
std::pair<FullFillVolumeRun, FullFillVolumeRun> run_full_fill_volume_twice(const char *pattern_id)
{
    PreparedPerimeterPrint prepared;
    prepare_slab(prepared, full_fill_volume_config(pattern_id, "100%", true), nullptr);

    ScopedActivePlugins active({
        SIMPLE_PERIMETER_GENERATOR,
        INITIAL_TYPED_SURFACE_BUILDER,
        "infill.generator.default",
        pattern_id,
        k_full_fill_volume_corrector
    });
    run_until_infill(prepared);
    run_until_post_infill(prepared);
    const FullFillVolumeRun first = collect_full_fill_volume_run(prepared.print);
    run_until_post_infill(prepared);
    const FullFillVolumeRun second = collect_full_fill_volume_run(prepared.print);
    return { first, second };
}

} // namespace

TEST_CASE("The full fill volume corrector registers as a post-infill plugin",
          "[plugins][infill][volume]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const Plugin *plugin = Orchestrator::instance().get_plugin(k_full_fill_volume_corrector);
    REQUIRE(plugin != nullptr);
    CHECK(plugin->get_step() == STEP_POST_INFILL);
}

TEST_CASE("Full fill volume targets follow the enforce_full_fill_volume setting",
          "[plugins][infill][volume]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const char *pattern_id = GENERATE("gyroid", "hilbertcurve");
    INFO(pattern_id);

    const FullFillVolumeRun enforced = run_full_fill_volume_case(pattern_id, "100%", true);
    REQUIRE(enforced.root_count > 0);
    CHECK(enforced.stamped_roots.size() > 0);
    CHECK(stamped_target_mm3(enforced) > 0.);

    // Sparse surfaces never request the correction, even at 100% density: only
    // the solid first-layer bottom and last-layer top surfaces are stamped.
    CHECK(enforced.stamped_roots.size() < enforced.root_count);
    CHECK(enforced.stamped_roots.size() == 2);
    for (const StampedRootVolume &root : enforced.stamped_roots)
        CHECK(root.solid_surface);

    // Same rule at low density: the few solid surfaces are stamped, the sparse
    // ones are not.
    const FullFillVolumeRun sparse = run_full_fill_volume_case(pattern_id, "20%", true);
    REQUIRE(sparse.root_count > 0);
    CHECK(sparse.stamped_roots.size() > 0);
    CHECK(sparse.stamped_roots.size() < sparse.root_count);
    CHECK(sparse.stamped_roots.size() == 2);
    for (const StampedRootVolume &root : sparse.stamped_roots)
        CHECK(root.solid_surface);

    // A 100% surface without the option carries no target.
    const FullFillVolumeRun disabled = run_full_fill_volume_case(pattern_id, "100%", false);
    REQUIRE(disabled.root_count > 0);
    CHECK(disabled.stamped_roots.empty());
}

TEST_CASE("Full fill volume correction reaches the stamped target",
          "[plugins][infill][volume]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const char *pattern_id = GENERATE("gyroid", "hilbertcurve", "concentric");
    INFO(pattern_id);

    const FullFillVolumeComparison comparison =
        run_full_fill_volume_comparison(pattern_id, "100%", true, k_full_fill_volume_corrector);
    const FullFillVolumeRun &before = comparison.before;
    const FullFillVolumeRun &after = comparison.after;

    REQUIRE(before.stamped_roots.size() > 0);
    REQUIRE(after.stamped_roots.size() == before.stamped_roots.size());

    size_t unclamped = 0;
    double corrected_target_total = 0.;
    for (const StampedRootVolume &root : before.stamped_roots) {
        const StampedRootVolume *corrected = find_stamped_root(after, root.surface_id);
        REQUIRE(corrected != nullptr);
        REQUIRE(root.target_mm3 > 0.);
        REQUIRE(root.volume_mm3 > 0.);

        // The plugin compares the raw target with the normalized measurement.
        const double ratio = root.target_mm3 / (root.volume_mm3 * root.normalization);
        CHECK(corrected->volume_mm3 ==
              Approx(root.volume_mm3 * clamp_flow_mult(ratio)).epsilon(1e-6));
        if (ratio > 0.8 && ratio < 1.3) {
            ++unclamped;
            CHECK(corrected->volume_mm3 == Approx(corrected_target_mm3(root)).epsilon(0.02));
        }
        corrected_target_total += corrected_target_mm3(root);
    }

    // The scenario must exercise the normal, unclamped correction.
    CHECK(unclamped == before.stamped_roots.size());
    // The generated pattern volume never matches the free area exactly, so the
    // correction is meaningful, and it brings the total closer to the target.
    const double error_before = std::abs(stamped_volume_mm3(before) - corrected_target_total);
    const double error_after = std::abs(stamped_volume_mm3(after) - corrected_target_total);
    CHECK(error_before > corrected_target_total * 0.005);
    CHECK(error_after < error_before);
}

TEST_CASE("Full fill volume correction is a no-op on a second pass",
          "[plugins][infill][volume]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const char *pattern_id = GENERATE("gyroid", "hilbertcurve");
    INFO(pattern_id);

    // The scenario keeps the correction inside the failsafe bounds, so the
    // second pass measures the corrected volume and must not move it again.
    const std::pair<FullFillVolumeRun, FullFillVolumeRun> runs = run_full_fill_volume_twice(pattern_id);
    REQUIRE(runs.first.stamped_roots.size() > 0);
    REQUIRE(runs.second.stamped_roots.size() == runs.first.stamped_roots.size());
    for (const StampedRootVolume &root : runs.first.stamped_roots) {
        const StampedRootVolume *second = find_stamped_root(runs.second, root.surface_id);
        REQUIRE(second != nullptr);
        CHECK(second->volume_mm3 == Approx(root.volume_mm3).epsilon(1e-6));
    }
}

TEST_CASE("Full fill volume targets stay inert without the corrector",
          "[plugins][infill][volume]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const char *pattern_id = GENERATE("gyroid", "hilbertcurve");
    INFO(pattern_id);

    // The post-infill step runs but only the gap-fill placeholder is active, so
    // nothing may change the measured volume.
    const FullFillVolumeComparison comparison =
        run_full_fill_volume_comparison(pattern_id, "100%", true, k_post_infill_gap_fill);

    REQUIRE(comparison.before.stamped_roots.size() > 0);
    REQUIRE(comparison.after.stamped_roots.size() == comparison.before.stamped_roots.size());
    for (const StampedRootVolume &root : comparison.before.stamped_roots) {
        const StampedRootVolume *unchanged = find_stamped_root(comparison.after, root.surface_id);
        REQUIRE(unchanged != nullptr);
        CHECK(unchanged->volume_mm3 == Approx(root.volume_mm3).epsilon(1e-9));
    }
}

TEST_CASE("Reduced solid infill overlap is honored by standalone patterns",
          "[plugins][infill][volume]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    // Normalization keeps the intentional overlap voids open after the post-infill correction.
    const char *pattern_id = GENERATE("gyroid", "hilbertcurve", "rectilinear", "concentric");
    INFO(pattern_id);

    const std::initializer_list<ConfigOverride> reduced_overlap = {
        { "solid_infill_overlap", "50%" },
        { "top_solid_infill_overlap", "50%" }
    };

    const FullFillVolumeComparison comparison = run_full_fill_volume_comparison(
        pattern_id, "100%", true, k_full_fill_volume_corrector, reduced_overlap);
    const FullFillVolumeRun &before = comparison.before;
    const FullFillVolumeRun &after = comparison.after;

    REQUIRE(before.stamped_roots.size() > 0);
    REQUIRE(after.stamped_roots.size() == before.stamped_roots.size());

    size_t normalized_roots = 0;
    for (const StampedRootVolume &root : before.stamped_roots) {
        const StampedRootVolume *corrected = find_stamped_root(after, root.surface_id);
        REQUIRE(corrected != nullptr);
        REQUIRE(root.target_mm3 > 0.);
        REQUIRE(root.volume_mm3 > 0.);
        REQUIRE(root.normalization > 1.);

        CHECK(corrected->volume_mm3 * root.normalization ==
              Approx(root.target_mm3).epsilon(0.02));
        ++normalized_roots;
    }
    CHECK(normalized_roots > 0);
}
