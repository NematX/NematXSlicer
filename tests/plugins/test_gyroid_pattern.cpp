#include <catch2/catch.hpp>

#include <algorithm>
#include <initializer_list>
#include <string>
#include <vector>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepGenerateInfill.hpp"
#include "libslic3r/Steps/StepGeneratePerimeter.hpp"
#include "libslic3r/Steps/StepSurfaceGeneration.hpp"

/*
Gyroid pattern plugin
=====================

The gyroid pattern is the reference INFILL_PATTERN plugin implemented with the
public plugin API only. These tests exercise STEP_INFILL directly: they build a
sliced cube, run perimeter and surface generation, then run the infill step with
only the plugins it needs.

One case checks that gyroid is registered as an INFILL_PATTERN provider. The
generation case uses ScopedActivePlugins to select the required perimeter,
surface, infill-generator and gyroid plugins, restoring the previous selection
when the scope ends. It inspects the published INFILLS buckets after STEP_INFILL
returns and requires more than one item under count_infill_paths()'s shallow
counting convention; it does not compare gyroid paths with a geometry reference.

The two cases run independently:

    Registration case
    |-- ensure_plugin_test_runtime_initialized()
    `-- get_plugin("gyroid") and check its INFILL_PATTERN step

    Generation case
    |-- prepare_cube_print() with gyroid at 20% density and no solid top/bottom
    |-- ScopedActivePlugins selects only the required plugins
    |-- run_until_infill()
    |   |-- clear and run StepGeneratePerimeter
    |   |-- clear and run StepSurfaceGeneration
    |   `-- clear and run StepGenerateInfill
    |       |-- discover recipes and initialize the used pattern
    |       `-- generate gyroid output and publish it through host callbacks
    |-- count_infill_paths() after return
    |   `-- visit objects, layers, islands and region-island INFILLS buckets
    |       `-- return the shallow count of entities and direct group children
    `-- require that count to be greater than one
*/

using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

namespace {

// Count one path per ungrouped entity and the direct children of each logical group.
// Do not recurse into nested groups: this measures shallow published output, not
// the total number of printable leaf paths. The gyroid generation test requires
// more than one counted item; it does not use this count to compare path geometry.
size_t count_infill_paths(const Print &print);

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

DynamicPrintConfig gyroid_test_config()
{
    return perimeter_config({
        { "fill_pattern", "gyroid" },
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

size_t count_infill_paths(const Print &print)
{
    size_t paths = 0;
    for (const PrintObject &object : print.objects())
        for (const Layer &layer : object.layers())
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region_island : island.regions_islands()) {
                    if (!region_island.has_extrusion(LayerRegionIsland::INFILLS))
                        continue;
                    for (const ExtrusionEntity *entity : region_island.extrusion(LayerRegionIsland::INFILLS).entities())
                        paths += entity->is_collection() ? entity->child_count() : 1;
                }
    return paths;
}

} // namespace

TEST_CASE("The gyroid plugin registers as an infill pattern provider", "[plugins][infill][gyroid]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    const Plugin *plugin = orchestrator.get_plugin("gyroid");
    REQUIRE(plugin != nullptr);
    CHECK(plugin->get_step() == INFILL_PATTERN);

    // The plugin contributes itself to the sparse pattern list.
    const ConfigOptionDef *fill_pattern = PrintConfigDef::instance().get("fill_pattern");
    REQUIRE(fill_pattern != nullptr);
    REQUIRE(fill_pattern->enum_def.get() != nullptr);
    const std::vector<std::string> &values = fill_pattern->enum_def->values();
    CHECK(std::find(values.begin(), values.end(), "gyroid") != values.end());
}

TEST_CASE("STEP_INFILL generates gyroid extrusion with the gyroid plugin", "[plugins][infill][gyroid]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, gyroid_test_config());

    ScopedActivePlugins active({
        SIMPLE_PERIMETER_GENERATOR,
        INITIAL_TYPED_SURFACE_BUILDER,
        "infill.generator.default",
        "gyroid"
    });
    run_until_infill(prepared);

    // The plugin produced extrusion; a failure would leave the bucket empty.
    // The clipped waves stay separate paths: merging them would extrude across
    // the holes of the surface.
    CHECK(count_infill_paths(prepared.print) > 1);
}
