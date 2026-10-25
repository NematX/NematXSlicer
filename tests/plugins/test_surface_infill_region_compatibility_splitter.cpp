// These tests build perimeter and surface geometry, then check how infill settings
// split one island into region groups. A region group owns its fill surfaces;
// island-wide validation checks their overlap and coverage in scaled coordinates.
#include <catch2/catch.hpp>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/internal/LayerAccess.hpp"
#include "libslic3r/Api/internal/LayerRegionAccess.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintRegion.hpp"
#include "libslic3r/Steps/StepGeneratePerimeter.hpp"
#include "libslic3r/Steps/StepSurfaceGeneration.hpp"
#include "libslic3r/Surface.hpp"
#include "libslic3r/SurfaceCollection.hpp"

#include <cmath>
#include <initializer_list>
#include <memory>
#include <string>
#include <vector>

namespace {
using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

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
            INFO("Activating test plugin " << plugin_id);
            REQUIRE(m_orchestrator.set_plugin_active(plugin_id, true));
        }
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

double area_sum(const ExPolygons &areas)
{
    double out = 0.;
    for (const ExPolygon &area : areas)
        out += std::abs(area.area());
    return out;
}

double area_tolerance()
{
    const double side = double(scale_i(0.005));
    return side * side;
}

ExPolygons surface_expolygons(const SurfaceCollection &surfaces)
{
    ExPolygons out;
    out.reserve(surfaces.size());
    for (const Surface &surface : surfaces)
        if (!surface.empty())
            out.push_back(surface.expolygon);
    return out;
}

void require_same_union(const ExPolygons &actual, const ExPolygons &expected)
{
    const ExPolygons actual_union = union_ex(actual);
    const ExPolygons expected_union = union_ex(expected);
    INFO("actual area " << area_sum(actual_union) << ", expected area " << area_sum(expected_union));
    REQUIRE(area_sum(diff_ex(actual_union, expected_union)) <= area_tolerance());
    REQUIRE(area_sum(diff_ex(expected_union, actual_union)) <= area_tolerance());
}

void require_no_positive_overlap(const ExPolygons &areas)
{
    for (size_t first_idx = 0; first_idx < areas.size(); ++first_idx)
        for (size_t second_idx = first_idx + 1; second_idx < areas.size(); ++second_idx) {
            const ExPolygons overlap = intersection_ex(ExPolygons{areas[first_idx]},
                                                       ExPolygons{areas[second_idx]});
            INFO("area pair " << first_idx << " / " << second_idx);
            CHECK(area_sum(overlap) <= area_tolerance());
        }
}

LayerRegionSetCPtrs region_set(const LayerRegion &region)
{
    LayerRegionSetCPtrs out;
    out.insert(&region);
    return out;
}

void set_region_area(LayerRegion &region, const ExPolygon &area)
{
    ExPolygons &region_slices = ApiInternal::LayerRegionAccess::slices_mutable(region);
    region_slices = ExPolygons{area};
    ApiInternal::LayerRegionAccess::surfaces_mutable(region).set(region_slices, stPosInternal | stDensSparse);
}

void replace_layer_island(Layer &layer, const ExPolygon &area)
{
    ApiInternal::LayerAccess::set_islands(layer, ExPolygons{area});
    set_region_area(layer.region(0), area);
    layer.island(0).fill_regions(layer);
}

void add_region_with_settings(PreparedPerimeterPrint &prepared,
                              Layer &layer,
                              const ExPolygon &area,
                              std::initializer_list<std::pair<std::string, std::string>> settings)
{
    PrintRegionConfig config = layer.region(0).region().config();
    for (const std::pair<std::string, std::string> &setting : settings)
        config.set_deserialize_strict(setting.first, setting.second);

    ApiInternal::LayerAccess::add_region(layer, prepared.print.register_print_region(config));
    set_region_area(layer.region(layer.region_count() - 1), area);
}

void replace_layer_island_with_two_regions(PreparedPerimeterPrint &prepared,
                                           Layer &layer,
                                           std::initializer_list<std::pair<std::string, std::string>> right_settings)
{
    const ExPolygon whole = rectangle_expolygon(-10., -10., 10., 10.);
    ApiInternal::LayerAccess::set_islands(layer, ExPolygons{whole});
    set_region_area(layer.region(0), rectangle_expolygon(-10., -10., 0., 10.));
    add_region_with_settings(prepared, layer, rectangle_expolygon(0., -10., 10., 10.), right_settings);
    layer.island(0).fill_regions(layer);
}

void rebuild_island_overlap_graph(PrintObject &object)
{
    for (Layer &layer : object.layers())
        for (LayerSliceIsland &island : layer.islands()) {
            island.overlaps_above.clear();
            island.overlaps_below.clear();
        }

    for (size_t layer_idx = 1; layer_idx < object.layer_count(); ++layer_idx)
        Layer::build_up_down_graph(object.layer(layer_idx - 1), object.layer(layer_idx));
}

void run_surface_splitter_pipeline(Print &print)
{
    ScopedActivePlugins active({SIMPLE_PERIMETER_GENERATOR,
                                INITIAL_TYPED_SURFACE_BUILDER,
                                CLEAN_INFILL_SURFACES,
                                INFILL_REGION_COMPATIBILITY_SPLITTER});
    Orchestrator &orchestrator = Orchestrator::instance();

    Steps::StepGeneratePerimeter::clear(print);
    Steps::StepGeneratePerimeter::run_step(orchestrator, print);
    Steps::StepSurfaceGeneration::clear(print);
    Steps::StepSurfaceGeneration::run_step(orchestrator, print);

    std::string validation_error;
    const bool valid_surface_tree = Steps::StepSurfaceGeneration::validate_post(print, &validation_error);
    INFO("Surface-generation post validation: " << validation_error);
    REQUIRE(valid_surface_tree);
}

const LayerRegionIsland *region_island_for_regions(const LayerSliceIsland &island,
                                                   const LayerRegionSetCPtrs &regions)
{
    for (const LayerRegionIsland &region_island : island.regions_islands())
        if (region_island.regions() == regions && !region_island.fill_surfaces().empty())
            return &region_island;
    return nullptr;
}

size_t non_empty_region_island_count(const LayerSliceIsland &island)
{
    size_t count = 0;
    for (const LayerRegionIsland &region_island : island.regions_islands())
        if (!region_island.fill_surfaces().empty())
            ++count;
    return count;
}

bool has_empty_surface_only_region_island(const LayerSliceIsland &island)
{
    for (const LayerRegionIsland &region_island : island.regions_islands())
        if (region_island.fill_surfaces().empty() && !region_island.has_extrusions())
            return true;
    return false;
}

void require_region_island_area(const LayerRegionIsland &region_island, const ExPolygons &expected)
{
    const ExPolygons surfaces = surface_expolygons(region_island.fill_surfaces());
    require_no_positive_overlap(surfaces);
    require_same_union(surfaces, expected);
}

} // namespace

TEST_CASE("InfillRegionCompatibilitySplitter keeps compatible regions grouped",
          "[plugins][surface-generation][infill-region-compatibility]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    // Two regions share one island and have identical infill-relevant settings.
    // The splitter should therefore leave the broad LayerRegionIsland intact:
    // keeping this fast path matters because most models do not need a final
    // region split before infill generation.
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({{"perimeters", "0"}}));
    PrintObject &object = prepared.print.object(0);
    Layer &layer = object.layer(0);
    replace_layer_island_with_two_regions(prepared, layer, {});
    rebuild_island_overlap_graph(object);

    run_surface_splitter_pipeline(prepared.print);

    const LayerSliceIsland &island = layer.island(0);
    const LayerRegionIsland *whole = region_island_for_regions(island, island.regions());
    REQUIRE(whole != nullptr);
    CHECK(non_empty_region_island_count(island) == 1);
    require_region_island_area(*whole, island.infill_areas());
}

TEST_CASE("InfillRegionCompatibilitySplitter splits solid surfaces by solid infill extruder",
          "[plugins][surface-generation][infill-region-compatibility]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    // Both regions use the same sparse infill extruder, so
    // InitialTypedSurfaceBuilder first creates one shared region island. The
    // top solid surface is later filled with solid_infill_extruder, and the
    // right region overrides that setting. The splitter must cut only at this
    // final compatibility boundary and host cleanup must remove the emptied
    // old broad region island.
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({
        {"perimeters", "0"},
        {"nozzle_diameter", "0.4,0.4"},
        {"solid_infill_extruder", "1"}
    }));
    PrintObject &object = prepared.print.object(0);
    Layer &layer = object.layer(0);
    replace_layer_island_with_two_regions(prepared, layer, {{"solid_infill_extruder", "2"}});
    rebuild_island_overlap_graph(object);

    run_surface_splitter_pipeline(prepared.print);

    const LayerSliceIsland &island = layer.island(0);
    const LayerRegionIsland *left = region_island_for_regions(island, region_set(layer.region(0)));
    const LayerRegionIsland *right = region_island_for_regions(island, region_set(layer.region(1)));
    REQUIRE(left != nullptr);
    REQUIRE(right != nullptr);
    CHECK_FALSE(has_empty_surface_only_region_island(island));

    const ExPolygons expected_left = intersection_ex(island.infill_areas(), layer.region(0).get_raw_slices());
    const ExPolygons expected_right = intersection_ex(island.infill_areas(), layer.region(1).get_raw_slices());
    require_region_island_area(*left, expected_left);
    require_region_island_area(*right, expected_right);
}

TEST_CASE("InfillRegionCompatibilitySplitter splits sparse surfaces by common infill settings",
          "[plugins][surface-generation][infill-region-compatibility]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    // Middle layers are internal sparse infill. region_gcode does not affect
    // the geometry, but it changes the emitted infill recipe, so surfaces that
    // cross a region_gcode boundary must be split before the infill plugin can
    // safely process them as one job.
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({{"perimeters", "0"}}));
    PrintObject &object = prepared.print.object(0);
    REQUIRE(object.layer_count() > 4);
    Layer &layer = object.layer(object.layer_count() / 2);
    replace_layer_island_with_two_regions(prepared, layer, {{"region_gcode", "G4 P1"}});
    rebuild_island_overlap_graph(object);

    run_surface_splitter_pipeline(prepared.print);

    const LayerSliceIsland &island = layer.island(0);
    const LayerRegionIsland *left = region_island_for_regions(island, region_set(layer.region(0)));
    const LayerRegionIsland *right = region_island_for_regions(island, region_set(layer.region(1)));
    REQUIRE(left != nullptr);
    REQUIRE(right != nullptr);

    const ExPolygons expected_left = intersection_ex(island.infill_areas(), layer.region(0).get_raw_slices());
    const ExPolygons expected_right = intersection_ex(island.infill_areas(), layer.region(1).get_raw_slices());
    require_region_island_area(*left, expected_left);
    require_region_island_area(*right, expected_right);
}

// Build a real two-group island, replace its surfaces with clipped halves, and
// check boundary rounding versus geometric defects. The print owns all modified
// surfaces; failed validation reports an error without changing the geometry.
TEST_CASE("Surface partition validation removes only locally thin rounding dust",
          "[plugins][surface][surface-generation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({{"perimeters", "0"}}));
    PrintObject &object = prepared.print.object(0);
    Layer &layer = object.layer(object.layer_count() / 2);
    replace_layer_island_with_two_regions(prepared, layer, {{"region_gcode", "G4 P1"}});
    rebuild_island_overlap_graph(object);
    run_surface_splitter_pipeline(prepared.print);

    LayerSliceIsland &island = layer.island(0);
    REQUIRE(non_empty_region_island_count(island) == 2);
    LayerRegionIsland *left = nullptr;
    LayerRegionIsland *right = nullptr;
    for (LayerRegionIsland &region_island : island.regions_islands()) {
        if (region_island.regions() == region_set(layer.region(0)))
            left = &region_island;
        if (region_island.regions() == region_set(layer.region(1)))
            right = &region_island;
    }
    REQUIRE(left != nullptr);
    REQUIRE(right != nullptr);

    coord_t overlap_width = 0;
    bool expect_overlap = false;
    bool expect_missing = false;
    bool expect_slit = false;
    SECTION("A two hundred scaled units overlap is accepted") {
        overlap_width = 200;
    }
    SECTION("A three hundred scaled units overlap is accepted") {
        // The StepSurfaceGeneration.cpp header explains the approximately
        // three-epsilon cutoff: Clipper cleans away the thin eroded core.
        overlap_width = 300;
    }
    SECTION("A four hundred twenty scaled units overlap is rejected") {
        overlap_width = 420;
        expect_overlap = true;
    }
    SECTION("A half millimetre overlap is rejected") {
        overlap_width = scale_i(0.5);
        expect_overlap = true;
    }
    SECTION("A two hundred scaled units missing slit is accepted") {
        overlap_width = -200;
        expect_slit = true;
    }
    SECTION("A square millimetre hole is rejected") {
        expect_missing = true;
    }

    // Integer vertices make each band width explicit in scaled units. Clipping each
    // half to infill_areas preserves the perimeter-generated outer boundary.
    const coord_t extent = scale_i(20.);
    const ExPolygon left_clip(Polygon(Points{
        Point(-extent, -extent), Point(overlap_width, -extent),
        Point(overlap_width, extent), Point(-extent, extent)}));
    ExPolygons left_areas = intersection_ex(island.infill_areas(), ExPolygons{left_clip});
    ExPolygons right_areas = intersection_ex(island.infill_areas(),
                                            ExPolygons{rectangle_expolygon(0., -20., 20., 20.)});
    if (expect_missing)
        right_areas = diff_ex(right_areas, ExPolygons{rectangle_expolygon(4.5, -0.5, 5.5, 0.5)});
    left->set_fill_surfaces().set(left_areas, stPosInternal | stDensSparse);
    right->set_fill_surfaces().set(right_areas, stPosInternal | stDensSparse);

    ExPolygons surfaces = left_areas;
    surfaces.insert(surfaces.end(), right_areas.begin(), right_areas.end());
    const ExPolygons surface_union = union_ex(surfaces);
    const ExPolygons overlap = union_ex(intersection_ex(left_areas, right_areas));
    const ExPolygons missing = diff_ex(island.infill_areas(), surface_union);
    const ExPolygons extra = diff_ex(surface_union, island.infill_areas());
    const double raw_extra_area = area_sum(extra);
    // Measure the actual clipping results before opening so the accepted bands
    // prove that opening, rather than ExPolygon clipping, removes their area.
    const double raw_overlap_area = area_sum(overlap);
    const double raw_missing_area = area_sum(missing);
    const double overlap_area = area_sum(opening_ex(overlap, SCALED_EPSILON));
    const double missing_area = area_sum(opening_ex(missing, SCALED_EPSILON));
    const double extra_area = area_sum(opening_ex(extra, SCALED_EPSILON));
    const double epsilon_squared = double(SCALED_EPSILON) * double(SCALED_EPSILON);
    INFO("Overlap raw area=" << raw_overlap_area << ", opened area=" << overlap_area
         << ", epsilon squared=" << epsilon_squared);
    INFO("Missing raw area=" << raw_missing_area << ", opened area=" << missing_area
         << "; extra raw area=" << raw_extra_area << ", opened area=" << extra_area);
    std::string validation_error;
    const bool valid = Steps::StepSurfaceGeneration::validate_post(prepared.print, &validation_error);
    INFO("Surface-generation post validation: " << validation_error);
    REQUIRE(raw_extra_area == 0.);
    REQUIRE(extra_area == 0.);
    if (overlap_width > 0)
        REQUIRE(raw_overlap_area == double(overlap_width) * double(scale_i(20.)));
    else if (expect_slit)
        REQUIRE(raw_missing_area == double(-overlap_width) * double(scale_i(20.)));
    if (expect_overlap) {
        REQUIRE(overlap_area > epsilon_squared);
        REQUIRE_FALSE(valid);
        REQUIRE(validation_error.find("overlap") != std::string::npos);
    } else if (expect_missing) {
        REQUIRE(missing_area > epsilon_squared);
        REQUIRE_FALSE(valid);
        REQUIRE(validation_error.find("Missing area") != std::string::npos);
    } else if (expect_slit) {
        REQUIRE(raw_missing_area > 0.);
        REQUIRE(raw_missing_area > epsilon_squared);
        REQUIRE(missing_area == 0.);
        REQUIRE(valid);
    } else {
        REQUIRE(raw_overlap_area > 0.);
        REQUIRE(raw_overlap_area > epsilon_squared);
        REQUIRE(overlap_area == 0.);
        REQUIRE(valid);
    }
}
