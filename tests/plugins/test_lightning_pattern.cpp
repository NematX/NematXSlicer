#include <catch2/catch.hpp>

#include <tuple>
#include <set>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepGeneratePerimeter.hpp"
#include "libslic3r/Steps/StepSurfaceGeneration.hpp"
#include "libslic3r/EdgeGrid.hpp"
#include "libslic3r/Fill/Lightning/DistanceField.hpp"
#include "libslic3r/Fill/Lightning/TreeNode.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Plugins/Infill/InfillPaths.hpp"
#include "libslic3r/Plugins/Infill/Lightning/LightningPreparation.hpp"
#include "libslic3r/Plugins/Infill/Lightning/LightningDistanceField.hpp"
#include "libslic3r/Plugins/Infill/Lightning/LightningGenerator.hpp"
#include "libslic3r/Fill/Lightning/Generator.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Thread.hpp"
#include "libslic3r/Fill/FillLightning.hpp"
#include "libslic3r/Plugins/Infill/Lightning/LightningFill.hpp"
#include "libslic3r/Plugins/Infill/Lightning/LightningPattern.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include <atomic>
#include "test_data.hpp"
#include "gcode_test_helpers.hpp"
#include "libslic3r/Steps/StepGenerateInfill.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"
#include "libslic3r/Steps/StepPostSlicing.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include <chrono>
#include <sstream>
#include <oneapi/tbb/global_control.h>

/*
Test Lightning infill from its geometric preparation through published paths.
Small surfaces and inspectable distance fields isolate tree growth and branch
connection; other cases compare native and plugin results or run the slicing
steps together. Geometry snapshots and path measurements are taken from owned
fixtures rather than changing the production interfaces to expose test state.

Activation guards restore the providers selected before a case. The owning
Model and Print outlive every borrowed surface, layer and extrusion view used
by that case, including work performed on internal workers.

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- make_cube()
        |-- prepared.print.apply()
        |-- Steps::StepLayerHeightGeneration::run_step()
        |-- Steps::StepSlicing::run_step()
        |-- Steps::StepPostSlicing::run_step()
        `-- Steps::StepGeneratePerimeter::run_step()
*/

using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

namespace {
// Inspect the native field without changing its production interface.
class InspectableField : public FillLightning::DistanceField
{
public:
    using DistanceField::DistanceField;
    std::vector<std::pair<Point, coord_t>> cells() const
    {
        std::vector<std::pair<Point, coord_t>> out;
        for (const UnsupportedCell &cell : m_unsupported_points)
            out.emplace_back(cell.loc, cell.dist_to_boundary);
        return out;
    }
};

// Compare native branch conversion and connection on the same offset-component order as the API.
class InspectableLightningFiller : public FillLightning::Filler
{
public:
    using Filler::_fill_surface_single;
};

// Restore activation even when an assertion aborts a fixture.
class ActivePlugins
{
public:
    explicit ActivePlugins(std::initializer_list<const char *> ids) : m_orchestrator(Orchestrator::instance()),
        m_previous(m_orchestrator.active_plugins().begin(), m_orchestrator.active_plugins().end())
    {
        m_orchestrator.clear_active_plugins();
        for (const char *id : ids)
            REQUIRE(m_orchestrator.set_plugin_active(id, true));
    }
    ~ActivePlugins()
    {
        m_orchestrator.clear_active_plugins();
        for (Plugin *plugin : m_previous)
            m_orchestrator.set_plugin_active(plugin, true);
    }
private:
    Orchestrator &m_orchestrator;
    std::vector<Plugin *> m_previous;
};

std::string surface_snapshot(const Print &print)
{
    std::ostringstream snapshot;
    for (const PrintObject &object : print.objects())
        for (const Layer &layer : object.layers())
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region : island.regions_islands())
                    for (const Surface &surface : region.fill_surfaces()) {
                        snapshot << surface.id() << ':' << int(surface.surface_type) << ':' << surface.bridge_angle << ';';
                        for (const Polygon &polygon : to_polygons(surface.expolygon))
                            for (const Point &point : polygon.points)
                                snapshot << point.x() << ',' << point.y() << ';';
                    }
    return snapshot.str();
}
}

TEST_CASE("Lightning intersections choose the hit nearest the destination", "[plugins][lightning][core]")
{
    const Polygon outline = rectangle_expolygon(-5., -5., 5., 5.).contour;
    EdgeGrid::Grid grid;
    grid.set_bbox(BoundingBox(Point(-scale_i(10.), -scale_i(10.)), Point(scale_i(10.), scale_i(10.))));
    grid.create(outline, scale_i(4.));
    Point hit;
    REQUIRE(FillLightning::lineSegmentPolygonsIntersection(Point(-scale_i(7.), 0), Point(scale_i(7.), 0), grid, hit, scale_i(3.)));
    CHECK(hit == Point(scale_i(5.), 0));
    CHECK_FALSE(FillLightning::lineSegmentPolygonsIntersection(Point(-scale_i(7.), 0), Point(scale_i(7.), 0), grid, hit, scale_i(1.)));
    CHECK_FALSE(FillLightning::lineSegmentPolygonsIntersection(Point(-scale_i(2.), 0), Point(scale_i(2.), 0), grid, hit, scale_i(3.)));
}

TEST_CASE("Lightning unsupported points have a strict distance-band order", "[plugins][lightning][core]")
{
    const coord_t radius = scale_i(2.);
    const Polygons outline{rectangle_expolygon(-10., -10., 10., 10.).contour};
    const BoundingBox bounds(outline.front().points);
    const InspectableField field(radius, outline, bounds, outline);
    const std::vector<std::pair<Point, coord_t>> cells = field.cells();
    REQUIRE(cells.size() > 100);
    const auto key = [radius](const std::pair<Point, coord_t> &cell) {
        return std::make_tuple(cell.second / radius, PointHash{}(cell.first) % 191);
    };
    for (size_t i = 1; i < cells.size(); ++i)
        CHECK_FALSE(key(cells[i]) < key(cells[i - 1]));
    const InspectableField repeated(radius, outline, bounds, outline);
    CHECK(cells == repeated.cells());
}

TEST_CASE("Shared infill clipping retains complete bent paths", "[plugins][lightning][paths]")
{
    namespace work = slic3r_api::PluginCommon::polygon_work;
    const ExPolygon boundary = rectangle_with_hole_expolygon();
    const Polylines input{Polyline{Point(-scale_i(15.), 0), Point(-scale_i(6.), scale_i(6.)),
                                  Point(scale_i(6.), scale_i(6.)), Point(scale_i(15.), 0)}};
    work::Polylines subjects;
    for (const Polyline &path : input) {
        work::Polyline &converted = subjects.emplace_back();
        for (const Point &point : path.points)
            converted.points.emplace_back(point.x(), point.y());
    }
    const Polylines expected = intersection_pl(input, boundary);
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    const slic3r_api::StoredPolylineCollection stored = slic3r_api::Infill::Paths::store_polylines(handle, subjects);
    slic3r_api::StoredExPolygonCollection mask(handle);
    mask.push_back(slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&boundary)));
    const slic3r_api::StoredPolylineCollection clipped = slic3r_api::clipper_intersection_polylines_expolygons(handle, stored, mask);
    const work::Polylines actual = slic3r_api::Infill::Paths::read_polylines(clipped);
    REQUIRE(expected.size() == actual.size());
    bool retains_bend = false;
    for (size_t path = 0; path < expected.size(); ++path) {
        REQUIRE(expected[path].size() == actual[path].size());
        retains_bend = retains_bend || actual[path].size() > 2;
        for (size_t point = 0; point < expected[path].size(); ++point) {
            CHECK(expected[path].points[point].x() == actual[path].points[point].x());
            CHECK(expected[path].points[point].y() == actual[path].points[point].y());
        }
    }
    CHECK(retains_bend);
}

TEST_CASE("Batch API clipping retains directed boundary paths through work conversions", "[plugins][lightning][paths]")
{
    namespace paths = slic3r_api::Infill::Paths;
    namespace work = paths::work;
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    for (const ExPolygon &area : {rectangle_expolygon(-10., -10., 10., 10.), rectangle_with_hole_expolygon()})
        for (const coord_t shift : {coord_t(0), scale_i(1000.), -scale_i(1000.)}) {
            CAPTURE(area.holes.size(), shift);
            ExPolygon boundary = area;
            for (Point &point : boundary.contour.points)
                point += Point(shift, -shift);
            for (Polygon &hole : boundary.holes)
                for (Point &point : hole.points)
                    point += Point(shift, -shift);
            work::Polylines subjects{
                work::Polyline{work::Point(-scale_i(15.), 0), work::Point(-scale_i(6.), scale_i(6.)),
                               work::Point(scale_i(6.), scale_i(6.)), work::Point(scale_i(15.), 0)},
                work::Polyline{work::Point(-scale_i(15.), 0), work::Point(scale_i(15.), 0)},
                work::Polyline{work::Point(0, -scale_i(15.)), work::Point(0, scale_i(15.))},
                work::Polyline{work::Point(-scale_i(10.), -scale_i(10.)), work::Point(scale_i(10.), -scale_i(10.)),
                               work::Point(scale_i(10.), scale_i(10.))},
                work::Polyline{work::Point(-scale_i(15.), -scale_i(15.)), work::Point(scale_i(15.), scale_i(15.))},
                work::Polyline{work::Point(-scale_i(15.), 1), work::Point(scale_i(15.), 2)}
            };
            for (work::Polyline &line : subjects)
                line.translate(work::Point(shift, -shift));
            const slic3r_api::ExPolygon view(reinterpret_cast<const expolygon_handle *>(&boundary));
            Polylines input;
            for (const work::Polyline &line : subjects) {
                Polyline &path = input.emplace_back();
                for (const work::Point &point : line.points)
                    path.points.emplace_back(point.x(), point.y());
            }
            const Polylines expected = intersection_pl(input, boundary);
            const slic3r_api::StoredPolylineCollection stored = paths::store_polylines(handle, subjects);
            slic3r_api::StoredExPolygonCollection mask(handle);
            mask.push_back(view);
            const slic3r_api::StoredPolylineCollection clipped = slic3r_api::clipper_intersection_polylines_expolygons(handle, stored, mask);
            const work::Polylines actual = paths::read_polylines(clipped);
            REQUIRE(actual.size() == expected.size());
            for (size_t index = 0; index < actual.size(); ++index) {
                REQUIRE(actual[index].size() == expected[index].size());
                for (size_t point = 0; point < actual[index].size(); ++point) {
                    CHECK(actual[index][point].x() == expected[index].points[point].x());
                    CHECK(actual[index][point].y() == expected[index].points[point].y());
                }
            }
        }
}

TEST_CASE("Infill work conversions own their points and unwind cancelled copies", "[plugins][lightning][paths]")
{
    namespace paths = slic3r_api::Infill::Paths;
    namespace work = paths::work;
    work::Polylines subjects(1);
    for (coord_t index = 0; index < 2049; ++index)
        subjects.front().points.emplace_back(index * 1000, index % 2 == 0 ? -6000 : 6000);
    subjects.emplace_back();
    subjects.push_back(work::Polyline{work::Point(-12, -34)});
    subjects.push_back(work::Polyline{work::Point(-100, 0), work::Point(0, 100), work::Point(100, 0)});
    const work::Polylines original = subjects;
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    struct Probe { uint32_t checks = 0; uint32_t cancel_at = 0; } probe;
    plugin_run_context context = {};
    context.host_context = reinterpret_cast<plugin_host_context *>(&probe);
    context.is_cancelled = [](plugin_host_context *host) {
        Probe &probe = *reinterpret_cast<Probe *>(host);
        return int(++probe.checks >= probe.cancel_at);
    };
    for (const uint32_t cancel_at : {1u, 4u}) {
        probe = {0, cancel_at};
        CHECK_THROWS_AS(paths::store_polylines(handle, subjects, &context), slic3r_api::PluginCancelled);
        CHECK(probe.checks == cancel_at);
    }
    slic3r_api::StoredPolylineCollection stored = paths::store_polylines(handle, subjects);
    for (const uint32_t cancel_at : {1u, 4u}) {
        probe = {0, cancel_at};
        CHECK_THROWS_AS(paths::read_polylines(stored, &context), slic3r_api::PluginCancelled);
        CHECK(probe.checks == cancel_at);
    }
    subjects.front().points.front() = work::Point(99, 99);
    const work::Polylines copied = paths::read_polylines(stored);
    stored.clear();
    REQUIRE(copied.size() == original.size());
    for (size_t index = 0; index < original.size(); ++index)
        CHECK(copied[index].points == original[index].points);
    CHECK(paths::store_polylines(handle, {}).empty());
    CHECK(paths::read_polylines(stored).empty());
}

TEST_CASE("Lightning averages active region widths and densities", "[plugins][lightning][inputs]")
{
    namespace lightning = slic3r_api::Infill::Lightning;
    const lightning::Parameters parameters = lightning::make_parameters(
        {{0.2, 0.3, false}, {0.4, 0.9, false}, {0., 50., false}}, 0.4);
    CHECK(parameters.width_scaled == Approx(scale_d(0.6)));
    CHECK(parameters.supporting_radius == Approx(scale_d(2.)).margin(1.));
    const lightning::Parameters percent = lightning::make_parameters({{0.2, 120., true}}, 0.4);
    CHECK(percent.width_scaled == Approx(scale_d(double(1.125f * 0.4f) * 1.2)));
    CHECK_THROWS_AS(lightning::make_parameters({}, 0.4), std::runtime_error);
    CHECK_THROWS_AS(lightning::make_parameters({{0., 0.5, false}}, 0.4), std::runtime_error);
}

TEST_CASE("Lightning input snapshots use current surfaces and physical layer distances", "[plugins][lightning][inputs]")
{
    namespace lightning = slic3r_api::Infill::Lightning;
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({{"fill_pattern", "lightning"}, {"fill_density", "20%"},
        {"layer_height", "0.3"}, {"first_layer_height", "0.2"}, {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"}}));
    Orchestrator &orchestrator = Orchestrator::instance();
    const std::vector<Plugin *> previous(orchestrator.active_plugins().begin(), orchestrator.active_plugins().end());
    orchestrator.clear_active_plugins();
    REQUIRE(orchestrator.set_plugin_active(SIMPLE_PERIMETER_GENERATOR, true));
    REQUIRE(orchestrator.set_plugin_active(INITIAL_TYPED_SURFACE_BUILDER, true));
    Steps::StepGeneratePerimeter::run_step(orchestrator, prepared.print);
    Steps::StepSurfaceGeneration::run_step(orchestrator, prepared.print);
    orchestrator.clear_active_plugins();
    for (Plugin *plugin : previous)
        orchestrator.set_plugin_active(plugin, true);
    PrintObject &object = prepared.print.object(0);
    REQUIRE(object.layer_count() > 2);
    PluginStorage storage;
    const lightning::Inputs inputs = lightning::read_inputs(
        slic3r_api::Print(reinterpret_cast<const print_handle *>(&prepared.print)),
        slic3r_api::Object(reinterpret_cast<const object_handle *>(&object)),
        reinterpret_cast<storage_handle *>(&storage), nullptr);
    REQUIRE(inputs.layers.size() == object.layer_count());
    CHECK(inputs.parameters.supporting_radius > 0);
    size_t populated_layers = 0;
    for (size_t i = 0; i < inputs.layers.size(); ++i) {
        const lightning::LayerInput &input = inputs.layers[i];
        CHECK(input.identity == reinterpret_cast<const layer_handle *>(&object.layer(i)));
        CHECK(input.wall_radius == coord_t(double(object.layer(i).scaled_height()) * std::tan(PI / 4.)));
        if (i > 0)
            CHECK(input.prune_length == coord_t(double(object.layer(i).scaled_print_z() - object.layer(i - 1).scaled_print_z()) * std::tan(PI / 4.)));
        populated_layers += !input.outlines.empty();
    }
    CHECK(populated_layers > 1);
}

TEST_CASE("Lightning snapshots mixed regions and propagates physical middle-layer deltas",
          "[plugins][lightning][multi-region][variable-height][trees]")
{
    namespace lightning = slic3r_api::Infill::Lightning;
    namespace work = lightning::work;
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    const DynamicPrintConfig config = perimeter_config({{"fill_pattern", "rectilinear"}, {"fill_density", "20%"},
        {"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"}});
    struct RegionCase {
        const char *pattern;
        const char *density;
        const char *width;
        double size;
        double center;
    };
    const RegionCase regions[] = {
        {"lightning", "20%", "0.3", 8., -12.},
        {"lightning", "40%", "120%", 16., 0.},
        {"rectilinear", "80%", "1.2", 4., 10.},
        {"lightning", "0%", "0.9", 4., 14.}
    };
    ModelObject *model = prepared.model.add_object();
    for (const RegionCase &region : regions) {
        TriangleMesh mesh = make_cube(region.size, 20., 6.);
        mesh.translate(region.center, 0., 0.);
        ModelVolume *volume = model->add_volume(std::move(mesh));
        volume->config.set_deserialize_strict("fill_pattern", region.pattern);
        volume->config.set_deserialize_strict("fill_density", region.density);
        volume->config.set_deserialize_strict("infill_extrusion_width", region.width);
    }
    model->layer_height_profile.set(std::vector<coordf_t>{0., 0.2, 2., 0.2, 2.01, 0.1, 4., 0.1, 4.01, 0.3, 6., 0.3});
    model->add_instance();
    prepared.model.center_instances_around_point({100., 100.});
    model->ensure_on_bed();
    prepared.print.auto_assign_extruders(model);
    prepared.print.apply(prepared.model, config);
    Orchestrator &orchestrator = Orchestrator::instance();
    {
        // The upstream surface builder shares mutable storage; Lightning setup itself remains unrestricted.
        tbb::global_control serial(tbb::global_control::max_allowed_parallelism, 1);
        Steps::StepLayerHeightGeneration::run_step(orchestrator, prepared.print);
        Steps::StepSlicing::run_step(orchestrator, prepared.print);
        Steps::StepPostSlicing::run_step(orchestrator, prepared.print);
        ActivePlugins active({SIMPLE_PERIMETER_GENERATOR, INITIAL_TYPED_SURFACE_BUILDER});
        Steps::StepGeneratePerimeter::clear(prepared.print);
        Steps::StepGeneratePerimeter::run_step(orchestrator, prepared.print);
        Steps::StepSurfaceGeneration::clear(prepared.print);
        Steps::StepSurfaceGeneration::run_step(orchestrator, prepared.print);
    }
    PrintObject &object = prepared.print.object(0);
    REQUIRE(object.num_printing_regions() >= 4);
    // The initial builder leaves zero-density surfaces sparse; represent their downstream void classification explicitly.
    for (Layer &layer : object.layers())
        for (LayerSliceIsland &island : layer.islands())
            for (LayerRegionIsland &region : island.regions_islands())
                if ((*region.regions().begin())->region().config().option("fill_density")->serialize() == "0%")
                    for (Surface &surface : region.set_fill_surfaces().surfaces)
                        if (surface.surface_type & stPosInternal)
                            surface.surface_type = stPosInternal | stDensVoid;
    const std::string snapshot = surface_snapshot(prepared.print);
    PluginStorage storage;
    storage_handle *scratch = reinterpret_cast<storage_handle *>(&storage);
    const lightning::Inputs inputs = lightning::read_inputs(
        slic3r_api::Print(reinterpret_cast<const print_handle *>(&prepared.print)),
        slic3r_api::Object(reinterpret_cast<const object_handle *>(&object)), scratch, nullptr);
    const double expected_width = (0.3 + double(1.125f * 0.4f) * 1.2) / 2.;
    CHECK(inputs.parameters.width_scaled == Approx(scale_d(expected_width)));
    CHECK(inputs.parameters.supporting_radius == Approx(scale_d(expected_width / 0.3)).margin(1.));
    REQUIRE(inputs.layers.size() == object.layer_count());
    std::set<coord_t> middle_distances;
    std::set<const PrintRegion *> populated_regions;
    size_t void_surfaces = 0;
    size_t other_pattern_surfaces = 0;
    for (size_t index = 0; index < inputs.layers.size(); ++index) {
        const Layer &layer = object.layer(index);
        const lightning::LayerInput &input = inputs.layers[index];
        CAPTURE(index);
        CHECK(input.wall_radius == coord_t(double(layer.scaled_height()) * std::tan(PI / 4.)));
        if (index > 0) {
            const coord_t distance = coord_t(double(layer.scaled_print_z() - object.layer(index - 1).scaled_print_z()) * std::tan(PI / 4.));
            CHECK(input.prune_length == distance);
            CHECK(input.straightening_distance == distance);
            if (index > 1 && index + 1 < inputs.layers.size())
                middle_distances.insert(distance);
        }
        ExPolygons expected;
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const Surface &surface : region.fill_surfaces()) {
                    if (!(surface.surface_type & stPosInternal) || !(surface.surface_type & (stDensSparse | stDensVoid)))
                        continue;
                    expected.push_back(surface.expolygon);
                    for (const LayerRegion *layer_region : region.regions())
                        populated_regions.insert(&layer_region->region());
                    void_surfaces += bool(surface.surface_type & stDensVoid);
                    other_pattern_surfaces += (*region.regions().begin())->region().config().option("fill_pattern")->serialize() == "rectilinear";
                }
        Polygons actual;
        for (const work::Polygon &outline : input.outlines) {
            Polygon &polygon = actual.emplace_back();
            for (const work::Point &point : outline.points)
                polygon.points.emplace_back(point.x(), point.y());
        }
        CHECK(diff_ex(union_ex(expected), union_ex(actual)).empty());
        CHECK(diff_ex(union_ex(actual), union_ex(expected)).empty());
    }
    REQUIRE(middle_distances.size() >= 3);
    CHECK(populated_regions.size() == 4);
    CHECK(void_surfaces > 0);
    CHECK(other_pattern_surfaces > 0);
    const std::unique_ptr<lightning::Cache> cache = lightning::build_cache(inputs, scratch, nullptr);
    REQUIRE(cache->layers.size() == inputs.layers.size());
    size_t layers_with_roots = 0;
    for (const lightning::LayerTree &layer : cache->layers)
        layers_with_roots += !layer.roots.empty();
    CHECK(layers_with_roots > 1);

    work::Polygon outline;
    outline.points = {work::Point(-scale_i(20.), -scale_i(20.)), work::Point(scale_i(20.), -scale_i(20.)),
                      work::Point(scale_i(20.), scale_i(20.)), work::Point(-scale_i(20.), scale_i(20.))};
    const work::Polygons outlines{outline};
    lightning::grid::Grid locator;
    locator.create(outlines, lightning::locator_cell_size);
    const Point bend(scale_i(5.), scale_i(2.)), leaf(scale_i(9.), 0);
    for (const coord_t delta : middle_distances) {
        CAPTURE(delta);
        const lightning::NodePtr root = std::make_shared<lightning::Node>(work::Point(0, 0));
        const lightning::NodePtr child = root->add_child(work::Point(bend.x(), bend.y()));
        child->add_child(work::Point(leaf.x(), leaf.y()));
        std::vector<lightning::NodePtr> below;
        root->propagate(below, outlines, locator, delta, delta, nullptr);
        REQUIRE(below.size() == 1);
        REQUIRE(below.front()->children.size() == 1);
        const lightning::NodePtr moved = below.front()->children.front();
        REQUIRE(moved->children.size() == 1);
        // This long single-child chain cannot collapse; pruning and straightening have independent analytic endpoints.
        const Vec2d leaf_direction = (bend - leaf).cast<double>().normalized();
        const Point expected_leaf = leaf + Point(coord_t(double(delta) * leaf_direction.x()), coord_t(double(delta) * leaf_direction.y()));
        const coord_t first_length = coord_t(bend.norm());
        const coord_t total_length = first_length + coord_t((expected_leaf - bend).norm());
        const Point destination(expected_leaf.x() * int64_t(first_length) / total_length,
                                expected_leaf.y() * int64_t(first_length) / total_length);
        const Vec2d movement = (destination - bend).cast<double>().normalized();
        const Point expected_bend = bend + Point(coord_t(double(delta) * movement.x()), coord_t(double(delta) * movement.y()));
        const work::Polylines paths = below.front()->paths(0);
        REQUIRE(paths.size() == 1);
        const work::Points expected_path{work::Point(expected_leaf.x(), expected_leaf.y()),
                                        work::Point(expected_bend.x(), expected_bend.y()), work::Point(0, 0)};
        CHECK(paths.front().points == expected_path);
        CHECK(child->location == work::Point(bend.x(), bend.y()));
        CHECK(child->children.front()->location == work::Point(leaf.x(), leaf.y()));
    }
    CHECK(surface_snapshot(prepared.print) == snapshot);
}

TEST_CASE("Lightning distance field matches native samples and removal", "[plugins][lightning][field]")
{
    namespace lightning = slic3r_api::Infill::Lightning;
    namespace work = lightning::work;
    PluginStorage storage;
    for (const ExPolygon &polygon : {rectangle_expolygon(-10., -10., 10., 10.), rectangle_with_hole_expolygon()}) {
        const Polygons outlines = to_polygons(polygon);
        const BoundingBox bounds = get_extents(outlines);
        work::Polygons input;
        for (const Polygon &contour : outlines) {
            work::Polygon &converted = input.emplace_back();
            for (const Point &point : contour.points)
                converted.points.emplace_back(point.x(), point.y());
        }
        InspectableField native(scale_i(2.), outlines, bounds, outlines);
        const work::BoundingBox box(work::Point(bounds.min.x(), bounds.min.y()), work::Point(bounds.max.x(), bounds.max.y()));
        lightning::DistanceField field(scale_i(2.), box, input, reinterpret_cast<storage_handle *>(&storage), nullptr);
        const std::vector<std::pair<Point, coord_t>> cells = native.cells();
        REQUIRE(cells.size() == field.cells().size());
        for (size_t index = 0; index < cells.size(); ++index) {
            REQUIRE(cells[index].first.x() == field.cells()[index].location.x());
            REQUIRE(cells[index].first.y() == field.cells()[index].location.y());
            REQUIRE(cells[index].second == field.cells()[index].distance);
        }
        native.update(Point(-scale_i(8.), -scale_i(8.)), Point(-scale_i(3.), scale_i(7.)));
        field.update(work::Point(-scale_i(8.), -scale_i(8.)), work::Point(-scale_i(3.), scale_i(7.)));
        size_t native_index = 0, plugin_index = 0;
        Point next;
        work::Point ported;
        while (native.tryGetNextPoint(&next, &native_index, native_index)) {
            REQUIRE(field.next(ported, plugin_index, plugin_index));
            CHECK(native_index == plugin_index);
            CHECK(next.x() == ported.x());
            CHECK(next.y() == ported.y());
            ++native_index;
            ++plugin_index;
        }
        CHECK_FALSE(field.next(ported, plugin_index, plugin_index));
    }
}

TEST_CASE("Lightning layer trees match native top-down generation", "[plugins][lightning][trees]")
{
    namespace lightning = slic3r_api::Infill::Lightning;
    namespace work = lightning::work;
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({{"fill_pattern", "lightning"}, {"fill_density", "20%"},
                                                  {"infill_extrusion_width", "0.4"}}));
    PrintObject &object = prepared.print.object(0);
    lightning::Inputs inputs;
    inputs.parameters = lightning::make_parameters({{0.2, 0.4, false}}, 0.4);
    for (size_t index = 0; index < object.layer_count(); ++index) {
        const ExPolygon area = index < object.layer_count() / 2 ? rectangle_with_hole_expolygon() :
                              rectangle_expolygon(-10., -10., 10., 10.);
        object.layer(index).region(0).set_fill_surfaces().set(ExPolygons{area}, stPosInternal | stDensSparse);
        lightning::LayerInput &input = inputs.layers.emplace_back();
        input.identity = reinterpret_cast<const layer_handle *>(&object.layer(index));
        input.wall_radius = coord_t(scale_d(1.) * std::tan(PI / 4.));
        input.prune_length = input.wall_radius;
        input.straightening_distance = input.wall_radius;
        for (const Polygon &polygon : union_(to_polygons(area))) {
            work::Polygon &converted = input.outlines.emplace_back();
            for (const Point &point : polygon.points)
                converted.points.emplace_back(point.x(), point.y());
        }
    }
    PluginStorage storage;
    const std::unique_ptr<lightning::Cache> cache = lightning::build_cache(inputs, reinterpret_cast<storage_handle *>(&storage), nullptr);
    FillLightning::Generator reference(object, 20., [] {});
    thread_data().seed_random_generator(111);
    Polylines raw_native;
    for (const FillLightning::NodeSPtr &root : reference.getTreesForLayer(object.layer_count() - 1).tree_roots)
        root->convertToPolylines(raw_native, scale_i(0.2));
    thread_data().seed_random_generator(111);
    const work::Polylines raw_plugin = cache->layers.back().paths(scale_i(0.2));
    REQUIRE(raw_native.size() == raw_plugin.size());
    for (size_t path = 0; path < raw_native.size(); ++path) {
        REQUIRE(raw_native[path].size() == raw_plugin[path].size());
        for (size_t point = 0; point < raw_native[path].size(); ++point) {
            REQUIRE(raw_native[path].points[point].x() == raw_plugin[path].points[point].x());
            REQUIRE(raw_native[path].points[point].y() == raw_plugin[path].points[point].y());
        }
    }
    REQUIRE(cache->layers.size() == object.layer_count());
    for (size_t layer = 0; layer < cache->layers.size(); ++layer) {
        INFO("layer " << layer);
        const ExPolygon limit = rectangle_expolygon(-30., -30., 30., 30.);
        Slic3r::thread_data().seed_random_generator(987);
        const Polylines expected = reference.getTreesForLayer(layer).convertToLines(
            to_polygons(limit), scale_i(0.2));
        Slic3r::thread_data().seed_random_generator(987);
        const slic3r_api::StoredPolylineCollection subjects = slic3r_api::Infill::Paths::store_polylines(
            reinterpret_cast<storage_handle *>(&storage), cache->layers[layer].paths(scale_i(0.2)));
        slic3r_api::StoredExPolygonCollection mask(reinterpret_cast<storage_handle *>(&storage));
        mask.push_back(slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&limit)));
        const slic3r_api::StoredPolylineCollection clipped = slic3r_api::clipper_intersection_polylines_expolygons(
            reinterpret_cast<storage_handle *>(&storage), subjects, mask);
        const work::Polylines actual = slic3r_api::Infill::Paths::read_polylines(clipped);
        REQUIRE(expected.size() == actual.size());
        // Only collection order is ignored; every path's point order remains exact.
        using Path = std::vector<std::pair<coord_t, coord_t>>;
        std::vector<Path> core_paths, plugin_paths;
        for (const Polyline &path : expected) {
            Path coordinates;
            for (const Point &point : path.points)
                coordinates.emplace_back(point.x(), point.y());
            core_paths.push_back(coordinates);
        }
        for (const work::Polyline &path : actual) {
            Path coordinates;
            for (const work::Point &point : path.points)
                coordinates.emplace_back(point.x(), point.y());
            plugin_paths.push_back(coordinates);
        }
        std::sort(core_paths.begin(), core_paths.end());
        std::sort(plugin_paths.begin(), plugin_paths.end());
        REQUIRE(core_paths == plugin_paths);
    }
}

TEST_CASE("Lightning propagation copies its source and drops empty destinations", "[plugins][lightning][trees]")
{
    namespace lightning = slic3r_api::Infill::Lightning;
    namespace work = lightning::work;
    const lightning::NodePtr root = std::make_shared<lightning::Node>(work::Point(0, 0));
    const lightning::NodePtr child = root->add_child(work::Point(scale_i(5.), 0));
    child->add_child(work::Point(scale_i(8.), scale_i(3.)));
    const work::Point original = child->location;
    work::Polygon outline;
    outline.points = {work::Point(-scale_i(10.), -scale_i(10.)), work::Point(scale_i(10.), -scale_i(10.)),
                      work::Point(scale_i(10.), scale_i(10.)), work::Point(-scale_i(10.), scale_i(10.))};
    const work::Polygons outlines{outline};
    lightning::grid::Grid locator;
    locator.create(outlines, lightning::locator_cell_size);
    std::vector<lightning::NodePtr> lower;
    root->propagate(lower, outlines, locator, scale_i(1.), scale_i(0.2), nullptr);
    REQUIRE_FALSE(lower.empty());
    CHECK(lower.front() != root);
    CHECK(child->location == original);
    CHECK(child->parent.lock() == root);
    std::vector<lightning::NodePtr> empty;
    root->propagate(empty, {}, locator, scale_i(1.), scale_i(0.2), nullptr);
    CHECK(empty.empty());
    CHECK(root->children.size() == 1);
}

namespace {
// Collect complete directed leaves; sortable group order is not part of pattern parity.
using PathCoordinates = std::vector<std::pair<coord_t, coord_t>>;
void collect_output(const slic3r_api::ExtrusionEntity &entity, std::vector<PathCoordinates> &paths)
{
    if (entity.is_leaf()) {
        if (entity.point_count() < 2)
            return;
        PathCoordinates &path = paths.emplace_back();
        for (uint32_t index = 0; index < entity.point_count(); ++index) {
            const c_point point = entity.point(index);
            path.emplace_back(point.x, point.y);
        }
    } else
        for (const slic3r_api::ExtrusionEntity child : entity.children())
            collect_output(child, paths);
}
}

TEST_CASE("Lightning surface filling matches native paths and connections", "[plugins][lightning][fill]")
{
    namespace lightning = slic3r_api::Infill::Lightning;
    namespace work = lightning::work;
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({{"fill_pattern", "lightning"}, {"fill_density", "20%"},
                                                  {"infill_extrusion_width", "0.4"}}));
    PrintObject &object = prepared.print.object(0);
    lightning::Inputs inputs;
    inputs.parameters = lightning::make_parameters({{0.2, 0.4, false}}, 0.4);
    const ExPolygon area = rectangle_with_hole_expolygon();
    for (size_t index = 0; index < object.layer_count(); ++index) {
        object.layer(index).region(0).set_fill_surfaces().set(ExPolygons{area}, stPosInternal | stDensSparse);
        lightning::LayerInput &layer = inputs.layers.emplace_back();
        layer.identity = reinterpret_cast<const layer_handle *>(&object.layer(index));
        layer.wall_radius = coord_t(scale_d(1.) * std::tan(PI / 4.));
        layer.prune_length = layer.straightening_distance = layer.wall_radius;
        for (const Polygon &polygon : union_(to_polygons(area))) {
            work::Polygon &converted = layer.outlines.emplace_back();
            for (const Point &point : polygon.points)
                converted.points.emplace_back(point.x(), point.y());
        }
    }
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    const std::unique_ptr<lightning::Cache> cache = lightning::build_cache(inputs, handle, nullptr);
    FillLightning::Generator reference(object, 20., [] {});
    const ExPolygons narrow_bridge = union_ex(ExPolygons{rectangle_expolygon(-10., -5., -4., 5.),
        rectangle_expolygon(-4., -0.1, 4., 0.1), rectangle_expolygon(4., -5., 10., 5.)});
    REQUIRE(narrow_bridge.size() == 1);
    REQUIRE(offset_ex(narrow_bridge, -scale_i(0.2)).size() == 2);
    for (const ExPolygon &boundary : {area, narrow_bridge.front()})
    for (const InfillConnection connection : {InfillConnection::icConnected, InfillConnection::icNotConnected,
                                               InfillConnection::icHoles, InfillConnection::icOuterShell})
      for (const std::pair<float, float> &anchors : {std::pair<float, float>{0.f, 0.f}, {2.f, 5.f}}) {
        CAPTURE(boundary.holes.size(), connection, anchors.first, anchors.second);
        thread_data().seed_random_generator(111);
        Polylines raw_native;
        for (const FillLightning::NodeSPtr &root : reference.getTreesForLayer(object.layer_count() - 1).tree_roots)
            root->convertToPolylines(raw_native, scale_i(0.2));
        thread_data().seed_random_generator(111);
        const work::Polylines raw_plugin = cache->layers.back().paths(scale_i(0.2));
        REQUIRE(raw_native.size() == raw_plugin.size());
        for (size_t path = 0; path < raw_native.size(); ++path) {
            CAPTURE(path);
            REQUIRE(raw_native[path].size() == raw_plugin[path].size());
            for (size_t point = 0; point < raw_native[path].size(); ++point) {
                REQUIRE(raw_native[path].points[point].x() == raw_plugin[path].points[point].x());
                REQUIRE(raw_native[path].points[point].y() == raw_plugin[path].points[point].y());
            }
        }
        FillParams params;
        params.config = &object.printing_region(0).config();
        params.density = 0.2f;
        params.connection = connection;
        params.anchor_length = anchors.first;
        params.anchor_length_max = anchors.second;
        params.fill_resolution = scale_i(0.0125);
        InspectableLightningFiller filler;
        filler.generator = &reference;
        filler.layer_id = object.layer_count() - 1;
        filler.angle = 0.f;
        filler.z = 1.;
        filler.set_bounding_box(get_extents(boundary));
        filler.init_spacing(0.4, params);
        Surface surface(stPosInternal | stDensSparse, boundary);
        const slic3r_api::StoredExPolygonCollection api_inset = slic3r_api::clipper_offset(
            slic3r_api::ClipperOperand(handle, slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&boundary))),
            -scale_d(0.2)).to_expolygon_collection();
        thread_data().seed_random_generator(111);
        Polylines expected;
        if (api_inset.size() == 1) {
            expected = filler.fill_surface(&surface, params);
        } else {
            const ExPolygons native_inset = offset_ex(boundary, -scale_d(0.2));
            REQUIRE(api_inset.size() == native_inset.size());
            for (const slic3r_api::ExPolygon component : api_inset) {
                const c_bounding_box box = component.contour().bounding_box();
                const ExPolygons::const_iterator native = std::find_if(native_inset.begin(), native_inset.end(),
                    [&box](const ExPolygon &candidate) {
                        const BoundingBox bounds = get_extents(candidate);
                        return bounds.min == Point(box.min.x, box.min.y) && bounds.max == Point(box.max.x, box.max.y);
                    });
                REQUIRE(native != native_inset.end());
                REQUIRE(component.hole_size() == 0);
                ExPolygon api_geometry;
                const multipoint_const_view contour = component.contour().view();
                for (uint32_t index = 0; index < contour.size; ++index)
                    api_geometry.contour.points.emplace_back(contour.array[index].x, contour.array[index].y);
                CHECK(diff_ex(ExPolygons{*native}, ExPolygons{api_geometry}).empty());
                CHECK(diff_ex(ExPolygons{api_geometry}, ExPolygons{*native}).empty());
                filler._fill_surface_single(params, surface.thickness_layers, {0.f, Point(0, 0)}, *native, expected);
                ensure_valid(expected, params.fill_resolution);
            }
        }
        raw_infill_pattern_params raw = {};
        raw.density = params.density;
        raw.spacing = 0.4;
        raw.connection = static_cast<c_infill_connection>(connection);
        raw.anchor_length = anchors.first;
        raw.anchor_length_max = anchors.second;
        raw.fill_resolution = params.fill_resolution;
        raw.flow.width = scale_i(0.4);
        raw.flow.height = scale_i(0.2);
        raw.flow.mm3_per_mm = 0.07;
        raw.flow_mult = 1.;
        raw.extrusion_role = RAW_EXTRUSION_ROLE_INTERNAL_INFILL;
        slic3r_api::StoredExtrusionEntity output(handle);
        run_ctx_infill_pattern context = {};
        context.layer = inputs.layers.back().identity;
        context.surface = reinterpret_cast<const surface_handle *>(&surface);
        context.params = &raw;
        context.output = output.mutable_handle();
        thread_data().seed_random_generator(111);
        lightning::fill_surface(context, *cache, handle, nullptr);
        std::vector<PathCoordinates> actual, core;
        collect_output(output.readonly(), actual);
        for (const Polyline &path : expected) {
            PathCoordinates &coordinates = core.emplace_back();
            for (const Point &point : path.points)
                coordinates.emplace_back(point.x(), point.y());
        }
        REQUIRE_FALSE(core.empty());
        std::sort(core.begin(), core.end());
        std::sort(actual.begin(), actual.end());
        REQUIRE(core.size() == actual.size());
        for (size_t path = 0; path < core.size(); ++path) {
            CAPTURE(path);
            std::ostringstream dump;
            for (const auto &point : core[path])
                dump << "core " << point.first << ',' << point.second << ';';
            for (const auto &point : actual[path])
                dump << "plugin " << point.first << ',' << point.second << ';';
            INFO(dump.str());
            REQUIRE(core[path].size() == actual[path].size());
            for (size_t point = 0; point < core[path].size(); ++point) {
                CAPTURE(point);
                REQUIRE(core[path][point].first == actual[path][point].first);
                REQUIRE(core[path][point].second == actual[path][point].second);
            }
        }
        if (api_inset.size() > 1 && connection == InfillConnection::icNotConnected && anchors.first == 0.f) {
            struct Probe { size_t checks = 0; size_t cancel_at = 0; } probe;
            plugin_run_context run = {};
            run.host_context = reinterpret_cast<plugin_host_context *>(&probe);
            run.is_cancelled = [](plugin_host_context *host) {
                Probe &probe = *reinterpret_cast<Probe *>(host);
                ++probe.checks;
                return int(probe.cancel_at > 0 && probe.checks >= probe.cancel_at);
            };
            slic3r_api::StoredExtrusionEntity counted(handle);
            context.output = counted.mutable_handle();
            thread_data().seed_random_generator(111);
            lightning::fill_surface(context, *cache, handle, &run);
            REQUIRE_FALSE(counted.empty());
            REQUIRE(probe.checks > 1);
            // Cancel at the final checkpoint after earlier components have built their unpublished paths.
            probe.cancel_at = probe.checks;
            probe.checks = 0;
            slic3r_api::StoredExtrusionEntity cancelled(handle);
            context.output = cancelled.mutable_handle();
            thread_data().seed_random_generator(111);
            CHECK_THROWS_AS(lightning::fill_surface(context, *cache, handle, &run), slic3r_api::PluginCancelled);
            CHECK(probe.checks == probe.cancel_at);
            CHECK(cancelled.empty());
        }
    }
}

TEST_CASE("Lightning setup builds isolated generation caches and clears cancelled work", "[plugins][lightning][setup]")
{
    namespace lightning = slic3r_api::Infill::Lightning;
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({{"fill_pattern", "lightning"}, {"fill_density", "20%"},
                                                  {"infill_extrusion_width", "0.4"}}));
    Orchestrator &orchestrator = Orchestrator::instance();
    const std::vector<Plugin *> previous(orchestrator.active_plugins().begin(), orchestrator.active_plugins().end());
    orchestrator.clear_active_plugins();
    REQUIRE(orchestrator.set_plugin_active(SIMPLE_PERIMETER_GENERATOR, true));
    REQUIRE(orchestrator.set_plugin_active(INITIAL_TYPED_SURFACE_BUILDER, true));
    Steps::StepGeneratePerimeter::run_step(orchestrator, prepared.print);
    Steps::StepSurfaceGeneration::run_step(orchestrator, prepared.print);
    orchestrator.clear_active_plugins();
    for (Plugin *plugin : previous)
        orchestrator.set_plugin_active(plugin, true);
    PrintObject &object = prepared.print.object(0);
    for (Layer &layer : object.layers())
        for (LayerSliceIsland &island : layer.islands())
            for (LayerRegionIsland &region : island.regions_islands())
                region.set_fill_surfaces().set(ExPolygons{rectangle_expolygon(-9., -9., 9., 9.)}, stPosInternal | stDensSparse);
    lightning::LightningPattern pattern(reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    const plugin_instance instance = pattern.c_instance();
    raw_infill_pattern_assignment assignment = {};
    raw_infill_pattern_object_init job = {};
    job.object = reinterpret_cast<const object_handle *>(&object);
    job.assignments = &assignment;
    job.assignment_count = 1;
    run_ctx_infill_pattern_init init = {};
    init.print = reinterpret_cast<const print_handle *>(&prepared.print);
    init.objects = &job;
    init.object_count = 1;
    struct Probe { std::atomic_bool cancel{false}; std::string error; } probe;
    PluginStorage storage;
    plugin_run_context context = {};
    context.step = INFILL_PATTERN;
    context.data = &init;
    context.plugin_storage = reinterpret_cast<storage_handle *>(&storage);
    context.host_context = reinterpret_cast<plugin_host_context *>(&probe);
    context.is_cancelled = [](plugin_host_context *ctx) { return int(reinterpret_cast<Probe *>(ctx)->cancel.load()); };
    context.report_error = [](plugin_host_context *ctx, const char *message) { reinterpret_cast<Probe *>(ctx)->error = message; };
    instance.vt->setup(instance.ctx, &context, 1);
    REQUIRE(probe.error.empty());
    raw_infill_pattern_params raw = {};
    raw.spacing = 0.4;
    raw.connection = C_INFILL_CONNECTION_NOT_CONNECTED;
    raw.fill_resolution = scale_i(0.0125);
    raw.flow_mult = 1.;
    Surface surface(stPosInternal | stDensSparse, rectangle_expolygon(-9., -9., 9., 9.));
    const auto render = [&]() {
        slic3r_api::StoredExtrusionEntity output(context.plugin_storage);
        run_ctx_infill_pattern run = {};
        run.object = job.object;
        run.layer = reinterpret_cast<const layer_handle *>(&object.layer(object.layer_count() - 1));
        run.surface = reinterpret_cast<const surface_handle *>(&surface);
        run.params = &raw;
        run.output = output.mutable_handle();
        context.data = &run;
        instance.vt->run(instance.ctx, &context);
        return output.child_count();
    };
    CHECK(render() > 0);
    probe.cancel = true;
    context.data = &init;
    instance.vt->setup(instance.ctx, &context, 1);
    probe.cancel = false;
    CHECK(render() == 0);
    CHECK(probe.error.find("was not prepared") != std::string::npos);
}

TEST_CASE("STEP_INFILL generates Lightning on two objects without modifying surfaces", "[plugins][lightning][infill]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    const Plugin *provider = orchestrator.get_plugin("lightning");
    REQUIRE(provider != nullptr);
    CHECK(provider->get_description().find("setup-owned") != std::string::npos);
    PreparedPerimeterPrint prepared;
    INFO("initializing two objects");
    Slic3r::Test::init_print({make_cube(20., 20., 10.), make_cube(16., 12., 8.)}, prepared.print, prepared.model,
        perimeter_config({{"fill_pattern", "lightning"}, {"fill_density", "20%"},
            {"top_fill_pattern", "rectilinear"}, {"bottom_fill_pattern", "rectilinear"},
            {"solid_fill_pattern", "rectilinear"}, {"top_solid_layers", "1"}, {"bottom_solid_layers", "0"}}));
    INFO("layer heights");
    // Upstream object plugins share mutable storage; release this fixture-only limit before Lightning setup.
    std::unique_ptr<tbb::global_control> upstream = std::make_unique<tbb::global_control>(tbb::global_control::max_allowed_parallelism, 1);
    Steps::StepLayerHeightGeneration::run_step(orchestrator, prepared.print);
    Steps::StepSlicing::run_step(orchestrator, prepared.print);
    Steps::StepPostSlicing::run_step(orchestrator, prepared.print);
    INFO("perimeters and surfaces");
    ActivePlugins active({SIMPLE_PERIMETER_GENERATOR, INITIAL_TYPED_SURFACE_BUILDER, SOLID_SHELLS,
                           "infill.generator.default", "lightning", "rectilinear"});
    Steps::StepGeneratePerimeter::run_step(orchestrator, prepared.print);
    Steps::StepSurfaceGeneration::run_step(orchestrator, prepared.print);
    upstream.reset();
    const std::string before = surface_snapshot(prepared.print);
    INFO("Lightning setup and infill");
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    Steps::StepGenerateInfill::clear(prepared.print);
    Steps::StepGenerateInfill::run_step(orchestrator, prepared.print);
    CHECK(surface_snapshot(prepared.print) == before);
    REQUIRE(prepared.print.objects().size() == 2);
    for (const PrintObject &object : prepared.print.objects()) {
        size_t sparse_paths = 0;
        const auto inspect = [&](const ExtrusionEntity &entity, const auto &self) -> void {
            if (entity.is_leaf()) {
                const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
                if (attributes && attributes->role == ExtrusionRole::InternalInfill && entity.has_polyline()) {
                    ++sparse_paths;
                    CHECK(attributes->mm3_per_mm > 0.);
                    CHECK(attributes->width > 0.f);
                    CHECK(attributes->height > 0.f);
                }
            } else
                for (const ExtrusionEntityUPtr &child : entity.children())
                    self(*child, self);
        };
        for (const Layer &layer : object.layers())
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region : island.regions_islands())
                    if (region.has_extrusion(LayerRegionIsland::INFILLS))
                        for (const ExtrusionEntity *entity : region.extrusion(LayerRegionIsland::INFILLS).entities())
                            inspect(*entity, inspect);
        CHECK(sparse_paths > 1);
    }
    const double elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    INFO("Lightning two-object infill milliseconds " << elapsed_ms);
}

TEST_CASE("Lightning exports through the existing G-code pipeline", "[plugins][lightning][gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("fill_pattern", "lightning");
    config.set_deserialize_strict("fill_density", "20%");
    config.set_deserialize_strict("gcode_firmware_plugin", "gcode.firmware.marlin2");
    PreparedPerimeterPrint prepared;
    Slic3r::Test::init_print({Slic3r::Test::TestMesh::cube_20x20x20}, prepared.print, prepared.model, config);
    const boost::filesystem::path path = Slic3r::Test::GCode::temporary_gcode_path();
    try {
        prepared.print.process();
        Orchestrator::instance().export_gcode(prepared.print, path.string(), nullptr);
        const std::string code = Slic3r::Test::GCode::read_text_file(path);
        CHECK(code.find("G1") != std::string::npos);
        CHECK(code.find(" E") != std::string::npos);
        std::istringstream stream(code);
        std::string line;
        while (std::getline(stream, line))
            if (line.find("G1 ") == 0 || line.find("G0 ") == 0) {
                const std::string command = line.substr(0, line.find(';'));
                CHECK(command.find("nan") == std::string::npos);
                CHECK(command.find("inf") == std::string::npos);
            }
    } catch (...) {
        Slic3r::Test::GCode::remove_output_pair(path);
        throw;
    }
    Slic3r::Test::GCode::remove_output_pair(path);
}

TEST_CASE("Lightning removes coincident junction vertices without dividing by zero", "[plugins][lightning][trees]")
{
    namespace lightning = slic3r_api::Infill::Lightning;
    namespace work = lightning::work;
    const lightning::NodePtr root = std::make_shared<lightning::Node>(work::Point(0, 0));
    const lightning::NodePtr child = std::make_shared<lightning::Node>(work::Point(0, 0));
    root->attach(child);
    child->add_child(work::Point(scale_i(5.), 0));
    for (coord_t overlap : {coord_t(0), scale_i(0.2), -scale_i(0.2)}) {
        const work::Polylines paths = root->paths(overlap);
        REQUIRE(paths.size() == 1);
        REQUIRE(paths.front().size() == 2);
        for (const work::Point &point : paths.front().points) {
            CHECK(std::abs(point.x()) <= scale_i(6.));
            CHECK(point.y() == 0);
        }
    }
}

TEST_CASE("Lightning refuses recipe-only selection without a configured region", "[plugins][lightning][setup]")
{
    namespace lightning = slic3r_api::Infill::Lightning;
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({{"fill_pattern", "rectilinear"}}));
    lightning::LightningPattern pattern(reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    const plugin_instance instance = pattern.c_instance();
    raw_infill_pattern_assignment assignment = {};
    raw_infill_pattern_object_init object = {};
    object.object = reinterpret_cast<const object_handle *>(&prepared.print.object(0));
    object.assignments = &assignment;
    object.assignment_count = 1;
    run_ctx_infill_pattern_init init = {};
    init.print = reinterpret_cast<const print_handle *>(&prepared.print);
    init.objects = &object;
    init.object_count = 1;
    std::string error;
    plugin_run_context context = {};
    context.step = INFILL_PATTERN;
    context.data = &init;
    context.host_context = reinterpret_cast<plugin_host_context *>(&error);
    context.report_error = [](plugin_host_context *host, const char *message) { *reinterpret_cast<std::string *>(host) = message; };
    instance.vt->setup(instance.ctx, &context, 1);
    CHECK(error.find("configured Lightning region") != std::string::npos);
}
