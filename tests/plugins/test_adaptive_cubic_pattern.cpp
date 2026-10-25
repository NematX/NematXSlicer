#include <catch2/catch.hpp>

#include <array>
#include <initializer_list>
#include <future>
#include <random>
#include <set>
#include <sstream>
#include <vector>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "test_data.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintRegion.hpp"
#include "libslic3r/Fill/FillAdaptive.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/ShortestPath.hpp"
#include "libslic3r/Tesselate.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Plugins/Infill/AdaptiveCubic/AdaptiveFill.hpp"
#include "libslic3r/Plugins/Infill/AdaptiveCubic/AdaptivePaths.hpp"
#include "libslic3r/Plugins/Infill/InfillPaths.hpp"
#include "libslic3r/Steps/StepGenerateInfill.hpp"
#include "libslic3r/Steps/StepGeneratePerimeter.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepPostSlicing.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"
#include "libslic3r/Steps/StepSurfaceGeneration.hpp"

/*
Adaptive Cubic pattern integration tests
========================================

The pattern prepares its cache through setup(), without a preparatory pipeline
step. Directed path parity, generation lifetime and source immutability are
checked independently of the sortable group's output order.
*/

using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

namespace {

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

DynamicPrintConfig adaptive_cubic_test_config()
{
    return perimeter_config({
        { "fill_pattern", "adaptivecubic" },
        { "fill_density", "20%" },
        { "top_solid_layers", "0" },
        { "bottom_solid_layers", "0" },
        { "solid_infill_every_layers", "0" },
        { "infill_every_layers", "1" }
    });
}

DynamicPrintConfig support_cubic_test_config()
{
    return perimeter_config({
        { "fill_pattern", "supportcubic" },
        { "fill_density", "20%" },
        { "top_solid_layers", "0" },
        { "bottom_solid_layers", "0" },
        { "solid_infill_every_layers", "0" },
        { "infill_every_layers", "1" }
    });
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
                        paths += entity->is_collection() ?
                            static_cast<const ExtrusionEntityCollection *>(entity)->entities().size() : 1;
                }
    return paths;
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

} // namespace

TEST_CASE("STEP_INFILL generates Adaptive Cubic extrusion", "[plugins][infill][adaptive-cubic]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    DynamicPrintConfig config;
    {
        ScopedActivePlugins active({ "adaptivecubic" });
        config = adaptive_cubic_test_config();
    }

    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, config);

    ScopedActivePlugins active({
        SIMPLE_PERIMETER_GENERATOR,
        INITIAL_TYPED_SURFACE_BUILDER,
        "infill.generator.default",
        "adaptivecubic"
    });
    run_until_infill(prepared);

    CHECK(count_infill_paths(prepared.print) > 1);
}

TEST_CASE("STEP_INFILL generates Support Cubic extrusion without pre-infill", "[plugins][infill][support-cubic]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    DynamicPrintConfig config;
    {
        ScopedActivePlugins active({ "supportcubic" });
        config = support_cubic_test_config();
    }

    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, config);

    ScopedActivePlugins active({
        SIMPLE_PERIMETER_GENERATOR,
        INITIAL_TYPED_SURFACE_BUILDER,
        "infill.generator.default",
        "supportcubic"
    });
    run_until_infill(prepared);

    CHECK(count_infill_paths(prepared.print) > 1);
}

namespace {
namespace adaptive = slic3r_api::Infill::AdaptiveCubic;
namespace work = slic3r_api::Infill::RectilinearPatternsPlugin::work;

void require_paths(const Polylines &expected, const work::Polylines &actual, bool compare_order = true)
{
    using Coordinates = std::vector<std::pair<coord_t, coord_t>>;
    std::vector<Coordinates> expected_paths, actual_paths;
    for (const Polyline &line : expected) {
        Coordinates &points = expected_paths.emplace_back();
        for (const Point &point : line.points)
            points.emplace_back(point.x(), point.y());
    }
    for (const work::Polyline &line : actual) {
        Coordinates &points = actual_paths.emplace_back();
        for (const work::Point &point : line.points)
            points.emplace_back(point.x(), point.y());
    }
    if (!compare_order) {
        std::sort(expected_paths.begin(), expected_paths.end());
        std::sort(actual_paths.begin(), actual_paths.end());
    }
    REQUIRE(expected_paths.size() == actual_paths.size());
    for (size_t path = 0; path < expected_paths.size(); ++path) {
        INFO("path " << path);
        REQUIRE(expected_paths[path].size() == actual_paths[path].size());
        for (size_t point = 0; point < expected_paths[path].size(); ++point) {
            INFO("point " << point);
            REQUIRE(expected_paths[path][point] == actual_paths[path][point]);
        }
    }
}

void collect_paths(const slic3r_api::ExtrusionEntity &entity, work::Polylines &paths)
{
    if (entity.is_leaf()) {
        if (entity.point_count() < 2)
            return;
        work::Polyline &path = paths.emplace_back();
        for (uint32_t i = 0; i < entity.point_count(); ++i)
            path.points.emplace_back(entity.point(i));
    } else {
        for (const slic3r_api::ExtrusionEntity child : entity.children())
            collect_paths(child, paths);
    }
}
}

TEST_CASE("Adaptive chaining matches native greedy and two exchanges", "[plugins][adaptive-cubic][adaptive-chain]")
{
    std::mt19937 random(7891);
    for (size_t count : {size_t(0), size_t(1), size_t(2), size_t(7), size_t(20)}) {
        for (int trial = 0; trial < 5; ++trial) {
            INFO("count " << count << " trial " << trial);
            Polylines core;
            work::Polylines plugin;
            for (size_t i = 0; i < count; ++i) {
                const Point a(coord_t(random() % 40000000), coord_t(random() % 40000000));
                const Point b(coord_t(random() % 40000000), coord_t(random() % 40000000));
                core.emplace_back(a, b);
                plugin.emplace_back(work::Point(a.x(), a.y()), work::Point(b.x(), b.y()));
            }
            require_paths(Slic3r::chain_polylines(std::move(core)), slic3r_api::Infill::Paths::chain_polylines(std::move(plugin), nullptr));
        }
    }
}

TEST_CASE("Adaptive triangulation preserves marked surface coordinates and holes", "[plugins][adaptive-cubic]")
{
    for (const ExPolygon &polygon : {rectangle_expolygon(-7., 2., 13., 17.), rectangle_with_hole_expolygon()}) {
        const slic3r_api::ExPolygon view(reinterpret_cast<const expolygon_handle *>(&polygon));
        const std::vector<Vec3d> expected = triangulate_expolygon_3d(polygon, 2.35);
        const std::vector<adaptive::Triangle> triangles = adaptive::triangulate_boundary(view, 2.35);
        REQUIRE(expected.size() == triangles.size() * 3);
        size_t i = 0;
        for (const adaptive::Triangle &triangle : triangles)
            for (const adaptive::Vec3 &point : {triangle.a, triangle.b, triangle.c}) {
                CHECK(point.x == expected[i].x());
                CHECK(point.y == expected[i].y());
                CHECK(point.z == expected[i].z());
                ++i;
            }
    }
}

TEST_CASE("Adaptive complete paths match the native filler", "[plugins][adaptive-cubic][adaptive-parity]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    indexed_triangle_set mesh = its_make_cube(40., 40., 20.);
    for (Vec3f &vertex : mesh.vertices) {
        vertex.x() -= 20.f;
        vertex.y() -= 20.f;
    }
    std::vector<adaptive::Triangle> triangles;
    for (const stl_triangle_vertex_indices &indices : mesh.indices) {
        const Vec3f &a = mesh.vertices[indices[0]], &b = mesh.vertices[indices[1]], &c = mesh.vertices[indices[2]];
        triangles.push_back({{a.x(), a.y(), a.z()}, {b.x(), b.y(), b.z()}, {c.x(), c.y(), c.z()}});
    }
    const Eigen::Matrix<double, 3, 3, Eigen::DontAlign> rotation = FillAdaptive::transform_to_octree().toRotationMatrix();
    std::vector<Vec3d> bridge_vertices;
    SECTION("Without additional bridges") {}
    SECTION("With projected bridge surfaces and holes") {
        for (const std::pair<ExPolygon, double> &bridge : {
            std::pair<ExPolygon, double>{rectangle_expolygon(-13., -8., 7., 9.), 7.4},
            std::pair<ExPolygon, double>{rectangle_with_hole_expolygon(), 14.2}}) {
            const std::vector<Vec3d> vertices = triangulate_expolygon_3d(bridge.first, bridge.second);
            bridge_vertices.insert(bridge_vertices.end(), vertices.begin(), vertices.end());
        }
        REQUIRE_FALSE(bridge_vertices.empty());
    }
    std::vector<adaptive::Triangle> bridge_triangles;
    for (size_t index = 0; index < bridge_vertices.size(); index += 3) {
        const Vec3d &a = bridge_vertices[index], &b = bridge_vertices[index + 1], &c = bridge_vertices[index + 2];
        bridge_triangles.push_back({{a.x(), a.y(), a.z()}, {b.x(), b.y(), b.z()}, {c.x(), c.y(), c.z()}});
    }
    for (Vec3d &vertex : bridge_vertices)
        vertex = rotation * vertex;
    its_transform(mesh, rotation, true);
    FillAdaptive::OctreePtr reference = FillAdaptive::build_octree(mesh, bridge_vertices, 5., false);
    adaptive::AdaptiveCache cache;
    cache.line_spacing = 5.;
    cache.octree = adaptive::Octree::build(triangles, bridge_triangles, 5., adaptive::CubicMode::Adaptive);
    REQUIRE(cache.octree != nullptr);
    PrintRegionConfig config;
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    const ExPolygons narrow_bridge = union_ex(ExPolygons{rectangle_expolygon(-10., -5., -4., 5.),
        rectangle_expolygon(-4., -0.1, 4., 0.1), rectangle_expolygon(4., -5., 10., 5.)});
    REQUIRE(narrow_bridge.size() == 1);
    REQUIRE(offset_ex(narrow_bridge, -scale_i(0.225)).size() == 2);
    for (double z : {0.2, 5.3, 13.7})
        for (const ExPolygon &polygon : {rectangle_expolygon(-19., -19., 19., 19.), rectangle_with_hole_expolygon(), narrow_bridge.front()})
            for (const InfillConnection connection : {InfillConnection::icConnected, InfillConnection::icNotConnected,
                                                       InfillConnection::icOuterShell, InfillConnection::icHoles})
              for (const std::pair<float, float> anchors : {std::pair<float, float>{0.f, 0.f}, {2.f, 5.f}, {1000.f, 1000.f}}) {
                INFO("z " << z << " holes " << polygon.holes.size() << " connection " << int(connection));
                INFO("anchors " << anchors.first << ',' << anchors.second);
                FillAdaptive::Filler filler;
                filler.adapt_fill_octree = reference.get();
                filler.z = z;
                filler.angle = 0.;
                filler.layer_id = 1;
                filler.set_bounding_box(get_extents(polygon));
                FillParams params;
                params.config = &config;
                params.density = 0.2f;
                params.anchor_length = anchors.first;
                params.anchor_length_max = anchors.second;
                params.connection = connection;
                params.fill_resolution = scale_i(0.0125);
                filler.init_spacing(0.45, params);
                Surface surface(stPosInternal | stDensSparse, polygon);
                const Polylines expected = filler.fill_surface(&surface, params);

                raw_infill_pattern_params raw = {};
                raw.density = params.density;
                raw.spacing = 0.45;
                raw.z = z;
                raw.anchor_length = params.anchor_length;
                raw.anchor_length_max = params.anchor_length_max;
                raw.connection = static_cast<c_infill_connection>(connection);
                raw.fill_resolution = params.fill_resolution;
                raw.flow_mult = 1.f;
                raw.flow.width = scale_i(0.45);
                raw.flow.height = scale_i(0.2);
                raw.flow.mm3_per_mm = 0.08;
                raw.extrusion_role = RAW_EXTRUSION_ROLE_INTERNAL_INFILL;
                slic3r_api::StoredExtrusionEntity output(handle);
                run_ctx_infill_pattern context = {};
                context.surface = reinterpret_cast<const surface_handle *>(&surface);
                context.params = &raw;
                context.output = output.mutable_handle();
                adaptive::fill_surface(context, cache, handle, nullptr);
                work::Polylines actual;
                collect_paths(output.readonly(), actual);
                require_paths(expected, actual, false);
            }
}

TEST_CASE("Support Cubic filters model normals but keeps bridge triangles", "[plugins][support-cubic][octree]")
{
    const auto snapshot = [](const adaptive::Octree &octree, const double z) {
        std::ostringstream out;
        for (const std::pair<c_point, c_point> &line : octree.generate_lines(z))
            out << line.first.x << ',' << line.first.y << ':' << line.second.x << ',' << line.second.y << ';';
        return out.str();
    };
    const adaptive::Triangle below_threshold{
        {-10., -10., 0.}, {10., -10., 0.}, {-10., 10., 20.1}
    };
    const adaptive::Triangle above_threshold{
        {-10., -10., 0.}, {10., -10., 0.}, {-10., 10., 19.9}
    };
    const auto reverse = [](const adaptive::Triangle &triangle) {
        return adaptive::Triangle{triangle.a, triangle.c, triangle.b};
    };

    const std::unique_ptr<adaptive::Octree> adaptive_front = adaptive::Octree::build(
        {below_threshold}, {}, 1., adaptive::CubicMode::Adaptive);
    const std::unique_ptr<adaptive::Octree> adaptive_back = adaptive::Octree::build(
        {reverse(below_threshold)}, {}, 1., adaptive::CubicMode::Adaptive);
    REQUIRE(adaptive_front != nullptr);
    REQUIRE(adaptive_back != nullptr);
    CHECK(snapshot(*adaptive_front, 5.) == snapshot(*adaptive_back, 5.));

    const std::unique_ptr<adaptive::Octree> rejected = adaptive::Octree::build(
        {below_threshold}, {}, 1., adaptive::CubicMode::Support);
    const std::unique_ptr<adaptive::Octree> reversed_rejected = adaptive::Octree::build(
        {reverse(below_threshold)}, {}, 1., adaptive::CubicMode::Support);
    const std::unique_ptr<adaptive::Octree> accepted = adaptive::Octree::build(
        {above_threshold}, {}, 1., adaptive::CubicMode::Support);
    const std::unique_ptr<adaptive::Octree> reversed_accepted = adaptive::Octree::build(
        {reverse(above_threshold)}, {}, 1., adaptive::CubicMode::Support);
    REQUIRE(rejected != nullptr);
    REQUIRE(reversed_rejected != nullptr);
    REQUIRE(accepted != nullptr);
    REQUIRE(reversed_accepted != nullptr);
    CHECK(snapshot(*rejected, 5.) == snapshot(*reversed_rejected, 5.));
    CHECK(snapshot(*accepted, 5.) != snapshot(*reversed_accepted, 5.));

    const std::unique_ptr<adaptive::Octree> with_bridge_triangle = adaptive::Octree::build(
        {reverse(below_threshold)}, {above_threshold}, 1., adaptive::CubicMode::Support);
    REQUIRE(with_bridge_triangle != nullptr);
    CHECK(snapshot(*with_bridge_triangle, 5.) != snapshot(*rejected, 5.));

    adaptive::Triangle rejected_far_away = reverse(above_threshold);
    rejected_far_away.a.x += 1000.;
    rejected_far_away.b.x += 1000.;
    rejected_far_away.c.x += 1000.;
    const std::unique_ptr<adaptive::Octree> full_mesh_bounds = adaptive::Octree::build(
        {reverse(below_threshold), rejected_far_away}, {above_threshold}, 1., adaptive::CubicMode::Support);
    REQUIRE(full_mesh_bounds != nullptr);
    CHECK(snapshot(*full_mesh_bounds, 5.) != snapshot(*with_bridge_triangle, 5.));
}

namespace {

std::string surfaces_snapshot(const PrintObject &object)
{
    std::ostringstream out;
    out.precision(17);
    for (const Layer &layer : object.layers())
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const Surface &surface : region.fill_surfaces()) {
                    out << surface.id() << ':' << int(surface.surface_type) << ':' << surface.bridge_angle << ':';
                    for (const Polygon &polygon : to_polygons(surface.expolygon)) {
                        for (const Point &point : polygon.points)
                            out << point.x() << ',' << point.y() << ';';
                        out << '|';
                    }
                }
    return out.str();
}

std::string lines_snapshot(const adaptive::Octree &tree, double z)
{
    std::ostringstream out;
    for (const std::pair<c_point, c_point> &line : tree.generate_lines(z))
        out << line.first.x << ',' << line.first.y << ':' << line.second.x << ',' << line.second.y << ';';
    return out.str();
}

std::string paths_snapshot(const work::Polylines &paths)
{
    std::ostringstream out;
    for (const work::Polyline &path : paths) {
        for (const work::Point &point : path.points)
            out << point.x() << ',' << point.y() << ';';
        out << '|';
    }
    return out.str();
}

struct CallbackProbe
{
    size_t checks = 0;
    size_t cancel_at = 0;
    std::string error;
};

plugin_run_context probe_context(CallbackProbe &probe, PluginStorage &storage)
{
    plugin_run_context context = {};
    context.step = INFILL_PATTERN;
    context.plugin_storage = reinterpret_cast<storage_handle *>(&storage);
    context.host_context = reinterpret_cast<plugin_host_context *>(&probe);
    context.is_cancelled = [](plugin_host_context *handle) {
        CallbackProbe &probe = *reinterpret_cast<CallbackProbe *>(handle);
        ++probe.checks;
        return int(probe.cancel_at > 0 && probe.checks >= probe.cancel_at);
    };
    context.report_error = [](plugin_host_context *handle, const char *message) {
        reinterpret_cast<CallbackProbe *>(handle)->error = message;
    };
    return context;
}

raw_infill_pattern_params test_recipe()
{
    raw_infill_pattern_params raw = {};
    raw.density = 0.2f;
    raw.flow.width = scale_i(0.45);
    raw.flow.height = scale_i(0.2);
    raw.flow.nozzle_diameter = scale_i(0.4);
    raw.flow.mm3_per_mm = 0.08;
    raw.flow_mult = 1.f;
    raw.spacing = 0.45;
    raw.z = 2.35;
    raw.fill_resolution = scale_i(0.0125);
    raw.anchor_length = 2.f;
    raw.anchor_length_max = 5.f;
    raw.connection = C_INFILL_CONNECTION_CONNECTED;
    raw.extrusion_role = RAW_EXTRUSION_ROLE_INTERNAL_INFILL;
    return raw;
}

void prepare_surfaces(PreparedPerimeterPrint &prepared,
                      const DynamicPrintConfig &config = perimeter_config({{"fill_pattern", "adaptivecubic"},
                          {"fill_density", "80%"}, {"infill_extrusion_width", "0.45"},
                          {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"}}))
{
    prepare_cube_print(prepared, config);
    ScopedActivePlugins active({SIMPLE_PERIMETER_GENERATOR, INITIAL_TYPED_SURFACE_BUILDER});
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepGeneratePerimeter::clear(prepared.print);
    Steps::StepGeneratePerimeter::run_step(orchestrator, prepared.print);
    Steps::StepSurfaceGeneration::clear(prepared.print);
    Steps::StepSurfaceGeneration::run_step(orchestrator, prepared.print);
}

// Both configured modes need real populated regions to exercise independent object caches.
void prepare_mixed_cubic_surfaces(PreparedPerimeterPrint &prepared);
}

TEST_CASE("Configured Cubic preparation reads only marked bridges", "[plugins][adaptive-cubic][support-cubic]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const adaptive::CubicMode mode : {adaptive::CubicMode::Adaptive, adaptive::CubicMode::Support}) {
    PreparedPerimeterPrint prepared;
    prepare_surfaces(prepared, perimeter_config({{"fill_pattern", mode == adaptive::CubicMode::Adaptive ? "adaptivecubic" : "supportcubic"},
        {"fill_density", "80%"}, {"infill_extrusion_width", "0.45"}, {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"}}));
    PrintObject &object = prepared.print.object(0);
    const slic3r_api::Print print(reinterpret_cast<const print_handle *>(&prepared.print));
    const slic3r_api::Object view(reinterpret_cast<const object_handle *>(&object));
    std::unique_ptr<adaptive::AdaptiveCache> initial = adaptive::prepare_cache(
        print, view, mode, nullptr);
    REQUIRE(initial->octree != nullptr);
    CHECK(initial->line_spacing == Approx(0.45 / (0.8 * 0.333333333f)));

    Layer &layer = object.layer(object.layer_count() / 2);
    LayerRegionIsland &region = layer.islands().front().regions_islands().front();
    region.set_fill_surfaces().surfaces.emplace_back(stPosInternal | stDensSolid,
                                                    rectangle_with_hole_expolygon());
    Surface &extra = region.set_fill_surfaces().surfaces.back();
    const std::unique_ptr<adaptive::AdaptiveCache> solid = adaptive::prepare_cache(
        print, view, mode, nullptr);
    CHECK(lines_snapshot(*initial->octree, 2.35) == lines_snapshot(*solid->octree, 2.35));
    extra.surface_type |= stModOverBridge;
    const std::unique_ptr<adaptive::AdaptiveCache> over = adaptive::prepare_cache(
        print, view, mode, nullptr);
    CHECK(lines_snapshot(*initial->octree, 2.35) == lines_snapshot(*over->octree, 2.35));
    extra.surface_type = stPosInternal | stDensSolid | stModBridge;
    const std::string snapshot = surfaces_snapshot(object);
    const std::unique_ptr<adaptive::AdaptiveCache> bridge = adaptive::prepare_cache(
        print, view, mode, nullptr);
    CHECK(surfaces_snapshot(object) == snapshot);
    const double bottom_z = unscaled(layer.scaled_bottom_z());
    CHECK(lines_snapshot(*initial->octree, bottom_z) != lines_snapshot(*bridge->octree, bottom_z));

    CallbackProbe probe;
    PluginStorage storage;
    plugin_run_context context = probe_context(probe, storage);
    probe.cancel_at = 8;
    CHECK_THROWS_AS(adaptive::prepare_cache(print, view, mode, &context),
                    slic3r_api::PluginCancelled);
    CHECK(surfaces_snapshot(object) == snapshot);
    }
}

TEST_CASE("Adaptive setup owns generation caches without a pre-infill step", "[plugins][adaptive-cubic]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_surfaces(prepared);
    PrintObject &object = prepared.print.object(0);
    const std::string snapshot = surfaces_snapshot(object);
    Plugin *plugin = Orchestrator::instance().get_plugin("adaptivecubic");
    REQUIRE(plugin != nullptr);
    PluginStorage storage;
    CallbackProbe probe;
    plugin_run_context context = probe_context(probe, storage);
    raw_infill_pattern_params raw = test_recipe();
    raw_infill_pattern_assignment assignment = {};
    assignment.params = &raw;
    raw_infill_pattern_object_init object_init = {};
    object_init.object = reinterpret_cast<const object_handle *>(&object);
    object_init.assignments = &assignment;
    object_init.assignment_count = 1;
    std::array<raw_infill_pattern_object_init, 2> objects{{object_init, object_init}};
    run_ctx_infill_pattern_init init = {};
    init.print = reinterpret_cast<const print_handle *>(&prepared.print);
    init.objects = objects.data();
    init.object_count = 1;
    context.data = &init;
    plugin->setup(context, 2);
    REQUIRE(probe.error.empty());
    const size_t preparation_checks = probe.checks;
    REQUIRE(preparation_checks > 10);
    probe.checks = 0;
    init.object_count = 2;
    plugin->setup(context, 2);
    CHECK(probe.checks == preparation_checks + 1);

    Surface surface(stPosInternal | stDensSparse, rectangle_expolygon(-9., -9., 9., 9.));
    run_ctx_infill_pattern run = {};
    run.object = object_init.object;
    run.params = &raw;
    run.surface = reinterpret_cast<const surface_handle *>(&surface);
    const auto render = [&]() {
        slic3r_api::StoredExtrusionEntity output(context.plugin_storage);
        run.output = output.mutable_handle();
        context.data = &run;
        plugin->setup_run(context);
        plugin->run(context);
        work::Polylines paths;
        collect_paths(output.readonly(), paths);
        return paths;
    };
    const work::Polylines first = render();
    REQUIRE_FALSE(first.empty());
    CHECK(paths_snapshot(first) == paths_snapshot(render()));
    CHECK(surfaces_snapshot(object) == snapshot);
    prepared.model.objects().front().volumes.front()->config.set_deserialize_strict("fill_density", "20%");
    prepared.print.apply(prepared.model, perimeter_config({{"fill_pattern", "adaptivecubic"}, {"fill_density", "80%"},
        {"infill_extrusion_width", "0.45"}, {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"}}));
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepLayerHeightGeneration::run_step(orchestrator, prepared.print);
    Steps::StepSlicing::run_step(orchestrator, prepared.print);
    Steps::StepPostSlicing::run_step(orchestrator, prepared.print);
    {
        ScopedActivePlugins active({SIMPLE_PERIMETER_GENERATOR, INITIAL_TYPED_SURFACE_BUILDER});
        Steps::StepGeneratePerimeter::clear(prepared.print);
        Steps::StepGeneratePerimeter::run_step(orchestrator, prepared.print);
        Steps::StepSurfaceGeneration::clear(prepared.print);
        Steps::StepSurfaceGeneration::run_step(orchestrator, prepared.print);
    }
    for (raw_infill_pattern_object_init &entry : objects)
        entry.object = reinterpret_cast<const object_handle *>(&prepared.print.object(0));
    run.object = objects.front().object;
    const std::string rebuilt_snapshot = surfaces_snapshot(prepared.print.object(0));
    context.data = &init;
    plugin->setup(context, 2);
    const work::Polylines changed = render();
    CHECK(paths_snapshot(first) != paths_snapshot(changed));
    CHECK(probe.error.empty());
    CHECK(surfaces_snapshot(prepared.print.object(0)) == rebuilt_snapshot);

    probe.checks = 0;
    probe.cancel_at = 8;
    context.data = &init;
    plugin->setup(context, 2);
    probe.cancel_at = 0;
    CHECK(render().empty());
    CHECK(probe.error.find("was not prepared") != std::string::npos);
    CHECK(surfaces_snapshot(prepared.print.object(0)) == rebuilt_snapshot);
}

TEST_CASE("Adaptive and Support keep separate setup caches for one object", "[plugins][support-cubic]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_mixed_cubic_surfaces(prepared);
    PrintObject &object = prepared.print.object(0);
    const std::string snapshot = surfaces_snapshot(object);
    Plugin *adaptive_plugin = Orchestrator::instance().get_plugin("adaptivecubic");
    Plugin *support_plugin = Orchestrator::instance().get_plugin("supportcubic");
    REQUIRE(adaptive_plugin != nullptr);
    REQUIRE(support_plugin != nullptr);

    PluginStorage storage;
    CallbackProbe probe;
    plugin_run_context context = probe_context(probe, storage);
    raw_infill_pattern_params raw = test_recipe();
    raw_infill_pattern_assignment assignment = {};
    assignment.params = &raw;
    raw_infill_pattern_object_init object_init = {};
    object_init.object = reinterpret_cast<const object_handle *>(&object);
    object_init.assignments = &assignment;
    object_init.assignment_count = 1;
    run_ctx_infill_pattern_init init = {};
    init.print = reinterpret_cast<const print_handle *>(&prepared.print);
    init.objects = &object_init;
    init.object_count = 1;

    const Surface surface(stPosInternal | stDensSparse, rectangle_expolygon(-9., -9., 9., 9.));
    const auto render = [&](Plugin &plugin) {
        slic3r_api::StoredExtrusionEntity output(context.plugin_storage);
        run_ctx_infill_pattern run = {};
        run.object = object_init.object;
        run.params = &raw;
        run.surface = reinterpret_cast<const surface_handle *>(&surface);
        run.output = output.mutable_handle();
        context.data = &run;
        plugin.setup_run(context);
        plugin.run(context);
        work::Polylines paths;
        collect_paths(output.readonly(), paths);
        return paths;
    };

    context.data = &init;
    adaptive_plugin->setup(context, 1);
    const work::Polylines adaptive_before = render(*adaptive_plugin);
    CHECK_FALSE(adaptive_before.empty());

    context.data = &init;
    support_plugin->setup(context, 1);
    CHECK_FALSE(render(*support_plugin).empty());
    CHECK(paths_snapshot(adaptive_before) == paths_snapshot(render(*adaptive_plugin)));
    CHECK(probe.error.empty());
    CHECK(surfaces_snapshot(object) == snapshot);
}

TEST_CASE("Adaptive marked bridges stay in slicing coordinates for offset objects", "[plugins][adaptive-cubic]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    TriangleMesh mesh = make_cube(40., 30., 20.);
    mesh.translate(17., -23., 0.);
    Slic3r::Test::init_print({mesh}, prepared.print, prepared.model,
        perimeter_config({{"fill_pattern", "adaptivecubic"}, {"fill_density", "80%"}, {"infill_extrusion_width", "0.45"},
            {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"}}));
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepLayerHeightGeneration::run_step(orchestrator, prepared.print);
    Steps::StepSlicing::run_step(orchestrator, prepared.print);
    Steps::StepPostSlicing::run_step(orchestrator, prepared.print);
    ScopedActivePlugins active({SIMPLE_PERIMETER_GENERATOR, INITIAL_TYPED_SURFACE_BUILDER});
    Steps::StepGeneratePerimeter::run_step(orchestrator, prepared.print);
    Steps::StepSurfaceGeneration::run_step(orchestrator, prepared.print);
    PrintObject &object = prepared.print.object(0);
    REQUIRE(object.center_offset() != Point(0, 0));
    Layer &layer = object.layer(object.layer_count() / 2);
    LayerRegionIsland &region = layer.islands().front().regions_islands().front();
    const ExPolygon bridge = rectangle_expolygon(-8., -5., 8., 5.);
    region.set_fill_surfaces().surfaces.emplace_back(stPosInternal | stDensSolid | stModBridge, bridge);
    const std::string snapshot = surfaces_snapshot(object);
    const std::unique_ptr<adaptive::AdaptiveCache> cache = adaptive::prepare_cache(
        slic3r_api::Print(reinterpret_cast<const print_handle *>(&prepared.print)),
        slic3r_api::Object(reinterpret_cast<const object_handle *>(&object)), adaptive::CubicMode::Adaptive, nullptr);

    indexed_triangle_set model_mesh = object.model_object()->raw_indexed_triangle_set();
    std::vector<adaptive::Triangle> triangles;
    for (const stl_triangle_vertex_indices &indices : model_mesh.indices) {
        adaptive::Triangle triangle;
        adaptive::Vec3 *vertices[] = {&triangle.a, &triangle.b, &triangle.c};
        for (int i = 0; i < 3; ++i) {
            const Vec3d point = object.trafo_centered() * model_mesh.vertices[indices[i]].cast<double>();
            *vertices[i] = {point.x(), point.y(), point.z()};
        }
        triangles.push_back(triangle);
    }
    const std::vector<Vec3d> native = triangulate_expolygon_3d(bridge, unscaled(layer.scaled_bottom_z()));
    std::vector<adaptive::Triangle> marked;
    for (size_t i = 0; i < native.size(); i += 3)
        marked.push_back({{native[i].x(), native[i].y(), native[i].z()},
                          {native[i + 1].x(), native[i + 1].y(), native[i + 1].z()},
                          {native[i + 2].x(), native[i + 2].y(), native[i + 2].z()}});
    const std::unique_ptr<adaptive::Octree> expected = adaptive::Octree::build(
        triangles, marked, cache->line_spacing, adaptive::CubicMode::Adaptive);
    REQUIRE(cache->octree != nullptr);
    REQUIRE(expected != nullptr);
    for (double z : {2.35, 8.73, 13.19})
        CHECK(lines_snapshot(*cache->octree, z) == lines_snapshot(*expected, z));
    CHECK(surfaces_snapshot(object) == snapshot);

    const auto read_tree = [&cache]() { return lines_snapshot(*cache->octree, 8.73); };
    std::future<std::string> first = std::async(std::launch::async, read_tree);
    std::future<std::string> second = std::async(std::launch::async, read_tree);
    CHECK(first.get() == second.get());
}

TEST_CASE("Support Cubic preserves oriented paths through model transforms", "[plugins][support-cubic][transforms]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    struct TransformCase
    {
        bool mirror_volume;
        bool mirror_instance;
        bool scale_volume;
        bool rotate_instance;
    };
    const TransformCase cases[] = {
        {true, false, false, false},
        {false, true, false, false},
        {true, true, false, false},
        {true, false, true, true}
    };
    const InfillConnection connections[] = {
        InfillConnection::icConnected,
        InfillConnection::icNotConnected,
        InfillConnection::icOuterShell,
        InfillConnection::icHoles
    };
    const std::pair<float, float> anchor_lengths[] = {
        {0.f, 0.f}, {2.f, 5.f}, {1000.f, 1000.f}
    };
    const ExPolygon boundaries[] = {
        rectangle_expolygon(-19., -19., 19., 19.),
        rectangle_with_hole_expolygon()
    };

    DynamicPrintConfig config;
    {
        ScopedActivePlugins active({"supportcubic"});
        config = support_cubic_test_config();
    }

    for (const TransformCase &test_case : cases) {
        CAPTURE(test_case.mirror_volume, test_case.mirror_instance,
                test_case.scale_volume, test_case.rotate_instance);
        PreparedPerimeterPrint prepared;
        TriangleMesh mesh = make_cube(40., 30., 20.);
        mesh.translate(17., -23., 0.);
        Slic3r::Test::init_print({mesh}, prepared.print, prepared.model, config);

        ModelObject &model_object = prepared.model.objects().front();
        ModelVolume &volume = *model_object.volumes.front();
        ModelInstance &instance = *model_object.instances.front();
        if (test_case.scale_volume)
            volume.set_scaling_factor(Vec3d(0.8, 1.2, 1.1));
        if (test_case.mirror_volume)
            volume.set_mirror(Axis::X, -1.);
        if (test_case.rotate_instance)
            instance.set_rotation(Axis::Z, Geometry::deg2rad(31.));
        if (test_case.mirror_instance)
            instance.set_mirror(Axis::Y, -1.);
        prepared.print.apply(prepared.model, config);

        Orchestrator &orchestrator = Orchestrator::instance();
        Steps::StepLayerHeightGeneration::run_step(orchestrator, prepared.print);
        Steps::StepSlicing::run_step(orchestrator, prepared.print);
        Steps::StepPostSlicing::run_step(orchestrator, prepared.print);
        {
            ScopedActivePlugins active({SIMPLE_PERIMETER_GENERATOR, INITIAL_TYPED_SURFACE_BUILDER});
            Steps::StepGeneratePerimeter::clear(prepared.print);
            Steps::StepGeneratePerimeter::run_step(orchestrator, prepared.print);
            Steps::StepSurfaceGeneration::clear(prepared.print);
            Steps::StepSurfaceGeneration::run_step(orchestrator, prepared.print);
        }

        PrintObject &object = prepared.print.object(0);
        REQUIRE(object.center_offset() != Point(0, 0));
        raw_infill_pattern_params raw = test_recipe();
        raw.density = 0.2f;
        raw.spacing = 0.45;
        raw.z = 3.2;
        raw.anchor_length = 0.f;
        raw.anchor_length_max = 0.f;
        raw.connection = C_INFILL_CONNECTION_NOT_CONNECTED;
        raw.flow.width = scale_i(0.45);
        raw.flow.height = scale_i(0.2);
        raw.flow.nozzle_diameter = scale_i(0.4);
        raw.flow.mm3_per_mm = 0.08;
        raw.extrusion_role = RAW_EXTRUSION_ROLE_INTERNAL_INFILL;
        const slic3r_api::Print api_print(reinterpret_cast<const print_handle *>(&prepared.print));
        const slic3r_api::Object api_object(reinterpret_cast<const object_handle *>(&object));
        const std::unique_ptr<adaptive::AdaptiveCache> support_cache = adaptive::prepare_cache(
            api_print, api_object, adaptive::CubicMode::Support, nullptr);
        REQUIRE(support_cache->octree != nullptr);

        indexed_triangle_set model_mesh = object.model_object()->raw_indexed_triangle_set();
        const Eigen::Matrix3d octree_rotation = FillAdaptive::transform_to_octree().toRotationMatrix();
        its_transform(model_mesh, octree_rotation * object.trafo_centered(), true);
        FillAdaptive::OctreePtr native_octree =
            FillAdaptive::build_octree(model_mesh, {}, support_cache->line_spacing, true);
        REQUIRE(native_octree != nullptr);

        const PrintRegionConfig &region_config = object.printing_region(0).config();
        PluginStorage storage;
        storage_handle *storage_handle_value = reinterpret_cast<storage_handle *>(&storage);
        for (const ExPolygon &boundary : boundaries) {
            CAPTURE(boundary.holes.size());
            for (double z : {3.2, 10.1, 15.3}) {
                for (const InfillConnection connection : connections) {
                    for (const std::pair<float, float> &anchors : anchor_lengths) {
                        CAPTURE(z, connection, anchors.first, anchors.second);
                        raw.z = z;
                        raw.connection = static_cast<c_infill_connection>(connection);
                        raw.anchor_length = anchors.first;
                        raw.anchor_length_max = anchors.second;

                        FillAdaptive::Filler native_filler;
                        native_filler.adapt_fill_octree = native_octree.get();
                        native_filler.z = z;
                        native_filler.angle = 0.;
                        native_filler.layer_id = 1;
                        native_filler.set_bounding_box(object.bounding_box());
                        FillParams params;
                        params.config = &region_config;
                        params.density = raw.density;
                        params.anchor_length = anchors.first;
                        params.anchor_length_max = anchors.second;
                        params.connection = connection;
                        params.fill_resolution = raw.fill_resolution;
                        native_filler.init_spacing(raw.spacing, params);
                        Surface surface(stPosInternal | stDensSparse, boundary);
                        const Polylines expected = native_filler.fill_surface(&surface, params);

                        slic3r_api::StoredExtrusionEntity output(storage_handle_value);
                        run_ctx_infill_pattern context = {};
                        context.surface = reinterpret_cast<const surface_handle *>(&surface);
                        context.params = &raw;
                        context.output = output.mutable_handle();
                        adaptive::fill_surface(context, *support_cache, storage_handle_value, nullptr);
                        work::Polylines actual;
                        collect_paths(output.readonly(), actual);
                        require_paths(expected, actual, false);
                    }
                }
            }
        }
    }
}

TEST_CASE("Cubic modes use configured widths and maximum nozzle", "[plugins][adaptive-cubic][support-cubic]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::pair<const char *, double> widths[] = {
        {"0", double(1.125f * 0.6f)}, {"0.5", 0.5}, {"120%", 0.72}
    };
    const adaptive::CubicMode modes[] = {adaptive::CubicMode::Adaptive, adaptive::CubicMode::Support};
    for (const adaptive::CubicMode mode : modes) {
        const char *pattern = mode == adaptive::CubicMode::Adaptive ? "adaptivecubic" : "supportcubic";
        for (const std::pair<const char *, double> &width : widths) {
            CAPTURE(pattern, width.first);
            DynamicPrintConfig config;
            {
                ScopedActivePlugins active({pattern});
                config = perimeter_config({{"fill_pattern", pattern}, {"fill_density", "20%"},
                    {"nozzle_diameter", "0.4,0.6"}, {"infill_extrusion_width", width.first},
                    {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"}});
            }
            PreparedPerimeterPrint prepared;
            prepare_surfaces(prepared, config);
            const std::unique_ptr<adaptive::AdaptiveCache> cache = adaptive::prepare_cache(
                slic3r_api::Print(reinterpret_cast<const print_handle *>(&prepared.print)),
                slic3r_api::Object(reinterpret_cast<const object_handle *>(&prepared.print.object(0))), mode, nullptr);
            CHECK(cache->line_spacing == Approx(width.second / (0.2 * 0.333333333f)));
        }
    }
}

TEST_CASE("Support Cubic spacing excludes Adaptive regions", "[plugins][support-cubic]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config;
    {
        ScopedActivePlugins active({"supportcubic"});
        config = perimeter_config({
            {"fill_pattern", "supportcubic"},
            {"fill_density", "20%"},
            {"nozzle_diameter", "0.4,0.6"},
            {"infill_extrusion_width", "0.5"},
            {"top_solid_layers", "0"},
            {"bottom_solid_layers", "0"}
        });
    }

    PreparedPerimeterPrint prepared;
    prepare_surfaces(prepared, config);
    const PrintObject &object = prepared.print.object(0);
    const slic3r_api::Print api_print(reinterpret_cast<const print_handle *>(&prepared.print));
    const slic3r_api::Object api_object(reinterpret_cast<const object_handle *>(&object));
    const std::unique_ptr<adaptive::AdaptiveCache> support = adaptive::prepare_cache(
        api_print, api_object, adaptive::CubicMode::Support, nullptr);
    CHECK_THROWS_AS(adaptive::prepare_cache(api_print, api_object, adaptive::CubicMode::Adaptive, nullptr), std::runtime_error);

    CHECK(support->mode == adaptive::CubicMode::Support);
    CHECK(support->line_spacing == Approx(7.5));
    REQUIRE(support->octree != nullptr);
}

TEST_CASE("Cubic refuses recipe-only selection without a configured region", "[plugins][adaptive-cubic][support-cubic]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_surfaces(prepared, perimeter_config({{"fill_pattern", "rectilinear"}, {"fill_density", "80%"}}));
    const PrintObject &object = prepared.print.object(0);
    const std::string snapshot = surfaces_snapshot(object);
    raw_infill_pattern_params raw = test_recipe();
    raw.density = 0.8f;
    Surface surface(stPosInternal | stDensSparse, rectangle_expolygon(-9., -9., 9., 9.));
    raw_infill_pattern_assignment assignment = {};
    assignment.params = &raw;
    assignment.surface_count = 1;
    raw_infill_pattern_object_init object_init = {};
    object_init.object = reinterpret_cast<const object_handle *>(&object);
    object_init.assignments = &assignment;
    object_init.assignment_count = 1;
    run_ctx_infill_pattern_init init = {};
    init.print = reinterpret_cast<const print_handle *>(&prepared.print);
    init.objects = &object_init;
    init.object_count = 1;
    for (const char *id : {"adaptivecubic", "supportcubic"}) {
        INFO(id);
        PluginStorage storage;
        CallbackProbe probe;
        plugin_run_context context = probe_context(probe, storage);
        Plugin *plugin = Orchestrator::instance().get_plugin(id);
        REQUIRE(plugin != nullptr);
        context.data = &init;
        plugin->setup(context, 1);
        CHECK(probe.error.find("requires a configured region") != std::string::npos);
        probe.error.clear();
        slic3r_api::StoredExtrusionEntity output(context.plugin_storage);
        run_ctx_infill_pattern run = {};
        run.object = object_init.object;
        run.surface = reinterpret_cast<const surface_handle *>(&surface);
        run.params = &raw;
        run.output = output.mutable_handle();
        context.data = &run;
        plugin->run(context);
        CHECK(output.empty());
        CHECK(probe.error.find("was not prepared") != std::string::npos);
        CHECK(surfaces_snapshot(object) == snapshot);
    }
}

namespace {
void prepare_mixed_cubic_surfaces(PreparedPerimeterPrint &prepared)
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config;
    {
        ScopedActivePlugins active({"adaptivecubic", "supportcubic"});
        config = perimeter_config({
            {"fill_pattern", "adaptivecubic"},
            {"fill_density", "20%"},
            {"perimeters", "0"},
            {"top_solid_layers", "0"},
            {"bottom_solid_layers", "0"}
        });
    }

    struct RegionSettings
    {
        const char *pattern;
        const char *density;
        const char *width;
        double size_x;
        double center_x;
    };
    const RegionSettings settings[] = {
        {"adaptivecubic", "20%", "0.3", 8., -12.},
        {"adaptivecubic", "40%", "0.5", 16., 0.},
        {"supportcubic", "50%", "0.9", 8., 12.}
    };
    ModelObject *model_object = prepared.model.add_object();
    model_object->name = "mixed_cubic_patterns";
    for (const RegionSettings &setting : settings) {
        TriangleMesh mesh = make_cube(setting.size_x, 20., 10.);
        mesh.translate(setting.center_x, 0., 0.);
        ModelVolume *volume = model_object->add_volume(std::move(mesh));
        volume->config.set_deserialize_strict("fill_pattern", setting.pattern);
        volume->config.set_deserialize_strict("fill_density", setting.density);
        volume->config.set_deserialize_strict("infill_extrusion_width", setting.width);
    }
    model_object->add_instance();
    prepared.model.center_instances_around_point({100., 100.});
    for (ModelObject &current : prepared.model.objects()) {
        current.ensure_on_bed();
        prepared.print.auto_assign_extruders(&current);
    }
    prepared.print.apply(prepared.model, config);
    prepared.print.validate();

    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepLayerHeightGeneration::run_step(orchestrator, prepared.print);
    Steps::StepSlicing::run_step(orchestrator, prepared.print);
    Steps::StepPostSlicing::run_step(orchestrator, prepared.print);
    {
        ScopedActivePlugins active({SIMPLE_PERIMETER_GENERATOR, INITIAL_TYPED_SURFACE_BUILDER});
        Steps::StepGeneratePerimeter::clear(prepared.print);
        Steps::StepGeneratePerimeter::run_step(orchestrator, prepared.print);
        Steps::StepSurfaceGeneration::clear(prepared.print);
        Steps::StepSurfaceGeneration::run_step(orchestrator, prepared.print);
    }
}
}

TEST_CASE("Adaptive and Support spacing remain separate in mixed regions", "[plugins][support-cubic][multi-region]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_mixed_cubic_surfaces(prepared);

    const PrintObject &object = prepared.print.object(0);
    REQUIRE(object.num_printing_regions() >= 3);
    std::set<const PrintRegion *> populated_regions;
    size_t adaptive_region_count = 0;
    size_t support_region_count = 0;
    for (const Layer &layer : object.layers())
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region_island : island.regions_islands()) {
                if (region_island.fill_surfaces().empty())
                    continue;
                for (const LayerRegion *layer_region : region_island.regions()) {
                    const PrintRegion &region = layer_region->region();
                    if (!populated_regions.insert(&region).second)
                        continue;
                    const ConfigOption *pattern = region.config().option("fill_pattern");
                    REQUIRE(pattern != nullptr);
                    std::string id = pattern->serialize();
                    if (!id.empty() && id.front() == '!')
                        id.erase(id.begin());
                    adaptive_region_count += id == "adaptivecubic";
                    support_region_count += id == "supportcubic";
                }
            }
    CHECK(adaptive_region_count == 2);
    CHECK(support_region_count == 1);

    const slic3r_api::Object view(reinterpret_cast<const object_handle *>(&object));
    const std::unique_ptr<adaptive::AdaptiveCache> adaptive_cache = adaptive::prepare_cache(
        slic3r_api::Print(reinterpret_cast<const print_handle *>(&prepared.print)),
        view, adaptive::CubicMode::Adaptive, nullptr);
    const std::unique_ptr<adaptive::AdaptiveCache> support_cache = adaptive::prepare_cache(
        slic3r_api::Print(reinterpret_cast<const print_handle *>(&prepared.print)),
        view, adaptive::CubicMode::Support, nullptr);

    CHECK(adaptive_cache->line_spacing == Approx(4.));
    CHECK(support_cache->line_spacing == Approx(5.4));
}
