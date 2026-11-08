#include <catch2/catch.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Fill/FillBase.hpp"
#include "libslic3r/Plugins/Infill/SimpleInfillPatterns.hpp"
#include "libslic3r/Surface.hpp"

/*
Line pattern plugin tests
=========================

The plugin is a port of FillLine. The parity test fills the same surfaces with
the native filler and with the plugin and compares the directed paths, without
requiring the output order of the sortable group. Its plain square is 16 mm wide:
large enough for several joined lines at every density, small enough to keep
the full combination matrix cheap. The "not connected" mode is an intentional
difference: the legacy filler emitted nothing there.
*/

using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

namespace {

using DirectedPath = std::vector<std::pair<coord_t, coord_t>>;
using DirectedPaths = std::vector<DirectedPath>;

DirectedPaths core_paths(const Polylines &polylines)
{
    DirectedPaths out;
    for (const Polyline &polyline : polylines) {
        DirectedPath path;
        for (const Point &point : polyline.points)
            path.emplace_back(point.x(), point.y());
        out.push_back(std::move(path));
    }
    std::sort(out.begin(), out.end());
    return out;
}

void collect_api_paths(const slic3r_api::ExtrusionEntity &entity, DirectedPaths &out)
{
    if (entity.is_leaf()) {
        DirectedPath path;
        for (const c_point point : entity.points())
            path.emplace_back(point.x, point.y);
        out.push_back(std::move(path));
        return;
    }
    for (const slic3r_api::ExtrusionEntity child : entity.children())
        collect_api_paths(child, out);
}

// Readable dump of directed paths for failure messages.
std::string describe(const DirectedPaths &paths)
{
    std::string out = std::to_string(paths.size()) + " paths:";
    for (const DirectedPath &path : paths) {
        out += " [";
        for (const std::pair<coord_t, coord_t> &point : path)
            out += std::to_string(point.first) + "," + std::to_string(point.second) + " ";
        out += "]";
    }
    return out;
}

struct LineCase
{
    ExPolygon surface;
    float density;
    bool dont_adjust;
    float angle;
    size_t layer_id;
    InfillConnection connection;
    // Alternate the direction on odd layers.
    bool angle_cross = true;
    // Bridge direction, or a negative value for none.
    double bridge_angle = -1.;
};

raw_infill_pattern_params make_raw(const LineCase &line_case, const BoundingBox &bbox)
{
    raw_infill_pattern_params raw = {};
    raw.density = line_case.density;
    raw.spacing = 0.45;
    raw.angle = line_case.angle;
    raw.bridge_angle = line_case.bridge_angle;
    raw.layer_id = uint32_t(line_case.layer_id);
    raw.connection = static_cast<c_infill_connection>(line_case.connection);
    raw.infill_flags = uint16_t((line_case.angle_cross ? C_INFILL_FLAG_CAN_ANGLE_CROSS : 0) |
                                (line_case.dont_adjust ? C_INFILL_FLAG_DONT_ADJUST : 0));
    raw.fill_resolution = scale_i(0.0125);
    raw.flow_mult = 1.f;
    raw.flow.width = scale_i(0.45);
    raw.flow.height = scale_i(0.2);
    raw.flow.mm3_per_mm = 0.08;
    raw.extrusion_role = RAW_EXTRUSION_ROLE_INTERNAL_INFILL;
    raw.object_bounding_box.min = { bbox.min.x(), bbox.min.y() };
    raw.object_bounding_box.max = { bbox.max.x(), bbox.max.y() };
    return raw;
}

Polylines run_core(const LineCase &line_case, const BoundingBox &bbox)
{
    std::unique_ptr<Fill> filler(Fill::new_from_type(ipLine));
    filler->set_bounding_box(bbox);
    filler->layer_id = line_case.layer_id;
    filler->angle = line_case.angle;
    filler->can_angle_cross = line_case.angle_cross;
    static const PrintRegionConfig config;
    FillParams params;
    params.config = &config;
    params.density = line_case.density;
    params.dont_adjust = line_case.dont_adjust;
    params.connection = line_case.connection;
    params.fill_resolution = scale_i(0.0125);
    filler->init_spacing(0.45, params);
    Surface surface(stPosInternal | stDensSparse, line_case.surface);
    surface.bridge_angle = line_case.bridge_angle;
    return filler->fill_surface(&surface, params);
}

DirectedPaths run_plugin(const LineCase &line_case, const BoundingBox &bbox)
{
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    const raw_infill_pattern_params raw = make_raw(line_case, bbox);
    Surface surface(stPosInternal | stDensSparse, line_case.surface);
    slic3r_api::StoredExtrusionEntity output(handle);
    run_ctx_infill_pattern context = {};
    context.surface = reinterpret_cast<const surface_handle *>(&surface);
    context.params = &raw;
    context.output = output.mutable_handle();
    slic3r_api::Infill::fill_line_surface(context, handle, nullptr);
    DirectedPaths paths;
    collect_api_paths(output.readonly(), paths);
    std::sort(paths.begin(), paths.end());
    return paths;
}

std::vector<ExPolygon> parity_surfaces()
{
    const ExPolygons narrow_bridge = union_ex(ExPolygons{ rectangle_expolygon(-10., -5., -4., 5.),
        rectangle_expolygon(-4., -0.1, 4., 0.1), rectangle_expolygon(4., -5., 10., 5.) });
    REQUIRE(narrow_bridge.size() == 1);
    const ExPolygon lshape(Polygon({ Point(0, 0), Point(scale_i(30.), 0), Point(scale_i(30.), scale_i(10.)),
                                     Point(scale_i(10.), scale_i(10.)), Point(scale_i(10.), scale_i(25.)),
                                     Point(0, scale_i(25.)) }));
    return { rectangle_expolygon(-8., -8., 8., 8.), rectangle_with_hole_expolygon(), lshape,
             narrow_bridge.front() };
}

} // namespace

TEST_CASE("Line plugin matches the native line filler", "[plugins][infill][line]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::vector<ExPolygon> surfaces = parity_surfaces();
    for (const ExPolygon &surface : surfaces)
        for (const float density : { 0.15f, 0.5f, 1.f })
            for (const bool dont_adjust : { true, false })
                for (const float angle : { 0.f, float(M_PI / 6.), float(-M_PI / 4.) })
                    for (const size_t layer_id : { size_t(2), size_t(3) })
                        for (const InfillConnection connection : { InfillConnection::icConnected,
                                                                   InfillConnection::icOuterShell }) {
                            INFO("holes " << surface.holes.size() << " density " << density << " dont_adjust "
                                          << dont_adjust << " angle " << angle << " layer " << layer_id
                                          << " connection " << int(connection));
                            const LineCase line_case{ surface, density, dont_adjust, angle, layer_id, connection };
                            const BoundingBox bbox = get_extents(surface);
                            const DirectedPaths expected = core_paths(run_core(line_case, bbox));
                            REQUIRE_FALSE(expected.empty());
                            // The plain square keeps several lines at every density, and the
                            // connection modes join some of them.
                            if (&surface == &surfaces.front()) {
                                size_t points = 0;
                                for (const DirectedPath &path : expected)
                                    points += path.size();
                                CHECK(points >= 8);
                                CHECK(std::any_of(expected.begin(), expected.end(),
                                                  [](const DirectedPath &path) { return path.size() > 2; }));
                            }
                            const DirectedPaths actual = run_plugin(line_case, bbox);
                            INFO("expected " << describe(expected));
                            INFO("actual " << describe(actual));
                            CHECK(actual == expected);
                        }
}

TEST_CASE("Line plugin emits unjoined lines when not connected", "[plugins][infill][line]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ExPolygon surface = rectangle_expolygon(-19., -19., 19., 19.);
    const BoundingBox bbox = get_extents(surface);
    LineCase not_connected{ surface, 0.3f, true, 0.f, 2, InfillConnection::icNotConnected };
    LineCase connected = not_connected;
    connected.connection = InfillConnection::icConnected;

    // The legacy filler drops every line in this mode.
    CHECK(run_core(not_connected, bbox).empty());

    const DirectedPaths lines = run_plugin(not_connected, bbox);
    const DirectedPaths joined = run_plugin(connected, bbox);
    REQUIRE(lines.size() > 1);
    CHECK(lines.size() > joined.size());
    size_t line_points = 0;
    for (const DirectedPath &path : lines) {
        // Each line is one segment: no connector was added.
        CHECK(path.size() == 2);
        line_points += path.size();
    }
    size_t joined_points = 0;
    for (const DirectedPath &path : joined)
        joined_points += path.size();
    // Joining only concatenates the same lines.
    CHECK(line_points == joined_points);
}

TEST_CASE("Line plugin matches the native filler without alternation and on bridges", "[plugins][infill][line]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const ExPolygon &surface : parity_surfaces())
        for (const size_t layer_id : { size_t(2), size_t(3) })
            for (const bool angle_cross : { false, true })
                for (const double bridge_angle : { -1., 0.3 }) {
                    if (angle_cross && bridge_angle < 0.)
                        continue; // covered by the main parity test
                    INFO("holes " << surface.holes.size() << " layer " << layer_id << " cross " << angle_cross
                                  << " bridge " << bridge_angle);
                    LineCase line_case{ surface, 0.5f, true, float(M_PI / 6.), layer_id, InfillConnection::icConnected };
                    line_case.angle_cross = angle_cross;
                    line_case.bridge_angle = bridge_angle;
                    const BoundingBox bbox = get_extents(surface);
                    const DirectedPaths expected = core_paths(run_core(line_case, bbox));
                    REQUIRE_FALSE(expected.empty());
                    CHECK(run_plugin(line_case, bbox) == expected);
                }
}

TEST_CASE("Line plugin discards ready extrusion when publication is cancelled", "[plugins][infill][line][cancellation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const LineCase line_case{rectangle_expolygon(-19., -19., 19., 19.), 0.3f, true,
                             float(M_PI / 6.), 2, InfillConnection::icConnected};
    const BoundingBox bbox = get_extents(line_case.surface);
    const DirectedPaths expected = run_plugin(line_case, bbox);
    REQUIRE_FALSE(expected.empty());
    REQUIRE(expected.size() < 1024);

    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    const raw_infill_pattern_params raw = make_raw(line_case, bbox);
    Surface surface(stPosInternal | stDensSparse, line_case.surface);
    slic3r_api::StoredExtrusionEntity output(handle);
    const size_t initial_allocations = storage.size();
    run_ctx_infill_pattern context = {};
    context.surface = reinterpret_cast<const surface_handle *>(&surface);
    context.params = &raw;
    context.output = output.mutable_handle();
    struct Probe {
        PluginStorage *storage;
        const extrusion_entity_handle *output;
        size_t checks = 0;
        bool ready_group_seen = false;
    } probe{&storage, output.handle()};
    plugin_run_context run_ctx = {};
    run_ctx.host_context = reinterpret_cast<plugin_host_context *>(&probe);
    run_ctx.is_cancelled = [](plugin_host_context *host) {
        Probe &probe = *reinterpret_cast<Probe *>(host);
        ++probe.checks;
        // A nonempty temporary group appears only after paths have been built for publication.
        // Inspect a locked pointer snapshot; this single-threaded probe keeps the temporary entities alive.
        for (const ExtrusionEntity *owned : probe.storage->stored_extrusions()) {
            const slic3r_api::ExtrusionEntity entity(reinterpret_cast<const extrusion_entity_handle *>(owned));
            if (entity.handle() != probe.output && !entity.is_leaf() && !entity.empty()) {
                probe.ready_group_seen = true;
                return 1;
            }
        }
        return 0;
    };

    CHECK_THROWS_AS(slic3r_api::Infill::fill_line_surface(context, handle, &run_ctx), slic3r_api::PluginCancelled);
    CHECK(probe.ready_group_seen);
    CHECK(probe.checks > 1);
    CHECK(output.empty());
    CHECK(storage.size() == initial_allocations);
}
