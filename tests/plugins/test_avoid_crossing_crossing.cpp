#include <catch2/catch.hpp>

#include "plugin_test_helpers.hpp"
#include "test_data.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/AuxiliaryLayerHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/GCode/AvoidCrossingPerimeters.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/AvoidCrossing/PerimeterCrossing.hpp"
#include "libslic3r/Plugins/Common/ClipperHelpers.hpp"

/*
These tests compare PerimeterCrossing path-query booleans against the core class
on sliced square and circular rings, regional tool splits and excluded tops.
Deterministic lattice paths, source edges and multi-point excursions exercise
both results with two tools and two nozzle radii. Base and offset versions are
checked: object offsets are inset, but support offsets are expanded.

Support is attached manually as a public auxiliary layer using a many-sided
subject. For the core comparison, already_printed_object_layers contains the
support layer itself. Agreement therefore checks this preparation, not which
already printed object layers a support travel ought to consider. Matching the
core is not independent proof of its geometric correctness.

    test case: slice rings and tool regions, then attach a support subject
    |-- compare: prepare both implementations and compare deterministic paths
    |   |-- repeated prepare: exercise reuse with the same layer, tool and radius
    |   `-- support hole path: reach the large-hole box rejection
    `-- retained inputs: switch layer keys and switch back with fixed tool/radius

Print, input and storage stay alive for comparisons. Scene counters check tops,
support, tool-clip branches and both booleans. A focused path verifies that a
simplified hole still has more than ten points and reaches its rejecting box
test; a per-hole cancellation call observes that the hole loop was entered.
*/
namespace {
using namespace Slic3r;
namespace port = slic3r_api::LayerExtrusionEdit::AvoidCrossing;
namespace work = slic3r_api::PluginCommon::work;
namespace pw = slic3r_api::PluginCommon::polygon_work;
namespace clipper = slic3r_api::PluginCommon::clipper;

struct Coverage {
    size_t tops = 0, yes = 0, no = 0, support = 0;
    size_t same = 0, diff = 0, fill = 0, long_hole = 0;
};

// Compare both interior choices over deterministic paths and repeated cache preparations.
size_t compare(const Print &print, const Layer &layer, port::LayerIdentity identity, Coverage &coverage);
// Build a shallow ring with a separate island, leaving an open square hole.
TriangleMesh ring();
// Build a closed 24-sided annulus whose hole survives nozzle-radius simplification.
TriangleMesh circular_ring();

TEST_CASE("Plugin crossing queries match sliced core interiors", "[plugins][avoid-crossing-crossing]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "2"}, {"fill_density", "20%"}, {"top_solid_layers", "1"},
        {"bottom_solid_layers", "1"}, {"skirts", "0"}, {"brim_width", "0"},
        {"nozzle_diameter", "0.4,0.6"}, {"min_layer_height", "0.05,0.05"},
        {"max_layer_height", "0.3,0.3"}, {"gap_fill_enabled", "0"}, {"thin_walls", "0"},
        {"thin_perimeters", "0"}, {"avoid_crossing_top", "1"}});
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(ring(), ModelVolumeType::MODEL_PART, false);
    TriangleMesh round_island = circular_ring();
    round_island.translate(40., 0., 0.);
    object->add_volume(round_island, ModelVolumeType::MODEL_PART, false);
    ModelVolume *modifier = object->add_volume(make_cube(10., 20., 0.6), ModelVolumeType::PARAMETER_MODIFIER, false);
    DynamicPrintConfig regional;
    regional.set_deserialize_strict({{"infill_extruder", "2"}, {"solid_infill_extruder", "2"}});
    modifier->config.assign_config(regional);
    object->add_instance();
    object->ensure_on_bed();
    Print print;
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    REQUIRE(print.object(0).layers().size() >= 3);
    REQUIRE(print.object(0).layers().front().regions().size() > 1);
    size_t queries = 0;
    Coverage coverage;
    uint32_t index = 0;
    for (const Layer &layer : print.object(0).layers())
        queries += compare(print, layer, {0, index++, false}, coverage);

    PluginStorage storage;
    const slic3r_api::Print view(reinterpret_cast<const print_handle *>(&print));
    // Use a many-sided support subject so nozzle-radius simplification still leaves a hole large enough for box rejection.
    ExPolygons subject;
    for (const ExPolygon &polygon : print.object(0).layers().front().lslices())
        if (!polygon.holes.empty() && polygon.holes.front().size() > 10)
            subject.push_back(polygon);
    REQUIRE(subject.size() == 1);
    const coord_t z = view.object(0).layer(0).print_z();
    const slic3r_api::AuxiliaryLayerBuildResult built = slic3r_api::build_auxiliary_layer_regions_from_subject(
        reinterpret_cast<storage_handle *>(&storage), view, view.object(0),
        slic3r_api::ExPolygonCollection(reinterpret_cast<const expolygon_collection_handle *>(&subject)),
        scale_i(0.2), z, z - scale_i(0.1));
    REQUIRE(built.created);
    built.layer.mutable_properties().get_or_add(slic3r_api::LayerSupportProperty::key);
    const Layer &support = *reinterpret_cast<const Layer *>(built.layer.handle());
    queries += compare(print, support, {0, 0, true}, coverage);

    // Keep both different inputs alive while only the layer key changes, then switch back.
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    const port::Input normal_input = port::build_input(handle, view, {0, 0, false});
    const port::Input support_input = port::build_input(handle, view, {0, 0, true});
    REQUIRE(normal_input.target->lslices.size() != support_input.target->lslices.size());
    port::PerimeterCrossing cached;
    AvoidCrossingPerimeters oracle;
    std::vector<bool> first_results;
    std::vector<bool> second_results;
    for (int pass = 0; pass < 3; ++pass) {
        const Layer &layer = pass == 1 ? support : print.object(0).layers().front();
        const port::Input &input = pass == 1 ? support_input : normal_input;
        cached.prepare(handle, *input.target, 0, scale_i(0.2));
        oracle.prepare_crossing_test({layer, {&layer}, 0, 0, scale_i(0.2), {}});
        const BoundingBox box = get_extents(print.object(0).layers().front().lslices());
        for (int y = 0; y < 20; ++y) {
            Polyline path;
            path.points = {Point(box.min.x() - scale_i(1.), box.min.y() + scale_i(y)),
                           Point(box.max.x() + scale_i(1.), box.min.y() + scale_i(y))};
            work::Polyline converted;
            for (const Point &point : path.points)
                converted.points.emplace_back(point.x(), point.y());
            for (bool offset : {false, true}) {
                const bool result = cached.can_cross_perimeter(converted, offset);
                CHECK(result == oracle.can_cross_perimeter(path, offset));
                if (pass == 0) first_results.push_back(result);
                if (pass == 1) second_results.push_back(result);
                if (pass == 2) CHECK(result == first_results[size_t(y) * 2 + size_t(offset)]);
                ++queries;
            }
        }
    }
    CHECK(first_results != second_results);
    INFO("Compared crossing booleans: " << queries);
    CHECK(queries > 10000);
    CHECK(coverage.tops > 0);
    CHECK(coverage.yes > 0);
    CHECK(coverage.no > 0);
    CHECK(coverage.support > 0);
    CHECK(coverage.same > 0);
    CHECK(coverage.diff > 0);
    CHECK(coverage.fill > 0);
    CHECK(coverage.long_hole > 0);
}

size_t compare(const Print &print, const Layer &layer, port::LayerIdentity identity, Coverage &coverage)
{
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    const slic3r_api::Print view(reinterpret_cast<const print_handle *>(&print));
    const port::Input input = port::build_input(handle, view, identity);
    for (const port::RegionInput &region : input.target->regions)
        if (!region.top_surfaces.empty()) ++coverage.tops;
    port::PerimeterCrossing actual;
    AvoidCrossingPerimeters expected;
    const std::vector<const Layer *> printed{&layer};
    std::vector<Polyline> paths;
    const BoundingBox box = get_extents(layer.lslices());
    // Lattice pairs exercise interior, exterior, holes and paths crossing whole islands.
    for (int x = -2; x <= 12; ++x)
        for (int y = -2; y <= 12; ++y) {
            const Point a(box.min.x() + (box.max.x() - box.min.x()) * x / 10,
                          box.min.y() + (box.max.y() - box.min.y()) * y / 10);
            for (int d = 0; d < 4; ++d) {
                const Point b(box.min.x() + (box.max.x() - box.min.x()) * ((x * 7 + d * 3 + 19) % 17 - 3) / 10,
                              box.min.y() + (box.max.y() - box.min.y()) * ((y * 3 + d * 7 + 19) % 17 - 3) / 10);
                Polyline path;
                path.points = {a, b};
                paths.push_back(path);
                path.points.insert(path.points.begin() + 1, Point(box.min.x() - scale_i(1.), box.max.y() + scale_i(1.)));
                paths.push_back(path);
            }
        }
    // Exact source edges and vertices exercise inclusive containment and edge contact.
    for (const ExPolygon &polygon : layer.lslices()) {
        Polygons rings{polygon.contour};
        rings.insert(rings.end(), polygon.holes.begin(), polygon.holes.end());
        for (const Polygon &ring : rings)
            for (size_t i = 0; i < ring.size(); ++i) {
                Polyline path;
                path.points = {ring.points[i], ring.points[(i + 1) % ring.size()]};
                paths.push_back(path);
            }
    }
    size_t queries = 0;
    for (extruder_index_t tool : {extruder_index_t(0), extruder_index_t(1)})
        for (coord_t radius : {scale_i(0.2), scale_i(0.3)}) {
            if (!input.target->support)
                for (const port::RegionInput &region : input.target->regions) {
                    if (region.perimeter_extruder == region.infill_extruder &&
                        region.infill_extruder == region.solid_infill_extruder) {
                        if (region.perimeter_extruder == tool + 1) ++coverage.same;
                    } else if (region.perimeter_extruder == tool + 1) ++coverage.diff;
                    else ++coverage.fill;
                }
            actual.prepare(handle, *input.target, tool, radius);
            expected.prepare_crossing_test({layer, printed, 0, tool, radius, {}});
            // Repeat the same preparation to exercise the unchanged-key cache path before querying.
            actual.prepare(handle, *input.target, tool, radius);
            for (const Polyline &path : paths) {
                work::Polyline converted;
                for (const Point &point : path.points)
                    converted.points.emplace_back(point.x(), point.y());
                for (bool offset : {false, true}) {
                    CAPTURE(identity.layer, identity.auxiliary, tool, radius, offset, queries);
                    const bool result = actual.can_cross_perimeter(converted, offset);
                    CHECK(result == expected.can_cross_perimeter(path, offset));
                    if (result) ++coverage.yes;
                    else ++coverage.no;
                    if (input.target->support) ++coverage.support;
                    ++queries;
                }
            }
        }
    // A short path beside a large circular hole reaches its box rejection without
    // intersecting the outer contour. Cancellation proves the hole loop was reached.
    if (input.target->support) {
        const coord_t radius = scale_i(0.2);
        REQUIRE(input.target->lslices.size() == 1);
        const pw::ExPolygons simplified = clipper::simplify(handle, input.target->lslices.front(), radius);
        REQUIRE(simplified.size() == 1);
        REQUIRE(simplified.front().holes.size() == 1);
        const pw::Polygon &hole = simplified.front().holes.front();
        REQUIRE(hole.size() > 10);
        const work::BoundingBox box(hole.points);
        const work::Point center(box.min.x() - scale_i(1.), (box.min.y() + box.max.y()) / 2);
        work::Polyline path;
        path.points = {center, center + work::Point(scale_i(0.01), 0)};
        CHECK_FALSE(pw::contains(hole, path.front()));
        CHECK_FALSE(pw::contains(hole, path.back()));
        CHECK_FALSE(pw::cross(box, pw::Line(path.front(), path.back())));
        CHECK_FALSE(box.contains(path.front()));
        size_t calls = 0;
        actual.prepare(handle, *input.target, 0, radius, [&calls]() { ++calls; });
        const size_t before = calls;
        Polyline native;
        for (const work::Point &point : path.points) native.points.emplace_back(point.x(), point.y());
        expected.prepare_crossing_test({layer, printed, 0, 0, radius, {}});
        CHECK(actual.can_cross_perimeter(path, false) == expected.can_cross_perimeter(native, false));
        REQUIRE(calls == before + 1);
        coverage.long_hole += calls - before;
        ++queries;
    }
    return queries;
}

TriangleMesh ring()
{
    TriangleMesh mesh = make_cube(20., 4., 0.6);
    TriangleMesh top = make_cube(20., 4., 0.6);
    top.translate(0., 16., 0.);
    mesh.merge(top);
    TriangleMesh left = make_cube(4., 12., 0.6);
    left.translate(0., 4., 0.);
    mesh.merge(left);
    TriangleMesh right = make_cube(4., 12., 0.6);
    right.translate(16., 4., 0.);
    mesh.merge(right);
    TriangleMesh island = make_cube(3., 3., 0.6);
    island.translate(25., 0., 0.);
    mesh.merge(island);
    return mesh;
}

TriangleMesh circular_ring()
{
    indexed_triangle_set mesh;
    for (int i = 0; i < 24; ++i) {
        const double angle = 2. * PI * i / 24.;
        for (float z : {0.f, 0.6f})
            for (float radius : {10.f, 6.f})
                mesh.vertices.emplace_back(float(10. + radius * std::cos(angle)),
                                           float(10. + radius * std::sin(angle)), z);
    }
    // Each adjacent pair supplies an outer wall, inner wall, top and bottom.
    for (int i = 0; i < 24; ++i) {
        const int a = i * 4, b = ((i + 1) % 24) * 4;
        mesh.indices.emplace_back(a, b, b + 2);
        mesh.indices.emplace_back(a, b + 2, a + 2);
        mesh.indices.emplace_back(a + 1, a + 3, b + 3);
        mesh.indices.emplace_back(a + 1, b + 3, b + 1);
        mesh.indices.emplace_back(a + 2, b + 2, b + 3);
        mesh.indices.emplace_back(a + 2, b + 3, a + 3);
        mesh.indices.emplace_back(a, a + 1, b + 1);
        mesh.indices.emplace_back(a, b + 1, b);
    }
    return TriangleMesh(mesh);
}
}
