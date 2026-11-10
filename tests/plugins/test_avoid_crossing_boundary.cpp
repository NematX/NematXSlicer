#include <catch2/catch.hpp>

#include <iostream>
#include <type_traits>

#include "plugin_test_helpers.hpp"
#include "test_data.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/internal/LayerIslandAccess.hpp"
#include "libslic3r/Api/plugin/cpp/AuxiliaryLayerHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/GCode/AvoidCrossingPerimeters.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/AvoidCrossing/TravelBoundary.hpp"
#include "libslic3r/Plugins/Common/ClipperHelpers.hpp"

/*
Exact boundary comparisons for the perimeter-avoiding travel router
=================================================================
The sliced-scene tests check that TravelBoundary reproduces the core
AvoidCrossingPerimeters boundaries exactly. A direct jagged perimeter subject also
compares core caches; the other direct subjects check inset outcomes and geometry.
For each sliced layer, compare copies the input, builds both
plugin boundaries and calls core travel_to to fill the core's lazy caches. Checks
compare rings, island indices, boxes, cumulative float lengths, original/inset
pairs, excluded top areas and grid cells without tolerance or reordering.

Scenes exercise different parts of construction:
- A square ring and separate island, with regional tools and top settings, check
  island indexing, tool clipping and top exclusions. Another object and repeated
  instances check external hole placement.
- A faceted annulus supplies oblique edges; a 0.05 mm square skips inset attempts.
  An auxiliary support layer checks the use of lower object slices and the absence
  of excluded tops while avoid_crossing_top is enabled in its regions.
  A sliced top layer has excluded tops without the support marker and none with it;
  both versions are compared with the core.
- Dumbbells and frames vary neck or wall widths through 0.6, 1.0, 1.5 and 2.5 mm;
  frames keep their holes. A notched plate checks a passage facing a long edge.
- A small-nozzle plate, with a pointed hole 0.005 mm from a long edge and spacing
  near 0.083 mm, reaches retries and final acceptance with a changed hole count.
- Direct owned LayerInput subjects bypass surface and extrusion generation. A
  square exhausts inset attempts and keeps its shape; narrow necks remain connected.
  Odd spacing gives a half-unit search radius.
- A jagged perimeter subject replaces the perimeter cache of the cube layer before
  both builders read it, bypassing slicing simplification. Its final inset splits into pieces;
  exact core comparisons check the retained largest piece and its grid.

No scene makes resampling grow outside the original island, so resample_original
stays zero. The odd-radius check reads the
core Point rounding; no subject distinguishes the port's rounding from truncation.
The print and API storage outlive every input copy and grid.

    sliced scene tests
    |-- ring, circular_ring, dumbbell, narrow_frame: construct controlled geometry
    |-- Print::process: generate layer slices and regional surfaces
    `-- compare: copy inputs and compare each tool's boundaries
        |-- TravelBoundary::init_layer: reset and prepare the interior test
        |-- TravelBoundary::build_internal / build_external: build plugin boundaries
        |-- AvoidCrossingPerimeters::travel_to: fill core boundary caches
        `-- check_boundary: compare geometry and cached indices exactly
            |-- check_polygon / check_expolygons / check_box: compare owned data
            `-- check_grid: compare borrowed rings and cell edge references
    direct subject test
    |-- TravelBoundary::init_layer: reset inset branch counts
    `-- TravelBoundary::build_internal: exercise inset choices without slicing
    direct jagged subject test
    |-- Print::process: provide layer settings and owned perimeter storage
    |-- build_input: copy the replaced perimeter cache into owned plugin input
    |-- TravelBoundary::init_layer / AvoidCrossingPerimeters::init_layer: compare slice offsets
    |-- TravelBoundary::build_internal: count final split insets
    `-- AvoidCrossingPerimeters::travel_to / check_boundary: compare lazy core caches
    support top witness test
    |-- Print::process: generate real top fill surfaces
    |-- build_input: copy the layer without, then with, the support marker
    |-- TravelBoundary::init_layer / AvoidCrossingPerimeters::init_layer: compare slice offsets
    `-- build_internal / travel_to / check_boundary: compare excluded tops and boundaries
*/
namespace {
using namespace Slic3r;
namespace port = slic3r_api::LayerExtrusionEdit::AvoidCrossing;
namespace pw = slic3r_api::PluginCommon::polygon_work;
namespace work = slic3r_api::PluginCommon::work;
namespace eg = slic3r_api::PluginCommon::EdgeGrid;

struct Coverage {
    size_t layers = 0, tools = 0, tops = 0, support = 0, support_top_enabled = 0, holes = 0, islands = 0;
    size_t rings = 0, points = 0, growth = 0, external = 0, grids = 0;
    port::InnerOffsetCounts inset;
};

// Construct a shallow ring and disconnected square island from joined bars.
TriangleMesh ring();
// Construct a closed faceted annulus whose oblique edges exercise resampling truncation.
TriangleMesh circular_ring();
// Join two 10 mm squares with a 5 mm long neck of the requested width.
TriangleMesh dumbbell(double width);
// Build a rectangular frame with one wall close to its inner hole.
TriangleMesh narrow_frame(double width);
// Compare ring vertices in source order and count checked points.
void check_polygon(const pw::Polygon &actual, const Polygon &expected, Coverage &coverage);
// Compare contour and hole sequences without moving or normalizing either collection.
void check_expolygons(const pw::ExPolygons &actual, const ExPolygons &expected, Coverage &coverage);
// Compare integer box corners and its defined flag.
void check_box(const work::BoundingBox &actual, const BoundingBox &expected);
// Compare grid dimensions, borrowed ring points and cell edge references.
void check_grid(const eg::Grid &actual, const EdgeGrid::Grid &expected, Coverage &coverage);
// Compare all boundary data, including exact float distances and excluded tops.
void check_boundary(const port::Boundary &actual, const AvoidCrossingPerimeters::Boundary &expected, Coverage &coverage);
// Build snapshots and lazy core caches for one sliced layer, then compare both tool-specific interiors.
void compare(const Print &print, const Layer &layer, port::LayerIdentity identity, Coverage &coverage);

// Supply an owned jagged perimeter island to both builders and compare the retained largest inset piece.
TEST_CASE("Direct jagged island matches core after a split inset", "[plugins][avoid-crossing-boundary]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const Points outline{{4315200, 0}, {9433521, 2527704}, {2973758, 1716900}, {2591476, 2591476},
        {2955400, 5118903}, {820016, 3060343}, {0, 6799000}, {-770582, 2875851},
        {-2367600, 4100803}, {-2776879, 2776879}, {-3877889, 2238900}, {-5794106, 1552526},
        {-5329200, 0}, {-5258790, -1409089}, {-7548797, -4358300}, {-5282724, -5282724},
        {-179200, -310384}, {-1690787, -6310104}, {0, -280200}, {2285709, -8530381},
        {807400, -1398458}, {2748312, -2748312}, {2566726, -1481900}, {2243749, -601211}};
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "1"}, {"fill_density", "0%"}, {"top_solid_layers", "0"},
        {"bottom_solid_layers", "0"}, {"skirts", "0"}, {"brim_width", "0"},
        {"gap_fill_enabled", "0"}, {"thin_walls", "0"}, {"thin_perimeters", "0"},
        {"avoid_crossing_top", "0"}, {"perimeter_extrusion_width", "0.492922"},
        {"first_layer_extrusion_width", "0.492922"}});
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(20., 20., 0.6), ModelVolumeType::MODEL_PART, false);
    object->add_instance();
    object->ensure_on_bed();
    Print print;
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    Coverage coverage;
    size_t multiple = 0;
    uint32_t index = 0;
    for (Layer &layer : print.object(0).layers()) {
        REQUIRE(layer.islands().size() == 1);
        // The mutable layer owns this cache; replace only its perimeter subject to avoid slicing simplification.
        ExPolygon polygon;
        polygon.contour.points = outline;
        ApiInternal::LayerIslandAccess::perimeter_slices_mutable(layer.islands().front()) = {polygon};
        PluginStorage storage;
        storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
        const slic3r_api::Print view(reinterpret_cast<const print_handle *>(&print));
        const port::Input input = port::build_input(handle, view, {0, index++, false});
        std::cout << "jagged spacing=" << input.target->perimeter_spacing << std::endl;
        port::TravelBoundary actual;
        AvoidCrossingPerimeters expected;
        actual.init_layer(handle, *input.target);
        expected.init_layer(layer);
        check_expolygons(actual.layer_slices_offset(), expected.layer_slices_offset(), coverage);
        actual.build_internal(handle, input, 0);
        multiple += actual.inner_offset_counts().multiple;
        REQUIRE_FALSE(actual.internal_boundary().boundaries.empty());
        const pw::Polygon &boundary = actual.internal_boundary().boundaries.front();
        const Point from(boundary.front().x(), boundary.front().y());
        const Point to(boundary[boundary.size() / 2].x(), boundary[boundary.size() / 2].y());
        expected.travel_to({layer, from, Point::Zero(), extruder_index_t(0), 0., false}, to);
        check_boundary(actual.internal_boundary(), expected.internal_boundary(), coverage);
    }
    std::cout << "jagged island multiple=" << multiple << std::endl;
    REQUIRE(multiple > 0);
}

// Compare a real top layer with and without the support marker so omitted tops change the result.
TEST_CASE("Support marker omits real top surfaces from travel boundaries", "[plugins][avoid-crossing-boundary]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "2"}, {"fill_density", "20%"}, {"top_solid_layers", "1"},
        {"bottom_solid_layers", "1"}, {"skirts", "0"}, {"brim_width", "0"},
        {"gap_fill_enabled", "0"}, {"thin_walls", "0"}, {"thin_perimeters", "0"},
        {"avoid_crossing_top", "1"}});
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(20., 20., 0.6), ModelVolumeType::MODEL_PART, false);
    object->add_instance();
    object->ensure_on_bed();
    Print print;
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    REQUIRE_FALSE(print.object(0).layers().empty());
    const uint32_t index = uint32_t(print.object(0).layers().size() - 1);
    Layer &layer = print.object(0).layers().back();
    Coverage coverage;
    for (bool support : {false, true}) {
        CAPTURE(support);
        if (support) layer.get_or_add_property<LayerSupportProperty>();
        PluginStorage storage;
        storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
        const slic3r_api::Print view(reinterpret_cast<const print_handle *>(&print));
        const port::Input input = port::build_input(handle, view, {0, index, false});
        REQUIRE(input.target->support == support);
        size_t tops = 0;
        REQUIRE_FALSE(input.target->regions.empty());
        for (const port::RegionInput &region : input.target->regions) {
            REQUIRE(region.avoid_crossing_top);
            tops += region.top_surfaces.size();
        }
        if (!support) REQUIRE(tops > 0);
        port::TravelBoundary actual;
        AvoidCrossingPerimeters expected;
        actual.init_layer(handle, *input.target);
        expected.init_layer(layer);
        check_expolygons(actual.layer_slices_offset(), expected.layer_slices_offset(), coverage);
        actual.build_internal(handle, input, 0);
        REQUIRE(actual.internal_boundary().to_avoid.empty() == support);
        REQUIRE_FALSE(actual.internal_boundary().boundaries.empty());
        const pw::Polygon &boundary = actual.internal_boundary().boundaries.front();
        const Point from(boundary.front().x(), boundary.front().y());
        const Point to(boundary[boundary.size() / 2].x(), boundary[boundary.size() / 2].y());
        expected.travel_to({layer, from, Point::Zero(), extruder_index_t(0), 0., false}, to);
        REQUIRE(expected.internal_boundary().to_avoid.empty() == support);
        check_boundary(actual.internal_boundary(), expected.internal_boundary(), coverage);
        std::cout << "top witness support=" << support << " input_tops=" << tops
                  << " excluded_tops=" << actual.internal_boundary().to_avoid.size() << std::endl;
    }
}

// Build owned polygon subjects to check retained small islands and narrow necks.
TEST_CASE("Direct boundary subjects retain empty-inset squares and narrow necks", "[plugins][avoid-crossing-boundary]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const int shape = GENERATE(0, 1, 2, 3);
    CAPTURE(shape);
    ExPolygon polygon;
    if (shape == 0) {
        polygon.contour.points = {{0, 0}, {scale_i(0.3), 0}, {scale_i(0.3), scale_i(0.3)}, {0, scale_i(0.3)}};
    } else {
        const coord_t half = scale_i((0.2 + shape * 0.1) / 2.);
        const coord_t lo = scale_i(2.5) - half, hi = scale_i(2.5) + half;
        polygon.contour.points = {{0, 0}, {scale_i(5.), 0}, {scale_i(5.), lo}, {scale_i(10.), lo},
            {scale_i(10.), 0}, {scale_i(15.), 0}, {scale_i(15.), scale_i(5.)}, {scale_i(10.), scale_i(5.)},
            {scale_i(10.), hi}, {scale_i(5.), hi}, {scale_i(5.), scale_i(5.)}, {0, scale_i(5.)}};
    }
    const ExPolygons subject{polygon};
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    port::LayerInput layer;
    layer.perimeter_spacing = 450001;
    layer.external_perimeter_width = 450001.f;
    const double first_search_radius = 3. * (1.5 * layer.perimeter_spacing);
    REQUIRE(first_search_radius - std::floor(first_search_radius) == 0.5);
    CHECK(Point(first_search_radius, first_search_radius).x() == coord_t(std::round(first_search_radius)));
    layer.perimeter_slices = slic3r_api::PluginCommon::clipper::from_expolygon_collection_view(
        slic3r_api::ExPolygonCollection(reinterpret_cast<const expolygon_collection_handle *>(&subject)));
    port::Input input;
    input.target = std::make_unique<const port::LayerInput>(std::move(layer));
    port::TravelBoundary boundary;
    boundary.init_layer(handle, *input.target);
    CHECK(boundary.inner_offset_counts().same_holes == 0);
    boundary.build_internal(handle, input, 0);
    const port::InnerOffsetCounts &counts = boundary.inner_offset_counts();
    std::cout << "direct subject=" << shape << " small=" << counts.small << " same_holes=" << counts.same_holes
              << " retry=" << counts.retry << " changed_holes=" << counts.changed_holes
              << " multiple=" << counts.multiple << " empty_original=" << counts.empty_original
              << " kept=" << counts.resample_kept << std::endl;
    CHECK(counts.small == 0);
    REQUIRE(boundary.internal_boundary().boundary_growth.size() == 1);
    if (shape == 0) {
        CHECK(counts.empty_original > 0);
        CHECK(counts.retry > 0);
        const std::pair<pw::ExPolygon, pw::ExPolygon> &growth = boundary.internal_boundary().boundary_growth.front();
        CHECK(slic3r_api::PluginCommon::clipper::diff_ex(handle, growth.first, growth.second).empty());
        CHECK(slic3r_api::PluginCommon::clipper::diff_ex(handle, growth.second, growth.first).empty());
    } else CHECK(counts.same_holes > 0);
}

TEST_CASE("Plugin travel boundaries match sliced core caches", "[plugins][avoid-crossing-boundary]")
{
    static_assert(!std::is_copy_constructible<port::Boundary>::value, "Grids borrow owned point arrays");
    static_assert(!std::is_move_constructible<port::Boundary>::value, "Boundary addresses stay stable");
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
    TriangleMesh round = circular_ring();
    round.translate(40., 0., 0.);
    object->add_volume(round, ModelVolumeType::MODEL_PART, false);
    ModelVolume *modifier = object->add_volume(make_cube(10., 20., 0.6), ModelVolumeType::PARAMETER_MODIFIER, false);
    DynamicPrintConfig regional;
    regional.set_deserialize_strict({{"infill_extruder", "2"}, {"solid_infill_extruder", "2"}});
    modifier->config.assign_config(regional);
    object->add_instance();
    object->add_instance()->set_offset(Vec3d(50., 0., 0.));
    object->ensure_on_bed();
    ModelObject *second = model.add_object();
    second->add_volume(ring(), ModelVolumeType::MODEL_PART, false);
    second->add_instance()->set_offset(Vec3d(0., 50., 0.));
    second->ensure_on_bed();
    size_t neck_index = 0;
    for (double width : {0.6, 1.0, 1.5, 2.5}) {
        ModelObject *neck = model.add_object();
        neck->add_volume(dumbbell(width), ModelVolumeType::MODEL_PART, false);
        neck->add_instance()->set_offset(Vec3d(100., 30. * neck_index++, 0.));
        neck->ensure_on_bed();
        ModelObject *frame = model.add_object();
        frame->add_volume(narrow_frame(width), ModelVolumeType::MODEL_PART, false);
        frame->add_instance()->set_offset(Vec3d(140., 30. * neck_index, 0.));
        frame->ensure_on_bed();
    }
    // The long bottom edge faces a notch across a passage narrower than the full inset.
    TriangleMesh notched = make_cube(30., 0.5, 0.6);
    for (double x : {0., 16.}) {
        TriangleMesh side = make_cube(14., 9.5, 0.6);
        side.translate(float(x), 0.5f, 0.f);
        notched.merge(side);
    }
    ModelObject *notch = model.add_object();
    notch->add_volume(notched, ModelVolumeType::MODEL_PART, false);
    notch->add_instance()->set_offset(Vec3d(180., 0., 0.));
    notch->ensure_on_bed();
    Print print;
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    REQUIRE(print.objects().size() == 11);
    Coverage coverage;
    for (uint32_t o = 0; o < print.objects().size(); ++o) {
        uint32_t index = 0;
        for (const Layer &layer : print.object(o).layers()) compare(print, layer, {o, index++, false}, coverage);
    }
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    const slic3r_api::Print view(reinterpret_cast<const print_handle *>(&print));
    const Layer &lower = print.object(0).layers().front();
    const coord_t z = view.object(0).layer(0).print_z();
    const slic3r_api::AuxiliaryLayerBuildResult built = slic3r_api::build_auxiliary_layer_regions_from_subject(
        handle, view, view.object(0),
        slic3r_api::ExPolygonCollection(reinterpret_cast<const expolygon_collection_handle *>(&lower.lslices())),
        scale_i(0.2), z, z - scale_i(0.1));
    REQUIRE(built.created);
    built.layer.mutable_properties().get_or_add(slic3r_api::LayerSupportProperty::key);
    compare(print, *reinterpret_cast<const Layer *>(built.layer.handle()), {0, 0, true}, coverage);
    std::cout << "boundary coverage: layers=" << coverage.layers << " tools=" << coverage.tools
              << " tops=" << coverage.tops << " support=" << coverage.support
              << " support_top_enabled=" << coverage.support_top_enabled << " holes=" << coverage.holes
              << " islands=" << coverage.islands << " rings=" << coverage.rings << " points=" << coverage.points
              << " growth=" << coverage.growth << " external=" << coverage.external << " grids=" << coverage.grids << std::endl;
    std::cout << "inner offset: original=" << coverage.inset.resample_original << " kept=" << coverage.inset.resample_kept
              << " small=" << coverage.inset.small << " same_holes=" << coverage.inset.same_holes
              << " retry=" << coverage.inset.retry << " changed_holes=" << coverage.inset.changed_holes
              << " multiple=" << coverage.inset.multiple << " empty_original=" << coverage.inset.empty_original << std::endl;
    CHECK(coverage.layers >= 31);
    CHECK(coverage.tools > 0);
    CHECK(coverage.tops > 0);
    CHECK(coverage.support > 0);
    REQUIRE(coverage.support_top_enabled > 0);
    CHECK(coverage.holes > 0);
    CHECK(coverage.islands > 0);
    CHECK(coverage.external > 0);
    CHECK(coverage.growth > 0);
    CHECK(coverage.inset.resample_kept > 0);
    CHECK(coverage.inset.small > 0);
    CHECK(coverage.inset.same_holes > 0);
}

// Slice a pointed hole close to a long edge, then compare geometry on the final changed-hole inset attempt.
TEST_CASE("Sliced pointed hole matches core after inset retries", "[plugins][avoid-crossing-boundary]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"layer_height", "0.04"}, {"first_layer_height", "0.04"},
        {"nozzle_diameter", "0.08"}, {"min_layer_height", "0.02"}, {"max_layer_height", "0.06"},
        {"perimeter_extrusion_width", "0.089584"}, {"external_perimeter_extrusion_width", "0.089584"},
        {"first_layer_extrusion_width", "0.089584"}, {"perimeters", "2"}, {"fill_density", "0%"},
        {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"}, {"skirts", "0"}, {"brim_width", "0"},
        {"gap_fill_enabled", "0"}, {"thin_walls", "0"}, {"thin_perimeters", "0"},
        {"xy_size_compensation", "0"}, {"avoid_crossing_top", "0"}});
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(20., 10., 0.12), ModelVolumeType::MODEL_PART, false);
    // A closed triangular prism subtracts the hole without introducing another print region.
    indexed_triangle_set hole;
    hole.vertices = {{10.123f, 0.005f, 0.f}, {9.f, 5.f, 0.f}, {11.f, 5.f, 0.f},
        {10.123f, 0.005f, 0.12f}, {9.f, 5.f, 0.12f}, {11.f, 5.f, 0.12f}};
    hole.indices = {{0, 1, 2}, {3, 5, 4}, {0, 3, 1}, {1, 3, 4},
        {1, 4, 2}, {2, 4, 5}, {2, 5, 0}, {0, 5, 3}};
    object->add_volume(TriangleMesh(std::move(hole)), ModelVolumeType::NEGATIVE_VOLUME, false);
    object->add_instance();
    object->ensure_on_bed();
    Print print;
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    Coverage coverage;
    uint32_t index = 0;
    for (const Layer &layer : print.object(0).layers()) {
        REQUIRE(layer.regions().size() == 1);
        std::cout << "pointed hole layer=" << index << " spacing="
                  << unscaled(layer.regions().front().flow(frPerimeter).scaled_spacing()) << std::endl;
        compare(print, layer, {0, index++, false}, coverage);
    }
    std::cout << "pointed hole retry=" << coverage.inset.retry << " changed_holes=" << coverage.inset.changed_holes << std::endl;
    CHECK(coverage.inset.retry > 0);
    CHECK(coverage.inset.changed_holes > 0);
}

void compare(const Print &print, const Layer &layer, port::LayerIdentity identity, Coverage &coverage)
{
    CAPTURE(identity.object, identity.layer, identity.auxiliary);
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    const slic3r_api::Print view(reinterpret_cast<const print_handle *>(&print));
    const port::Input input = port::build_input(handle, view, identity);
    const port::ExternalInput external = port::build_external_input(view, identity);
    ++coverage.layers;
    if (input.target->support) {
        ++coverage.support;
        REQUIRE_FALSE(input.lower_perimeter_slices.empty());
        for (const port::RegionInput &region : input.target->regions)
            if (region.avoid_crossing_top) ++coverage.support_top_enabled;
    }
    for (const port::RegionInput &region : input.target->regions)
        coverage.tops += region.avoid_crossing_top ? region.top_surfaces.size() : 0;
    for (const ExPolygon &island : layer.lslices()) coverage.holes += island.holes.size();
    if (layer.lslices().size() > 1) ++coverage.islands;
    for (int tool : {0, 1}) {
        // This auxiliary subject has no generated fill paths for the second tool.
        if (identity.auxiliary && tool != 0) continue;
        CAPTURE(tool);
        bool used = false;
        for (const port::RegionInput &region : input.target->regions)
            used = used || region.perimeter_extruder == tool + 1 || region.infill_extruder == tool + 1 || region.solid_infill_extruder == tool + 1;
        if (!used) continue;
        ++coverage.tools;
        port::TravelBoundary actual;
        AvoidCrossingPerimeters expected;
        actual.init_layer(handle, *input.target);
        expected.init_layer(layer);
        check_expolygons(actual.layer_slices_offset(), expected.layer_slices_offset(), coverage);
        actual.build_internal(handle, input, tool);
        if (input.target->support) REQUIRE(actual.internal_boundary().to_avoid.empty());
        const port::InnerOffsetCounts &counts = actual.inner_offset_counts();
        coverage.inset.resample_original += counts.resample_original;
        coverage.inset.resample_kept += counts.resample_kept;
        coverage.inset.small += counts.small;
        coverage.inset.same_holes += counts.same_holes;
        coverage.inset.retry += counts.retry;
        coverage.inset.changed_holes += counts.changed_holes;
        coverage.inset.multiple += counts.multiple;
        coverage.inset.empty_original += counts.empty_original;
        REQUIRE_FALSE(actual.internal_boundary().boundaries.empty());
        const pw::Polygon &ring = actual.internal_boundary().boundaries.front();
        const Point from(ring[0].x(), ring[0].y());
        const Point to(ring[ring.size() / 2].x(), ring[ring.size() / 2].y());
        expected.travel_to({layer, from, Point::Zero(), extruder_index_t(tool), 0., false}, to);
        check_boundary(actual.internal_boundary(), expected.internal_boundary(), coverage);
        actual.build_external(handle, *input.target, external);
        expected.use_external_mp(true);
        REQUIRE_FALSE(actual.external_boundary().boundaries.empty());
        const pw::Polygon &outer = actual.external_boundary().boundaries.front();
        const Point external_from(outer[0].x(), outer[0].y());
        const Point external_to(outer[outer.size() / 2].x(), outer[outer.size() / 2].y());
        expected.travel_to({layer, external_from, Point::Zero(), extruder_index_t(tool), 0., false}, external_to);
        check_boundary(actual.external_boundary(), expected.external_boundary(), coverage);
        coverage.external += actual.external_boundary().boundaries.size();
    }
}

void check_boundary(const port::Boundary &actual, const AvoidCrossingPerimeters::Boundary &expected, Coverage &coverage)
{
    REQUIRE(actual.boundaries.size() == expected.boundaries.size());
    for (size_t i = 0; i < actual.boundaries.size(); ++i) check_polygon(actual.boundaries[i], expected.boundaries[i], coverage);
    CHECK(actual.islands == expected.islands);
    CHECK(actual.bbox.min.x() == expected.bbox.min.x());
    CHECK(actual.bbox.min.y() == expected.bbox.min.y());
    CHECK(actual.bbox.max.x() == expected.bbox.max.x());
    CHECK(actual.bbox.max.y() == expected.bbox.max.y());
    CHECK(actual.bbox.defined == expected.bbox.defined);
    REQUIRE(actual.bboxes.size() == expected.bboxes.size());
    for (size_t i = 0; i < actual.bboxes.size(); ++i) check_box(actual.bboxes[i], expected.bboxes[i]);
    CHECK(actual.boundaries_params == expected.boundaries_params);
    REQUIRE(actual.boundary_growth.size() == expected.boundary_growth.size());
    for (size_t i = 0; i < actual.boundary_growth.size(); ++i) {
        check_expolygons({actual.boundary_growth[i].first}, {expected.boundary_growth[i].first}, coverage);
        check_expolygons({actual.boundary_growth[i].second}, {expected.boundary_growth[i].second}, coverage);
        ++coverage.growth;
    }
    check_expolygons(actual.to_avoid, expected.to_avoid, coverage);
    check_grid(actual.grid, expected.grid, coverage);
    check_grid(actual.to_avoid_grid, expected.to_avoid_grid, coverage);
    if (!actual.to_avoid.empty())
        CHECK(actual.to_avoid_grid.contours().front()[0].x == actual.to_avoid.front().contour.points.front().x());
}

void check_grid(const eg::Grid &actual, const EdgeGrid::Grid &expected, Coverage &coverage)
{
    ++coverage.grids;
    CHECK(actual.resolution() == expected.resolution());
    CHECK(actual.rows() == expected.rows());
    CHECK(actual.cols() == expected.cols());
    check_box(actual.bbox(), expected.bbox());
    REQUIRE(actual.contours().size() == expected.contours().size());
    for (size_t i = 0; i < actual.contours().size(); ++i) {
        REQUIRE(actual.contours()[i].size() == size_t(expected.contours()[i].end() - expected.contours()[i].begin()));
        for (size_t j = 0; j < actual.contours()[i].size(); ++j) {
            CHECK(actual.contours()[i][j].x == expected.contours()[i].segment_start(j).x());
            CHECK(actual.contours()[i][j].y == expected.contours()[i].segment_start(j).y());
        }
    }
    for (size_t y = 0; y < actual.rows(); ++y)
        for (size_t x = 0; x < actual.cols(); ++x) {
            const auto a = actual.cell_data_range(y, x);
            const auto b = expected.cell_data_range(y, x);
            CHECK(std::vector<std::pair<size_t, size_t>>(a.first, a.second) ==
                  std::vector<std::pair<size_t, size_t>>(b.first, b.second));
        }
}

void check_box(const work::BoundingBox &actual, const BoundingBox &expected)
{
    CHECK(actual.min.x() == expected.min.x());
    CHECK(actual.min.y() == expected.min.y());
    CHECK(actual.max.x() == expected.max.x());
    CHECK(actual.max.y() == expected.max.y());
    CHECK(actual.defined == expected.defined);
}

void check_expolygons(const pw::ExPolygons &actual, const ExPolygons &expected, Coverage &coverage)
{
    REQUIRE(actual.size() == expected.size());
    for (size_t i = 0; i < actual.size(); ++i) {
        check_polygon(actual[i].contour, expected[i].contour, coverage);
        REQUIRE(actual[i].holes.size() == expected[i].holes.size());
        for (size_t h = 0; h < actual[i].holes.size(); ++h) check_polygon(actual[i].holes[h], expected[i].holes[h], coverage);
    }
}

void check_polygon(const pw::Polygon &actual, const Polygon &expected, Coverage &coverage)
{
    ++coverage.rings;
    REQUIRE(actual.size() == expected.size());
    for (size_t i = 0; i < actual.size(); ++i) {
        CHECK(actual[i].x() == expected[i].x());
        CHECK(actual[i].y() == expected[i].y());
        ++coverage.points;
    }
}

TriangleMesh ring()
{
    TriangleMesh mesh = make_cube(20., 4., 0.6);
    TriangleMesh bar = make_cube(20., 4., 0.6);
    bar.translate(0., 16., 0.);
    mesh.merge(bar);
    bar = make_cube(4., 12., 0.6);
    bar.translate(0., 4., 0.);
    mesh.merge(bar);
    bar.translate(16., 0., 0.);
    mesh.merge(bar);
    bar = make_cube(5., 5., 0.6);
    bar.translate(25., 0., 0.);
    mesh.merge(bar);
    bar = make_cube(0.05, 0.05, 0.6);
    bar.translate(32., 0., 0.);
    mesh.merge(bar);
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
    // Join the four surfaces of each sector, retaining a clockwise inner wall.
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

TriangleMesh dumbbell(double width)
{
    TriangleMesh mesh = make_cube(10., 10., 0.6);
    TriangleMesh right = make_cube(10., 10., 0.6);
    right.translate(15., 0., 0.);
    mesh.merge(right);
    TriangleMesh neck = make_cube(7., width, 0.6);
    neck.translate(9., 5. - width / 2., 0.);
    mesh.merge(neck);
    return mesh;
}

TriangleMesh narrow_frame(double width)
{
    TriangleMesh mesh = make_cube(20., 3., 0.6);
    TriangleMesh bar = make_cube(20., 3., 0.6);
    bar.translate(0., 17., 0.);
    mesh.merge(bar);
    bar = make_cube(3., 14., 0.6);
    bar.translate(0., 3., 0.);
    mesh.merge(bar);
    bar = make_cube(width, 14., 0.6);
    bar.translate(20. - width, 3., 0.);
    mesh.merge(bar);
    return mesh;
}

}
