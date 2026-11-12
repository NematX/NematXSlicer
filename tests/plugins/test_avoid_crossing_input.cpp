#include <catch2/catch.hpp>

#include <set>

#include "plugin_test_helpers.hpp"
#include "test_data.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/AuxiliaryLayerHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintRegion.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/AvoidCrossing/AvoidCrossingInput.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"

/*
These tests check that the plugin copies the layer geometry and settings needed
by crossing tests and travel routing. Small sliced meshes provide single and
multiple islands, shared regional tops, unequal object heights and instances.
Core public accessors supply expected rings and flow averages. Comparisons retain
every vertex, hole and collection position, without normalizing shapes.

Print::process builds object islands and fill surfaces. Support is not generated
by that call: with support_material disabled, the support test manually attaches
layers through build_auxiliary_layer_regions_from_subject and marks them with
LayerSupportProperty. It checks copied input without generating support toolpaths.

    test cases: build sliced scenes or manually attach support subjects
    |-- prepare object geometry
    |   |-- Print::process: build islands and regional fill surfaces
    |   `-- default-spacing test: run layer-height generation and slicing only
    `-- check_print: visit normal and auxiliary targets and probe Z lookups
        |-- build_input: copy target geometry and additional object-layer slices for support
        |-- build_external_input: copy ordered holes, shifts and spacing
        |-- check_layer: compare fields, regional geometry and flow averages
        `-- check_external: compare against core Z lookups and instance shifts

The Print stays alive throughout the comparisons. Copies are checked after
temporary plugin storage is destroyed; this demonstrates independence from that
storage, not from a destroyed Print. Coverage counters establish the particular
scene branches asserted by the tests, not exhaustive reconstruction coverage.
The default-spacing test stops after slicing: tool collection depends on configs,
so extrusion generation for zero-perimeter objects is not needed for that check.
*/

namespace {
using namespace Slic3r;
namespace input = slic3r_api::LayerExtrusionEdit::AvoidCrossing;
namespace pw = slic3r_api::PluginCommon::polygon_work;

// Record selected scene properties and reconstruction branches checked by explicit assertions below.
struct Coverage {
    size_t single = 0;
    size_t multiple = 0;
    size_t shared_top = 0;
    size_t empty_region = 0;
    size_t support = 0;
    size_t support_same_z = 0;
    size_t support_first = 0;
    size_t support_higher = 0;
    size_t support_duplicate_holes = 0;
    size_t support_lower_perimeters = 0;
};

// Compare ring coordinates without rotating, reversing or sorting either ring.
void check_ring(const pw::Polygon &actual, const Polygon &expected);
// Compare complete ordered islands including holes; failures identify the exact element.
void check_polygons(const pw::ExPolygons &actual, const ExPolygons &expected);
// Independently select regional tops from core region-island memberships.
ExPolygons core_tops(const Layer &layer, const LayerRegion &region, Coverage &coverage);
// Compare every layer field with native accessors and flow averages, recording exercised branches.
void check_layer(const input::LayerInput &actual, const Layer &layer, Coverage &coverage);
// Compare external holes, instance shifts and the global average with core Z lookups.
void check_external(const input::ExternalInput &actual, const Layer &layer, Coverage &coverage);
// Build and compare every normal and auxiliary target, testing Z boundaries independently.
Coverage check_print(const Print &print);
// Return a shallow square ring made from four bars, optionally with a separate island.
TriangleMesh ring_mesh(bool island);

TEST_CASE("Avoid-crossing input preserves sliced regional sequences", "[plugins][avoid-crossing-input]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "2"}, {"fill_density", "20%"}, {"top_solid_layers", "1"},
        {"bottom_solid_layers", "1"}, {"skirts", "0"}, {"brim_width", "0"},
        {"nozzle_diameter", "0.4,0.6"}, {"min_layer_height", "0.05,0.05"},
        {"max_layer_height", "0.3,0.3"}, {"avoid_crossing_top", "1"}});
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(ring_mesh(true), ModelVolumeType::MODEL_PART, false);
    ModelVolume *modifier = object->add_volume(make_cube(10., 20., 0.4), ModelVolumeType::PARAMETER_MODIFIER, false);
    DynamicPrintConfig region_config;
    region_config.set_deserialize_strict({{"avoid_crossing_top", "0"}, {"infill_extruder", "2"},
        {"solid_infill_extruder", "2"}, {"avoid_travel_island", "1"}, {"avoid_travel_island_weight", "2"}});
    modifier->config.assign_config(region_config);
    object->add_instance();
    object->add_instance()->set_offset(Vec3d(30., 0., 0.));
    ModelObject *second = model.add_object();
    second->add_volume(make_cube(5., 5., 1.), ModelVolumeType::MODEL_PART, false);
    second->add_instance()->set_offset(Vec3d(65., 0., 0.));
    DynamicPrintConfig second_config;
    second_config.set_deserialize_strict({{"brim_width", "1"}, {"perimeter_extrusion_width", "0.9"}});
    second->config.assign_config(second_config);
    model.center_instances_around_point({100., 100.});
    Print print;
    for (ModelObject &part : model.objects()) {
        part.ensure_on_bed();
        print.auto_assign_extruders(&part);
    }
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    REQUIRE(print.objects().size() == 2);
    CHECK(print.object(0).layers().back().scaled_print_z() != print.object(1).layers().back().scaled_print_z());
    const Coverage coverage = check_print(print);
    CHECK(coverage.single > 0);
    CHECK(coverage.multiple > 0);
    CHECK(coverage.empty_region > 0);
}

TEST_CASE("Avoid-crossing input clips tops of a shared region island", "[plugins][avoid-crossing-input]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "2"}, {"fill_density", "20%"}, {"top_solid_layers", "1"},
        {"bottom_solid_layers", "1"}, {"skirts", "0"}, {"brim_width", "0"}, {"avoid_crossing_top", "1"}});
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(20., 20., 0.6), ModelVolumeType::MODEL_PART, false);
    ModelVolume *modifier = object->add_volume(make_cube(10., 20., 0.6), ModelVolumeType::PARAMETER_MODIFIER, false);
    DynamicPrintConfig region_config;
    region_config.set_deserialize_strict({{"avoid_crossing_top", "0"}});
    modifier->config.assign_config(region_config);
    object->add_instance();
    object->ensure_on_bed();
    Print print;
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    const Coverage coverage = check_print(print);
    CHECK(coverage.shared_top > 0);
}

TEST_CASE("Avoid-crossing default spacing includes other objects' brim settings", "[plugins][avoid-crossing-input]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "0"}, {"fill_density", "20%"}, {"top_solid_layers", "1"},
        {"bottom_solid_layers", "1"}, {"skirts", "0"}, {"brim_width", "0"},
        {"nozzle_diameter", "0.4,0.6"}, {"min_layer_height", "0.05,0.05"},
        {"max_layer_height", "0.3,0.3"}, {"perimeter_extruder", "1"},
        {"infill_extruder", "2"}, {"solid_infill_extruder", "2"}});
    Model model;
    ModelObject *plain = model.add_object();
    plain->add_volume(make_cube(5., 5., 0.4), ModelVolumeType::MODEL_PART, false);
    plain->add_instance();
    ModelObject *brim = model.add_object();
    brim->add_volume(make_cube(5., 5., 0.6), ModelVolumeType::MODEL_PART, false);
    brim->add_instance()->set_offset(Vec3d(15., 0., 0.));
    DynamicPrintConfig brim_config;
    brim_config.set_deserialize_strict({{"brim_width", "1"}});
    brim->config.assign_config(brim_config);
    Print print;
    print.apply(model, config);
    print.set_status_silent();
    Steps::StepLayerHeightGeneration::run_step(Orchestrator::instance(), print);
    Steps::StepSlicing::run_step(Orchestrator::instance(), print);
    REQUIRE(print.object(0).config().brim_width.value == 0.);
    REQUIRE(print.object(0).object_extruders().size() == 2);
    check_print(print);
    PluginStorage storage;
    const slic3r_api::Print view(reinterpret_cast<const print_handle *>(&print));
    const input::Input snapshot = input::build_input(reinterpret_cast<storage_handle *>(&storage), view, {0, 0, false}, nullptr);
    CHECK(snapshot.target->default_perimeter_spacing == (scale_i(0.4) + scale_i(0.6)) / 2);
}

TEST_CASE("Avoid-crossing input copies support object context at exact Z", "[plugins][avoid-crossing-input]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "2"}, {"fill_density", "20%"}, {"skirts", "0"}, {"brim_width", "0"},
        {"support_material", "0"}, {"nozzle_diameter", "0.4"},
        {"gap_fill_enabled", "0"}, {"thin_walls", "0"}, {"thin_perimeters", "0"},
        {"top_solid_layers", "1"}, {"bottom_solid_layers", "1"}});
    Model model;
    Print print;
    Slic3r::Test::init_print({ring_mesh(false)}, print, model, config);
    print.process();
    PluginStorage storage;
    const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
    const slic3r_api::Object object_view = print_view.object(0);
    const ExPolygons &subject = print.object(0).layers().front().lslices();
    REQUIRE_FALSE(subject.empty());
    REQUIRE_FALSE(subject.front().holes.empty());
    // Equal-Z and intermediate-Z support targets exercise duplication and strict predecessor lookup.
    for (coord_t z : {object_view.layer(1).print_z(), object_view.layer(0).print_z(),
                      coord_t((object_view.layer(0).print_z() + object_view.layer(1).print_z()) / 2)}) {
        const slic3r_api::AuxiliaryLayerBuildResult built = slic3r_api::build_auxiliary_layer_regions_from_subject(
            reinterpret_cast<storage_handle *>(&storage), print_view, object_view,
            slic3r_api::ExPolygonCollection(reinterpret_cast<const expolygon_collection_handle *>(&subject)),
            scale_i(0.2), z, z - scale_i(0.1));
        REQUIRE(built.created);
        built.layer.mutable_properties().get_or_add(slic3r_api::LayerSupportProperty::key);
        const Layer &native = *reinterpret_cast<const Layer *>(built.layer.handle());
        // Public auxiliary indices follow Z order, independently of native creation ids.
        uint32_t layer_index = 0;
        while (layer_index < object_view.auxiliary_layer_count() &&
               object_view.auxiliary_layer(layer_index).handle() != built.layer.handle())
            ++layer_index;
        REQUIRE(layer_index < object_view.auxiliary_layer_count());
        const input::Input snapshot = input::build_input(reinterpret_cast<storage_handle *>(&storage), print_view,
            {0, layer_index, true}, nullptr);
        if (z == object_view.layer(1).print_z()) {
            REQUIRE(native.id() == 0);
            CHECK_FALSE(snapshot.target->first_layer);
        } else if (z == object_view.layer(0).print_z()) {
            REQUIRE(native.id() != 0);
            CHECK(snapshot.target->first_layer);
        }
    }
    const Coverage coverage = check_print(print);
    CHECK(coverage.support > 0);
    CHECK(coverage.support_same_z > 0);
    CHECK(coverage.support_first > 0);
    CHECK(coverage.support_higher > 0);
    CHECK(coverage.support_duplicate_holes > 0);
    CHECK(coverage.support_lower_perimeters > 0);
}

void check_ring(const pw::Polygon &actual, const Polygon &expected)
{
    REQUIRE(actual.points.size() == expected.points.size());
    for (size_t i = 0; i < expected.points.size(); ++i) {
        CAPTURE(i);
        CHECK(actual.points[i].x() == expected.points[i].x());
        CHECK(actual.points[i].y() == expected.points[i].y());
    }
}

void check_polygons(const pw::ExPolygons &actual, const ExPolygons &expected)
{
    REQUIRE(actual.size() == expected.size());
    for (size_t i = 0; i < expected.size(); ++i) {
        CAPTURE(i);
        check_ring(actual[i].contour, expected[i].contour);
        REQUIRE(actual[i].holes.size() == expected[i].holes.size());
        for (size_t h = 0; h < expected[i].holes.size(); ++h)
            check_ring(actual[i].holes[h], expected[i].holes[h]);
    }
}

ExPolygons core_tops(const Layer &layer, const LayerRegion &region, Coverage &coverage)
{
    ExPolygons result;
    if (layer.get_property<LayerSupportProperty>() != nullptr)
        return result;
    for (const LayerSliceIsland &island : layer.islands())
        for (const LayerRegionIsland &group : island.regions_islands()) {
            if (group.regions().count(&region) == 0)
                continue;
            for (const Surface &surface : group.fill_surfaces()) {
                if (!surface.has_pos_top())
                    continue;
                if (group.regions().size() > 1) {
                    ++coverage.shared_top;
                    append(result, intersection_ex(ExPolygons{surface.expolygon}, region.get_raw_slices()));
                } else
                    result.push_back(surface.expolygon);
            }
        }
    return result;
}

void check_layer(const input::LayerInput &actual, const Layer &layer, Coverage &coverage)
{
    CHECK(actual.print_z == layer.scaled_print_z());
    CHECK(actual.support == (layer.get_property<LayerSupportProperty>() != nullptr));
    CHECK(actual.first_layer == (layer.scaled_print_z() == layer.object()->layers().front().scaled_print_z()));
    check_polygons(actual.lslices, layer.lslices());
    ExPolygons perimeter_slices;
    for (const LayerSliceIsland &island : layer.islands())
        append(perimeter_slices, island.get_perimeter_slices());
    check_polygons(actual.perimeter_slices, perimeter_slices);
    coord_t fallback = 0;
    const std::set<extruder_index_t> tools = layer.object()->object_extruders();
    REQUIRE(!tools.empty());
    for (extruder_index_t tool : tools)
        fallback += scale_i(layer.object()->print()->config().nozzle_diameter.get_at(tool));
    fallback /= tools.size();
    CHECK(actual.default_perimeter_spacing == fallback);
    REQUIRE(actual.regions.size() == layer.regions().size());
    size_t count = 0;
    coord_t spacing = 0;
    float width = 0.f;
    for (size_t i = 0; i < actual.regions.size(); ++i) {
        CAPTURE(i);
        const LayerRegion &region = layer.regions()[i];
        const input::RegionInput &copy = actual.regions[i];
        check_polygons(copy.raw_slices, region.get_raw_slices());
        check_polygons(copy.fill_expolygons, region.fill_expolygons());
        check_polygons(copy.top_surfaces, core_tops(layer, region, coverage));
        size_t islands = 0;
        for (const LayerSliceIsland &island : layer.islands())
            islands += island.regions().count(&region);
        coverage.single += islands == 1;
        coverage.multiple += islands > 1;
        coverage.empty_region += region.get_raw_slices().empty();
        const PrintRegionConfig &config = region.region().config();
        CHECK(copy.perimeter_extruder == config.perimeter_extruder.value);
        CHECK(copy.infill_extruder == config.infill_extruder.value);
        CHECK(copy.solid_infill_extruder == config.solid_infill_extruder.value);
        CHECK(copy.avoid_crossing_top == config.avoid_crossing_top.value);
        CHECK(copy.perimeters == config.perimeters.value);
        CHECK(copy.ironing == config.ironing.value);
        CHECK(copy.avoid_travel_island == config.avoid_travel_island.value);
        CHECK(copy.avoid_travel_island_weight == config.avoid_travel_island_weight.value);
        if (!region.get_raw_slices().empty()) {
            spacing += region.flow(frPerimeter).scaled_spacing();
            width += float(region.flow(frExternalPerimeter).scaled_width());
            ++count;
        }
    }
    CHECK(actual.perimeter_spacing == (count == 0 ? fallback : spacing / count));
    CHECK(actual.external_perimeter_width == (count == 0 ? float(fallback) : width / float(count)));
}

void check_external(const input::ExternalInput &actual, const Layer &layer, Coverage &coverage)
{
    const Print &print = *layer.object()->print();
    REQUIRE(actual.external_objects.size() == print.objects().size());
    coord_t sum = 0;
    size_t count = 0;
    for (size_t oi = 0; oi < print.objects().size(); ++oi) {
        const PrintObject &object = print.object(oi);
        const input::ExternalObjectInput &copy = actual.external_objects[oi];
        CHECK(copy.object == oi);
        Polygons holes;
        size_t same_z_holes = 0;
        if (const Layer *same = object.get_layer_at_printz(layer.scaled_print_z())) {
            for (const ExPolygon &slice : same->lslices())
                append(holes, slice.holes);
            same_z_holes = holes.size();
            for (const LayerRegion &region : same->regions())
                if (!region.get_raw_slices().empty()) {
                    sum += region.flow(frPerimeter).scaled_spacing();
                    ++count;
                }
        }
        if (layer.get_property<LayerSupportProperty>() != nullptr)
            if (const Layer *below = object.get_first_layer_below_printz(layer.scaled_print_z() + SCALED_EPSILON)) {
                for (const ExPolygon &slice : below->lslices())
                    append(holes, slice.holes);
                if (below->scaled_print_z() == layer.scaled_print_z() && same_z_holes != 0) {
                    ++coverage.support_duplicate_holes;
                    REQUIRE(holes.size() == 2 * same_z_holes);
                }
            }
        REQUIRE(copy.holes.size() == holes.size());
        for (size_t h = 0; h < holes.size(); ++h)
            check_ring(copy.holes[h], holes[h]);
        REQUIRE(copy.instance_shifts.size() == object.instances().size());
        for (size_t i = 0; i < copy.instance_shifts.size(); ++i) {
            CHECK(copy.instance_shifts[i].x() == object.instances()[i].shift.x());
            CHECK(copy.instance_shifts[i].y() == object.instances()[i].shift.y());
        }
    }
    coord_t fallback = 0;
    const std::set<extruder_index_t> tools = layer.object()->object_extruders();
    for (extruder_index_t tool : tools)
        fallback += scale_i(print.config().nozzle_diameter.get_at(tool));
    fallback /= tools.size();
    CHECK(actual.external_perimeter_spacing == (count == 0 ? fallback : sum / count));
}

Coverage check_print(const Print &print)
{
    Coverage coverage;
    const slic3r_api::Print view(reinterpret_cast<const print_handle *>(&print));
    for (uint32_t oi = 0; oi < view.object_count(); ++oi) {
        const slic3r_api::Object object = view.object(oi);
        const PrintObject &core_object = print.object(oi);
        for (uint32_t li = 0; li < object.layer_count(); ++li) {
            const coord_t z = object.layer(li).print_z();
            for (coord_t probe : {coord_t(0), coord_t(z - 1), z, coord_t(z + 1), coord_t(z + SCALED_EPSILON), coord_t(z + scale_i(10.))}) {
                const std::optional<uint32_t> same = input::layer_at_print_z(object, probe);
                const std::optional<uint32_t> below = input::first_layer_below_print_z(object, probe);
                CHECK((same ? &core_object.layers()[*same] : nullptr) == core_object.get_layer_at_printz(probe));
                CHECK((below ? &core_object.layers()[*below] : nullptr) == core_object.get_first_layer_below_printz(probe));
            }
        }
        for (bool auxiliary : {false, true}) {
            const uint32_t count = auxiliary ? object.auxiliary_layer_count() : object.layer_count();
            for (uint32_t li = 0; li < count; ++li) {
                CAPTURE(oi, li, auxiliary);
                input::Input snapshot;
                input::ExternalInput external;
                {
                    PluginStorage storage;
                    snapshot = input::build_input(reinterpret_cast<storage_handle *>(&storage), view, {oi, li, auxiliary}, nullptr);
                    external = input::build_external_input(view, {oi, li, auxiliary});
                }
                const Layer &layer = auxiliary ? core_object.auxiliary_layers()[li] : core_object.layers()[li];
                CHECK(snapshot.target->identity.object == oi);
                CHECK(snapshot.target->identity.layer == li);
                CHECK(snapshot.target->identity.auxiliary == auxiliary);
                check_layer(*snapshot.target, layer, coverage);
                check_external(external, layer, coverage);
                if (snapshot.target->support) {
                    ++coverage.support;
                    coverage.support_same_z += core_object.get_layer_at_printz(layer.scaled_print_z()) != nullptr;
                    const Layer *below = core_object.get_first_layer_below_printz(layer.scaled_print_z() + SCALED_EPSILON);
                    ExPolygons lower_slices;
                    if (below)
                        for (const LayerSliceIsland &island : below->islands())
                            append(lower_slices, island.get_perimeter_slices());
                    check_polygons(snapshot.lower_perimeter_slices, lower_slices);
                    coverage.support_lower_perimeters += !snapshot.lower_perimeter_slices.empty();
                    if (layer.scaled_print_z() == core_object.layers().front().scaled_print_z()) {
                        ++coverage.support_first;
                        CHECK(snapshot.target->first_layer);
                    } else if (layer.scaled_print_z() > core_object.layers().front().scaled_print_z()) {
                        ++coverage.support_higher;
                        CHECK_FALSE(snapshot.target->first_layer);
                    }
                } else
                    CHECK(snapshot.lower_perimeter_slices.empty());
                const input::LayerInput *address = snapshot.target.get();
                input::Input moved = std::move(snapshot);
                CHECK(moved.target.get() == address);
            }
        }
    }
    return coverage;
}

TriangleMesh ring_mesh(bool island)
{
    TriangleMesh mesh = make_cube(20., 4., 0.6);
    TriangleMesh top = make_cube(20., 4., 0.6);
    top.translate(Vec3f(0.f, 16.f, 0.f));
    mesh.merge(top);
    TriangleMesh left = make_cube(4., 12., 0.6);
    left.translate(Vec3f(0.f, 4.f, 0.f));
    mesh.merge(left);
    TriangleMesh right = make_cube(4., 12., 0.6);
    right.translate(Vec3f(16.f, 4.f, 0.f));
    mesh.merge(right);
    if (island) {
        TriangleMesh inner = make_cube(3., 3., 0.6);
        inner.translate(Vec3f(8.f, 8.f, 0.f));
        mesh.merge(inner);
    }
    return mesh;
}

} // namespace
