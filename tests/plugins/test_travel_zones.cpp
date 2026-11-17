#include <catch2/catch.hpp>

#include <algorithm>

#include "avoid_crossing_test_helpers.hpp"
#include "layer_extrusion_edit_transition_test_helpers.hpp"
#include "libslic3r/Plugins/Common/ClipperHelpers.hpp"
#include "libslic3r/Plugins/ElephantFootCompensation.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/AdvancedTravel/TravelZones.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/AdvancedTravel/DefaultZoneContributors.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/AvoidCrossing/LayerInputCache.hpp"
#include "libslic3r/GCode/AvoidCrossingPerimeters.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/PrintObject.hpp"
#include "test_data.hpp"

/*
Travel-zone tests give contributors small owned rectangles and query their
dated, scoped unions without a production consumer. A direct connector check
uses the shared transition helpers to publish real scopes and entry state, then
checks pair owners before apply. All views borrow locally owned prints or PEs.
Sliced frames and separate islands compare raw published slices with the shared
late withdrawal utility and excluded tops with the core boundary cache, including
shifted instances and source-or-target dates on the original slices.
Printed tops are compared with that same core geometry and dated by the last
arrival in each containing original island; removing all such arrivals omits it.
The tools are simulated by alternating tool ids on a single-extruder slice;
the support target is simulated by marking a sliced top layer as support.
Late-withdrawal checks slice a dumbbell, a square and disjoint squares, compare
their boxes and surfaces with independent references, and retain a traversable
bridge through the result.

    rectangle: build a counter-clockwise work area
    ZoneTestInput: keep borrowed contributor input and polygon storage alive
    FixedContributor::contribute: append synthetic areas and count calls
    tests: collect eagerly, query lazy attractor unions, and retain real ordered pairs
    sliced default-zone test: slice frames, populate core caches, compare copied zones
        `-- check_shifted_area: compare contour and hole vertices in original order
*/

namespace {
namespace zones = slic3r_api::LayerExtrusionEdit::AdvancedTravel;
namespace travel = slic3r_api::LayerExtrusionEdit::TravelConnection;
namespace helpers = Slic3r::Test::TransitionPipeline;
namespace work = slic3r_api::PluginCommon::polygon_work;
namespace ac = slic3r_api::LayerExtrusionEdit::AvoidCrossing;

// Convert millimeter dimensions into one owned counter-clockwise rectangle; no API storage is borrowed.
work::ExPolygons rectangle(coord_t x, coord_t width);
// Compare every contour and hole vertex with the core area after the target instance shift.
void check_shifted_area(const work::ExPolygons &actual, const Slic3r::ExPolygons &expected, c_point shift);
// Compare every contour and hole vertex with an owned raw source slice after the instance shift.
void check_shifted_work_area(const work::ExPolygons &actual, const work::ExPolygon &expected, c_point shift);

struct ZoneTestInput
{
    Slic3r::Print print;
    Slic3r::PluginStorage storage;
    plugin_run_context context{};
    slic3r_api::Print print_view{reinterpret_cast<const print_handle *>(&print)};
    slic3r_api::PrintingPlan plan_view;
    slic3r_api::LayerExtrusionEdit::AvoidCrossing::LayerInputCache cache;
    travel::LayerTravelChronology chronology;
    size_t checks = 0;
    size_t cancel_at = 0;

    // Initialize host geometry operations and cancellation probes with locally owned storage and print views.
    ZoneTestInput()
    {
        Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
        plan_view = slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(
            &print.mutable_printing_plan()));
        context.plugin_storage = reinterpret_cast<storage_handle *>(&storage);
        context.host_context = reinterpret_cast<plugin_host_context *>(this);
        context.is_cancelled = [](plugin_host_context *host) {
            ZoneTestInput &self = *reinterpret_cast<ZoneTestInput *>(host);
            ++self.checks;
            return int(self.cancel_at != 0 && self.checks >= self.cancel_at);
        };
    }

    // Return borrowed input valid while this local owner remains alive.
    zones::ZoneContributorInput input()
    {
        return {print_view, plan_view, context, Slic3r::scale_i(0.2), cache,
                chronology.travels};
    }
};

struct FixedContributor : zones::ZoneContributor
{
    std::vector<zones::TravelZone> zones;
    mutable size_t calls = 0;

    // Copy owned test areas to the output and record the eager invocation; input is not retained.
    void contribute(const zones::ZoneContributorInput &, std::vector<zones::TravelZone> &out) const override
    {
        ++calls;
        out.insert(out.end(), zones.begin(), zones.end());
    }
};

work::ExPolygons rectangle(const coord_t x, const coord_t width)
{
    work::ExPolygon area;
    const coord_t left = Slic3r::scale_i(double(x));
    const coord_t right = Slic3r::scale_i(double(x + width));
    const coord_t top = Slic3r::scale_i(100.);
    area.contour.points = {{left, 0}, {right, 0}, {right, top}, {left, top}};
    return {area};
}

void check_shifted_area(const work::ExPolygons &actual, const Slic3r::ExPolygons &expected, const c_point shift)
{
    REQUIRE(actual.size() == expected.size());
    for (size_t i = 0; i < actual.size(); ++i) {
        REQUIRE(actual[i].holes.size() == expected[i].holes.size());
        for (size_t h = 0; h <= actual[i].holes.size(); ++h) {
            const work::Polygon &a = h == 0 ? actual[i].contour : actual[i].holes[h - 1];
            const Slic3r::Polygon &e = h == 0 ? expected[i].contour : expected[i].holes[h - 1];
            REQUIRE(a.size() == e.size());
            for (size_t p = 0; p < a.size(); ++p) {
                CHECK(a[p].x() == e[p].x() + shift.x);
                CHECK(a[p].y() == e[p].y() + shift.y);
            }
        }
    }
}

void check_shifted_work_area(const work::ExPolygons &actual, const work::ExPolygon &expected, const c_point shift)
{
    REQUIRE(actual.size() == 1);
    REQUIRE(actual.front().holes.size() == expected.holes.size());
    const auto check_ring = [shift](const work::Polygon &actual_ring, const work::Polygon &expected_ring) {
        REQUIRE(actual_ring.size() == expected_ring.size());
        for (size_t point = 0; point < actual_ring.size(); ++point) {
            CHECK(actual_ring[point].x() == expected_ring[point].x() + shift.x);
            CHECK(actual_ring[point].y() == expected_ring[point].y() + shift.y);
        }
    };
    check_ring(actual.front().contour, expected.contour);
    for (size_t hole = 0; hole < expected.holes.size(); ++hole)
        check_ring(actual.front().holes[hole], expected.holes[hole]);
}

// Publish raw source slices and check the shared late withdrawal keeps one usable result per slice.
TEST_CASE("Attractor zones publish raw slices and late withdrawal keeps thin bridges", "[plugins][travel-zones]")
{
    namespace clipper = slic3r_api::PluginCommon::clipper;
    namespace ac = slic3r_api::LayerExtrusionEdit::AvoidCrossing;
    const int scene = GENERATE(0, 1, 2);
    const bool bridge = scene == 1;
    ZoneTestInput owner;
    Slic3r::DynamicPrintConfig config = Slic3r::DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "1"}, {"fill_density", "0%"}, {"skirts", "0"}, {"brim_width", "0"}});
    Slic3r::Model model;
    Slic3r::ModelObject *object = model.add_object();
    Slic3r::TriangleMesh mesh = bridge ? Slic3r::Test::Plugins::dumbbell(0.4) : Slic3r::make_cube(20., 20., 0.6);
    if (scene == 2) {
        Slic3r::TriangleMesh second = Slic3r::make_cube(20., 20., 0.6);
        second.translate(30., 0., 0.);
        mesh.merge(second);
    }
    object->add_volume(mesh, Slic3r::ModelVolumeType::MODEL_PART, false);
    object->add_instance(); object->ensure_on_bed();
    owner.print.apply(model, config); owner.print.set_status_silent(); owner.print.process();
    const ac::Input input = ac::build_input(owner.context.plugin_storage, owner.print_view, {0, 0, false});
    const ac::LayerInput &layer = *input.target;
    REQUIRE(layer.perimeter_slices.size() == (scene == 2 ? 2 : 1));
    const work::ExPolygons &raw = layer.perimeter_slices;
    work::ExPolygons withdrawn;
    const double minimum_width = 0.01;
    const coord_t withdrawal = coord_t(1.5 * double(layer.perimeter_spacing));
    for (const work::ExPolygon &slice : raw) {
        const slic3r_api::StoredExPolygonCollection stored = clipper::to_stored_expolygons(
            owner.context.plugin_storage, {slice});
        const slic3r_api::ExPolygon source = stored.readonly().at(0);
        const slic3r_api::StoredExPolygonCollection result =
            slic3r_api::XYSizeCompensationPlugin::detail::elephant_foot_compensation(
            owner.context.plugin_storage, source, minimum_width, withdrawal);
        const work::ExPolygons copied = clipper::from_expolygon_collection_view(result.readonly());
        REQUIRE(copied.size() == 1);
        REQUIRE_FALSE(copied.front().contour.points.empty());
        withdrawn.push_back(copied.front());
    }
    const work::ExPolygons domain = clipper::union_ex(owner.context.plugin_storage, withdrawn);
    REQUIRE(domain.size() == (scene == 2 ? 2 : 1));
    const double mm2 = double(Slic3r::scale_i(1.)) * Slic3r::scale_i(1.);
    double original_area_mm2 = 0.;
    for (const work::ExPolygon &part : raw) original_area_mm2 += part.area() / mm2;
    double domain_area_mm2 = 0.;
    for (const work::ExPolygon &part : domain) domain_area_mm2 += part.area() / mm2;
    CAPTURE(scene, original_area_mm2, domain_area_mm2);
    if (bridge) {
        const Slic3r::Point minimum = Slic3r::get_extents(owner.print.object(0).layers().front().lslices()).min;
        const work::Point middle{minimum.x() + Slic3r::scale_i(12.5), minimum.y() + Slic3r::scale_i(5.)};
        CHECK(domain.front().contains(middle));
        CHECK(domain_area_mm2 < original_area_mm2);
        // The 0.4 mm neck is narrowed to the 0.01 mm minimum-width guard, not removed.
        const coord_t inside_half_width = Slic3r::scale_i(0.004);
        const coord_t outside_half_width = Slic3r::scale_i(0.006);
        CHECK(domain.front().contains({middle.x(), middle.y() - inside_half_width}, true));
        CHECK(domain.front().contains({middle.x(), middle.y() + inside_half_width}, true));
        CHECK_FALSE(domain.front().contains({middle.x(), middle.y() - outside_half_width}, true));
        CHECK_FALSE(domain.front().contains({middle.x(), middle.y() + outside_half_width}, true));
        work::Polyline through_bridge;
        through_bridge.points = {{minimum.x() + Slic3r::scale_i(5.), middle.y()},
                                 {minimum.x() + Slic3r::scale_i(20.), middle.y()}};
        CHECK(clipper::diff_pl(owner.context.plugin_storage, {through_bridge}, domain).empty());
    } else {
        double expected_area_mm2 = 0.;
        for (const work::ExPolygon &slice : raw) {
            const auto box = slice.contour.bounding_box();
            const double width = Slic3r::unscaled(box.max.x() - box.min.x()) - 2. * Slic3r::unscaled(withdrawal);
            const double height = Slic3r::unscaled(box.max.y() - box.min.y()) - 2. * Slic3r::unscaled(withdrawal);
            REQUIRE(width > 0.);
            REQUIRE(height > 0.);
            expected_area_mm2 += width * height;
        }
        // Four integer-grid edge roundings and the resampled contour stay below this 0.05 mm² margin.
        CHECK(domain_area_mm2 == Approx(expected_area_mm2).margin(0.05));
        if (scene == 0) {
            const auto box = raw.front().contour.bounding_box();
            const auto actual = domain.front().contour.bounding_box();
            CHECK(actual.min.x() == Approx(box.min.x() + withdrawal).margin(2.));
            CHECK(actual.min.y() == Approx(box.min.y() + withdrawal).margin(2.));
            CHECK(actual.max.x() == Approx(box.max.x() - withdrawal).margin(2.));
            CHECK(actual.max.y() == Approx(box.max.y() - withdrawal).margin(2.));
        }
    }
    REQUIRE(domain.size() == withdrawn.size());
    for (size_t component = 0; component < domain.size(); ++component) {
        CHECK(clipper::diff_ex(owner.context.plugin_storage, {domain[component]}, {withdrawn[component]}).empty());
        CHECK(clipper::diff_ex(owner.context.plugin_storage, {withdrawn[component]}, {domain[component]}).empty());
        CHECK(domain[component].area() == Approx(withdrawn[component].area()));
    }

    Slic3r::Layer &source_layer = owner.print.object(0).layers().front();
    const slic3r_api::Layer layer_view(reinterpret_cast<const layer_handle *>(&source_layer));
    owner.chronology.travels.clear();
    for (size_t instance = 0; instance < 1; ++instance) {
        const uint32_t catalog = owner.print.mutable_printing_plan().find_or_add_object_instance(owner.print.object(0), instance);
        const c_point shift = owner.print_view.object(0).instance_shift(instance);
        for (size_t slice = 0; slice < raw.size(); ++slice) {
            travel::PendingTravel pending;
            pending.target.printing_catalog_id = catalog;
            pending.target.region_island = layer_view.island(slice).get_or_create_full_region_island(0);
            pending.target.position = {raw[slice].contour.points.front().x() + shift.x,
                                       raw[slice].contour.points.front().y() + shift.y, layer_view.print_z()};
            owner.chronology.travels.push_back(pending);
        }
    }
    owner.plan_view = slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(
        &owner.print.mutable_printing_plan()));
    zones::DefaultAttractorContributor attractors;
    std::vector<zones::TravelZone> published;
    attractors.contribute(owner.input(), published);
    REQUIRE(published.size() == raw.size());
    for (size_t instance = 0; instance < 1; ++instance) {
        const uint32_t catalog = owner.chronology.travels[instance * raw.size()].target.printing_catalog_id;
        const c_point shift = owner.print_view.object(0).instance_shift(instance);
        for (size_t slice = 0; slice < raw.size(); ++slice) {
            const zones::TravelZone &zone = published[instance * raw.size() + slice];
            CHECK(zone.scope.printing_catalog_id == catalog);
            CHECK(zone.source_layer == reinterpret_cast<const layer_handle *>(&source_layer));
            CHECK(zone.begin <= zone.end);
            check_shifted_work_area(zone.area, raw[slice], shift);
        }
    }
}

// Slice two islands and two instances, then compare source-or-target dates and unchanged core top geometry.
TEST_CASE("Default travel zones publish raw slices and excluded tops", "[plugins][travel-zones]")
{
    const bool tops = GENERATE(false, true);
    const bool support = GENERATE(false, true);
    CAPTURE(tops, support);
    ZoneTestInput owner;
    Slic3r::DynamicPrintConfig config = Slic3r::DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "2"}, {"fill_density", "20%"}, {"top_solid_layers", "1"},
        {"bottom_solid_layers", "1"}, {"skirts", "0"}, {"brim_width", "0"},
        {"gap_fill_enabled", "0"}, {"thin_walls", "0"}, {"thin_perimeters", "0"},
        {"avoid_crossing_top", tops ? "1" : "0"}});
    Slic3r::TriangleMesh mesh = Slic3r::make_cube(20., 4., 0.6);
    Slic3r::TriangleMesh bar = Slic3r::make_cube(20., 4., 0.6);
    bar.translate(0., 16., 0.); mesh.merge(bar);
    bar = Slic3r::make_cube(4., 12., 0.6);
    bar.translate(0., 4., 0.); mesh.merge(bar);
    bar.translate(16., 0., 0.); mesh.merge(bar);
    bar = Slic3r::make_cube(5., 5., 0.6);
    bar.translate(25., 0., 0.); mesh.merge(bar);
    Slic3r::Model model;
    Slic3r::ModelObject *object = model.add_object();
    object->add_volume(mesh, Slic3r::ModelVolumeType::MODEL_PART, false);
    object->add_instance();
    object->add_instance()->set_offset(Slic3r::Vec3d(60., 30., 0.));
    object->ensure_on_bed();
    owner.print.apply(model, config);
    owner.print.set_status_silent();
    owner.print.process();
    owner.plan_view = slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(
        &owner.print.mutable_printing_plan()));
    Slic3r::Layer &layer = owner.print.object(0).layers().back();
    if (support) layer.get_or_add_property<Slic3r::LayerSupportProperty>();
    const slic3r_api::Layer layer_view(reinterpret_cast<const layer_handle *>(&layer));
    REQUIRE(layer_view.island_count() == 2);
    Slic3r::AvoidCrossingPerimeters oracle;
    oracle.init_layer(layer);
    const Slic3r::Point from = layer.lslices().front().contour.points.front();
    const Slic3r::Point to = layer.lslices().back().contour.points.front();
    oracle.travel_to({layer, from, Slic3r::Point::Zero(), extruder_index_t(0), 0., false}, to);
    const Slic3r::AvoidCrossingPerimeters::Boundary &expected = oracle.internal_boundary();
    REQUIRE(expected.boundary_growth.size() == (support ? 4 : 2));
    REQUIRE(expected.to_avoid.empty() == (!tops || support));
    REQUIRE_FALSE(expected.boundary_growth.front().first.holes.empty());
    std::vector<c_point> shifts;
    // The source keeps the first slice active through its departure; the invalid travel ends both runs.
    for (size_t instance = 0; instance < 2; ++instance) {
        const uint32_t id = owner.print.mutable_printing_plan().find_or_add_object_instance(owner.print.object(0), instance);
        const c_point shift = owner.print_view.object(0).instance_shift(instance);
        shifts.push_back(shift);
        for (size_t island : {size_t(0), size_t(0), size_t(1), size_t(0)}) {
            travel::PendingTravel pending;
            // Tops cases build the boundary with tool 1 rather than the oracle's tool 0 to check tool independence.
            pending.extruder_id = extruder_index_t((owner.chronology.travels.size() + size_t(tops)) % 2);
            pending.target.printing_catalog_id = id;
            pending.target.region_island = layer_view.island(island).get_or_create_full_region_island(0);
            // Target a hole border to check that island membership includes its border.
            const Slic3r::Point point = island == 0 && owner.chronology.travels.size() % 5 == 1 ?
                expected.boundary_growth[0].first.holes.front().points.front() :
                expected.boundary_growth[island].first.contour.points.front();
            pending.target.position = {point.x() + shift.x, point.y() + shift.y, layer_view.print_z()};
            if (!owner.chronology.travels.empty()) pending.source = owner.chronology.travels.back().target;
            owner.chronology.travels.push_back(pending);
        }
        owner.chronology.travels.emplace_back();
    }
    zones::DefaultAttractorContributor attractors;
    zones::DefaultForbiddenTopsContributor forbidden;
    std::vector<zones::TravelZone> masks, excluded;
    attractors.contribute(owner.input(), masks);
    forbidden.contribute(owner.input(), excluded);
    zones::PrintedTopsContributor printed;
    std::vector<zones::TravelZone> dated;
    printed.contribute(owner.input(), dated);
    REQUIRE(dated.size() == excluded.size());
    for (size_t z = 0; z < dated.size(); ++z) {
        const zones::TravelZone &zone = dated[z];
        const size_t instance = z / expected.to_avoid.size();
        const Slic3r::ExPolygon &top = expected.to_avoid[z % expected.to_avoid.size()];
        size_t last = SIZE_MAX;
        for (const std::pair<Slic3r::ExPolygon, Slic3r::ExPolygon> &growth : expected.boundary_growth) {
            if (!growth.first.contains(top.contour.points.front())) continue;
            for (size_t t = 0; t < owner.chronology.travels.size(); ++t) {
                const travel::TravelEndpoint &target = owner.chronology.travels[t].target;
                if (target.printing_catalog_id == zone.scope.printing_catalog_id && target.region_island.valid() &&
                    growth.first.contains(Slic3r::Point(target.position.x - shifts[instance].x,
                                                      target.position.y - shifts[instance].y))) last = t;
            }
            break;
        }
        REQUIRE(last != SIZE_MAX);
        CHECK(zone.begin == last + 1);
        CHECK(zone.end == SIZE_MAX);
        CHECK(zone.role == zones::ZoneRole::Forbidden);
        CHECK(zone.scope.extruder == EXTRUDER_ID_UNKNOWN);
        CHECK_FALSE(zone.scope.support_travels_only);
        check_shifted_area(zone.area, {top}, shifts[instance]);
    }
    // Keep valid layer/instance targets but move them outside every original island.
    const std::vector<travel::PendingTravel> saved = owner.chronology.travels;
    for (travel::PendingTravel &pending : owner.chronology.travels)
        pending.target.position.x += Slic3r::scale_i(1000.);
    std::vector<zones::TravelZone> absent;
    printed.contribute(owner.input(), absent);
    CHECK(absent.empty());
    owner.chronology.travels = saved;
    REQUIRE(masks.size() == (support ? 8 : 4));
    REQUIRE(excluded.size() == 2 * expected.to_avoid.size());
    for (const zones::TravelZone &zone : masks) {
        CHECK(zone.role == zones::ZoneRole::Attractor);
        CHECK_FALSE(zone.scope.support_travels_only);
    }
    for (const zones::TravelZone &zone : excluded) {
        CHECK(zone.role == zones::ZoneRole::Forbidden);
        CHECK_FALSE(zone.scope.support_travels_only);
    }
    for (size_t instance = 0; instance < 2; ++instance) {
        const uint32_t id = owner.chronology.travels[instance * 5].target.printing_catalog_id;
        const ac::Input &copied = owner.cache.get(owner.print_view, layer_view, &owner.context);
        work::ExPolygons slices = copied.target->perimeter_slices;
        if (support) slices.insert(slices.end(), copied.lower_perimeter_slices.begin(), copied.lower_perimeter_slices.end());
        for (size_t slice = 0; slice < slices.size(); ++slice) {
            const zones::TravelZone &zone = masks[instance * slices.size() + slice];
            CHECK(zone.scope.extruder == EXTRUDER_ID_UNKNOWN);
            CHECK(zone.scope.printing_catalog_id == id);
            CHECK_FALSE(zone.scope.support_travels_only);
            const travel::TravelEndpoint &first_target = owner.chronology.travels[instance * 5].target;
            const bool first = slices[slice].contains({first_target.position.x - shifts[instance].x,
                                                      first_target.position.y - shifts[instance].y});
            CHECK(zone.begin == instance * 5 + (first ? 0 : 2));
            CHECK(zone.end == instance * 5 + 4);
            work::ExPolygons domain{slices[slice]};
            work::ExPolygons local = zone.area;
            for (work::ExPolygon &area : local) {
                for (work::Point &point : area.contour.points) point -= work::Point(shifts[instance]);
                for (work::Polygon &hole : area.holes)
                    for (work::Point &point : hole.points) point -= work::Point(shifts[instance]);
            }
            CHECK(slic3r_api::PluginCommon::clipper::diff_ex(owner.context.plugin_storage, local, domain).empty());
            CHECK(slic3r_api::PluginCommon::clipper::diff_ex(owner.context.plugin_storage, domain, local).empty());
        }
        if (!excluded.empty()) {
            for (size_t top = 0; top < expected.to_avoid.size(); ++top) {
                const zones::TravelZone &zone = excluded[instance * expected.to_avoid.size() + top];
                CHECK(zone.begin == 0);
                CHECK(zone.end == SIZE_MAX);
                CHECK(zone.scope.extruder == EXTRUDER_ID_UNKNOWN);
                CHECK(zone.scope.printing_catalog_id == id);
                check_shifted_area(zone.area, {expected.to_avoid[top]}, shifts[instance]);
            }
        }
    }
    zones::TravelZoneView view({std::cref(attractors)}, owner.input());
    for (size_t t = 0; t < owner.chronology.travels.size(); ++t) {
        const travel::PendingTravel &pending = owner.chronology.travels[t];
        const zones::ActiveZones &active = view.query(t, pending);
        if (t % 5 == 4) { CHECK(active.attractors.empty()); continue; }
        REQUIRE(active.attractors.size() == (t % 5 < 2 ? 1 : 2));
        // Dates use the original slice borders, and every active index remains available to the planner.
        REQUIRE_FALSE(active.attractor_indexes.empty());
        for (const size_t index : active.attractor_indexes)
            CHECK_FALSE(view.zone_area(index).empty());
        const work::Point endpoint(pending.target.position.x, pending.target.position.y);
        CHECK(std::any_of(active.attractor_indexes.begin(), active.attractor_indexes.end(),
            [&view, &endpoint](const size_t index) {
                return std::any_of(view.zone_area(index).begin(), view.zone_area(index).end(),
                    [&endpoint](const work::ExPolygon &area) { return area.contains(endpoint, true); });
            }));
    }
    owner.cancel_at = owner.checks + 1;
    CHECK_THROWS_AS(attractors.contribute(owner.input(), masks), slic3r_api::PluginCancelled);
    CHECK_THROWS_AS(forbidden.contribute(owner.input(), excluded), slic3r_api::PluginCancelled);
    // Collection remains cancellable before dating, and the raw-slice copy has no hidden withdrawal phase.
    for (size_t delay : {size_t(2), size_t(3)}) {
        owner.cancel_at = owner.checks + delay;
        CHECK_THROWS_AS(attractors.contribute(owner.input(), masks), slic3r_api::PluginCancelled);
    }
    owner.cancel_at = owner.checks + 2;
    CHECK_THROWS_AS(forbidden.contribute(owner.input(), excluded), slic3r_api::PluginCancelled);
}

TEST_CASE("Travel zones collect eagerly and cache dated applicable sets", "[plugins][travel-zones]")
{
    ZoneTestInput owner;
    FixedContributor first;
    first.zones.push_back({zones::ZoneRole::Attractor, rectangle(0, 100), 1, 3, {}});
    first.zones.push_back({zones::ZoneRole::Attractor, rectangle(50, 100), 1, 3, {}});
    FixedContributor second;
    second.zones.push_back({zones::ZoneRole::Forbidden, rectangle(75, 25), 2, 4, {}});
    CHECK(first.calls == 0);
    zones::TravelZoneView view({std::cref(first), std::cref(second)}, owner.input());
    CHECK(first.calls == 1);
    CHECK(second.calls == 1);
    CHECK(view.contributor_calls() == 2);
    CHECK(view.union_builds() == 0);
    travel::PendingTravel pending;
    const zones::ActiveZones &empty = view.query(0, pending);
    CHECK(empty.attractors.empty());
    const zones::ActiveZones &attracted = view.query(1, pending);
    REQUIRE(attracted.attractors.size() == 1);
    CHECK(attracted.attractors.front().area() == Approx(15000. * Slic3r::scale_i(1.) * Slic3r::scale_i(1.)));
    CHECK(attracted.forbidden_indexes.empty());
    CHECK(attracted.id != empty.id);
    CHECK(&view.query(1, pending) == &attracted);
    CHECK(view.union_builds() == 1);
    const zones::ActiveZones &overlap = view.query(2, pending);
    CHECK(overlap.id != attracted.id);
    CHECK(overlap.attractors.front().area() == Approx(15000. * Slic3r::scale_i(1.) * Slic3r::scale_i(1.)));
    REQUIRE(overlap.forbidden_indexes.size() == 1);
    CHECK(view.zone_area(overlap.forbidden_indexes.front()).front().area() ==
          Approx(2500. * Slic3r::scale_i(1.) * Slic3r::scale_i(1.)));
    const zones::ActiveZones &ended = view.query(3, pending);
    CHECK(ended.attractors.empty());
    CHECK_FALSE(ended.forbidden_indexes.empty());
    CHECK(&view.query(4, pending) == &empty);
    CHECK(view.union_builds() == 2);
    CHECK_THROWS_AS(view.query(3, pending), std::logic_error);
    CHECK(first.calls == 1);
    CHECK(second.calls == 1);
}

TEST_CASE("Travel zone scope changes ids without a date event", "[plugins][travel-zones]")
{
    ZoneTestInput owner;
    FixedContributor contributor;
    contributor.zones.push_back({zones::ZoneRole::Attractor, rectangle(0, 100), 0, SIZE_MAX,
                                 {1, 42, true}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    Slic3r::Printing::PrintingExtrusion support;
    support.sregion_island_role = Slic3r::ExtrusionRole::SupportMaterial;
    travel::PendingTravel pending;
    pending.extruder_id = 1;
    pending.source.printing_catalog_id = 42;
    pending.source_extrusion = slic3r_api::PrintingExtrusion(
        reinterpret_cast<printing_extrusion_handle *>(&support));
    const zones::ActiveZones &matching = view.query(0, pending);
    REQUIRE_FALSE(matching.attractors.empty());
    pending.extruder_id = 0;
    const zones::ActiveZones &not_matching = view.query(1, pending);
    CHECK(not_matching.attractors.empty());
    CHECK(not_matching.id != matching.id);
    pending.extruder_id = 1;
    pending.source.printing_catalog_id = 8;
    CHECK(&view.query(1, pending) == &not_matching);
    pending.target.printing_catalog_id = 42;
    CHECK(&view.query(1, pending) == &matching);
    pending.source_extrusion = slic3r_api::PrintingExtrusion(nullptr);
    CHECK(&view.query(1, pending) == &not_matching);
    pending.target_extrusion = slic3r_api::PrintingExtrusion(
        reinterpret_cast<printing_extrusion_handle *>(&support));
    CHECK(&view.query(1, pending) == &matching);
    CHECK(view.union_builds() == 1);
}

TEST_CASE("Travel zones discard empty areas and reject reversed dates", "[plugins][travel-zones]")
{
    ZoneTestInput owner;
    FixedContributor contributor;
    contributor.zones.push_back({zones::ZoneRole::Attractor, {}, 0, SIZE_MAX, {}});
    contributor.zones.push_back({zones::ZoneRole::Attractor, {work::ExPolygon{}}, 0, SIZE_MAX, {}});
    contributor.zones.push_back({zones::ZoneRole::Attractor, rectangle(0, 100), 1, 1, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    travel::PendingTravel pending;
    const zones::ActiveZones &empty = view.query(0, pending);
    CHECK(empty.attractors.empty());
    CHECK(&view.query(1, pending) == &empty);
    CHECK(view.union_builds() == 0);
    contributor.zones.push_back({zones::ZoneRole::Attractor, rectangle(0, 100), 3, 2, {}});
    CHECK_THROWS_AS(zones::TravelZoneView({std::cref(contributor)}, owner.input()), std::invalid_argument);
}

TEST_CASE("Travel zones propagate collection and union cancellation", "[plugins][travel-zones]")
{
    ZoneTestInput owner;
    FixedContributor contributor;
    contributor.zones.push_back({zones::ZoneRole::Attractor, rectangle(0, 100), 0, SIZE_MAX, {}});
    contributor.zones.push_back({zones::ZoneRole::Forbidden, rectangle(50, 100), 0, SIZE_MAX, {}});
    SECTION("Before collection") {
        owner.cancel_at = 1;
        CHECK_THROWS_AS(zones::TravelZoneView({std::cref(contributor)}, owner.input()), slic3r_api::PluginCancelled);
        CHECK(contributor.calls == 0);
    }
    SECTION("Between contributors") {
        owner.cancel_at = 2;
        CHECK_THROWS_AS(zones::TravelZoneView({std::cref(contributor), std::cref(contributor)}, owner.input()),
                        slic3r_api::PluginCancelled);
        CHECK(contributor.calls == 1);
    }
    SECTION("During attractor union without publishing a partial cache") {
        zones::TravelZoneView view({std::cref(contributor)}, owner.input());
        owner.cancel_at = owner.checks + 5;
        travel::PendingTravel pending;
        CHECK_THROWS_AS(view.query(0, pending), slic3r_api::PluginCancelled);
        CHECK(view.union_builds() == 1);
        owner.cancel_at = 0;
        const zones::ActiveZones &active = view.query(0, pending);
        CHECK_FALSE(active.attractors.empty());
        CHECK_FALSE(active.forbidden_indexes.empty());
        CHECK(view.union_builds() == 2);
        owner.cancel_at = owner.checks + 1;
        CHECK_THROWS_AS(view.query(0, pending), slic3r_api::PluginCancelled);
    }
}

TEST_CASE("Travel chronology retains continuous and materialized printing owners and tools",
          "[plugins][travel-zones]")
{
    ZoneTestInput owner;
    Slic3r::Printing::PrintingPlan &plan = owner.print.mutable_printing_plan();
    plan.groups.resize(1);
    plan.groups[0].layers.resize(1);
    Slic3r::Printing::PrintingLayerGroup &layer = plan.groups[0].layers[0];
    layer.print_z = Slic3r::scale_i(0.2);
    layer.tool_groups.resize(2);
    layer.tool_groups[0].extruder_id = 0;
    layer.tool_groups[1].extruder_id = 1;
    helpers::append_printing_extrusion(layer.tool_groups[0], helpers::make_print_path(0, 1));
    helpers::append_printing_extrusion(layer.tool_groups[0], helpers::make_print_path(3, 4));
    helpers::append_printing_extrusion(layer.tool_groups[0], helpers::make_print_path(4, 5));
    helpers::append_printing_extrusion(layer.tool_groups[0], helpers::make_print_path(7, 8));
    std::unique_ptr<Slic3r::ExtrusionEntityCollection> multi_scope =
        std::make_unique<Slic3r::ExtrusionEntityCollection>(false, false);
    multi_scope->append(helpers::make_print_path(10, 11));
    multi_scope->append(helpers::make_print_path(13, 14));
    helpers::append_printing_extrusion(layer.tool_groups[1], std::move(multi_scope));
    helpers::run_layer_plugins(owner.print,
        {helpers::TRANSITION_SCOPE_PLUGIN, "layer_extrusion_edit.entry_state.default"});
    travel::ScopeTravelConnector connector(reinterpret_cast<orchestrator_handle *>(&Slic3r::Orchestrator::instance()));
    slic3r_api::PluginProgress progress;
    progress.reset(nullptr, 1, nullptr);
    connector.setup(slic3r_api::ConstPrintingPlan(owner.plan_view), 1);
    slic3r_api::PrintingLayerGroup layer_view(reinterpret_cast<printing_layer_group_handle *>(&layer));
    connector.setup_run(layer_view, 0, 0, progress);
    slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope stored(
        helpers::find_scope(helpers::entity_view(*layer.tool_groups[0].extrusions[3].root)), helpers::scope_property_key());
    REQUIRE(stored.travel().set_points(std::vector<c_point>{{Slic3r::scale_i(5.), 0}, {Slic3r::scale_i(7.), 0}}));
    stored.travel().get_or_add(slic3r_api::EPropertyAttributes::key).extrusion_role(RAW_EXTRUSION_ROLE_TRAVEL);
    travel::LayerTravelChronology chronology = connector.prepare(layer_view, 0, 0);
    REQUIRE(chronology.travels.size() == 3);
    REQUIRE(chronology.pair_extrusions.size() == 6);
    CHECK(chronology.travels[0].pair_index == 1);
    CHECK(chronology.travels[1].pair_index == 4);
    CHECK(chronology.travels[2].pair_index == 5);
    const std::vector<const printing_extrusion_handle *> expected_owners{
        reinterpret_cast<const printing_extrusion_handle *>(&layer.tool_groups[0].extrusions[0]),
        reinterpret_cast<const printing_extrusion_handle *>(&layer.tool_groups[0].extrusions[1]),
        reinterpret_cast<const printing_extrusion_handle *>(&layer.tool_groups[0].extrusions[2]),
        reinterpret_cast<const printing_extrusion_handle *>(&layer.tool_groups[0].extrusions[3]),
        reinterpret_cast<const printing_extrusion_handle *>(&layer.tool_groups[1].extrusions[0])};
    const std::vector<size_t> expected_pair_owners{0, 1, 2, 3, 4, 4};
    for (size_t index = 0; index < expected_pair_owners.size(); ++index) {
        CHECK(chronology.pair_extrusions[index].target.handle() == expected_owners[expected_pair_owners[index]]);
        CHECK(chronology.pair_extrusions[index].extruder_id == (index < 4 ? 0 : 1));
    }
    CHECK(chronology.travels[0].source_extrusion.handle() == expected_owners[0]);
    CHECK(chronology.travels[0].target_extrusion.handle() == expected_owners[1]);
    CHECK(chronology.travels[1].source_extrusion.handle() == expected_owners[3]);
    CHECK(chronology.travels[1].target_extrusion.handle() == expected_owners[4]);
    CHECK(chronology.travels[2].source_extrusion.handle() == expected_owners[4]);
    CHECK(chronology.travels[2].target_extrusion.handle() == expected_owners[4]);
    CHECK(chronology.travels[0].extruder_id == 0);
    CHECK(chronology.travels[1].extruder_id == 1);
    CHECK(chronology.travels[2].extruder_id == 1);
}

} // namespace
