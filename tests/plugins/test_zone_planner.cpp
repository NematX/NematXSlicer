#include <catch2/catch.hpp>
#include <algorithm>
#include <limits>
#include <utility>

#include "layer_extrusion_edit_transition_test_helpers.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/AdvancedTravel/ZonePlanner.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/AdvancedTravel/DefaultZoneContributors.hpp"
#include "libslic3r/Plugins/Common/ClipperHelpers.hpp"
#include "libslic3r/Plugins/ElephantFootCompensation.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Api/host/ApiHostUtils.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Steps/StepExtrusionOrdering.hpp"
#include "test_data.hpp"

/*
Zone planner tests supply small dated platter-space areas through a contributor.
Local print and storage owners keep public views alive. Regionless travels on empty
plan groups isolate obstacle routing; a sliced cube supplies real regional provenance and shifted
instances for domain routing. Segment checks inspect filled material,
not just route vertices. The fixture selects travel.advanced to keep its settings
active while slicing, but invokes ZonePlanner directly without production wiring.
Ranked candidate checks use real layer weights with provenance-matched perimeter endpoints,
M membership, W ring attachments, and repeat queries to check deterministic ordering.
M_a is the attractor material domain after Z3 removal and W_a is its withdrawn
boundary. M_p is the initial platter after Z3 removal and W_p is its Z4-withdrawn
boundary. A forbidden area can split one attractor into several M_a components,
which exercise the bridge and mixed-route choices. Platter references and endpoint
links exercise M_p and W_p, while the expanded
initial-platter copy covers paths that lie exactly on a border. Bridge cases assert the
selected exit and entry points on W rings, contour a forbidden top, and check the
component approaches and the reference outside-length bound. An obstacle splitting one attractor exercises the
bridge path; an offset pair of components places a bridge endpoint on a W-ring vertex.
Endpoint scenes outside M assert that no attractor candidate is created, while fragmented
M, mixed connectors and unavailable bridges exercise the remaining route branches.
Mixed cases retain the nearest W ring point between a component and a far free endpoint.
A bent hole's box overlaps a second hole containing an island; an unavailable
box-selected bridge must leave the mixed hypotheses available.
Platter routing checks slice an overhanging roof and two posts, retaining real support
printing extrusions. Controlled endpoint chords test each shifted component;
the support roles are read from the sliced plan, not synthesized.
When no hypothesis passes across endpoint-group choices, the planner counts one
straight fallback. Budget-rejected spanning walls without an endpoint stay active.
Repeated calls retain deterministic geometry and count one fallback per failed travel.
Normal-layer provenance distinguishes synthetic in-part domain scenes from outside
travels. Regionless and real adhesion/support scenes check platter blockers, ignored
endpoint parts, both mixed directions and no attachment to a nearby attractor.
Endpoint-group cases compare the choices of keeping or omitting each group,
check score ties and separate validation of straight exits and the remaining route.

    Owner: own print, geometry storage, cancellation and contributor inputs
    FixedZones::contribute: copy synthetic dated areas and infer missing provenance
    rectangle / pending: construct millimeter geometry in platter coordinates
    check_clear: compare each route segment with original filled areas
    is_detour: compare the returned route length with the chord between its endpoints
    check_exits: split known straight exits from the rest and check the respective masks
    tests: exercise eligibility, cached margins, domain routing, jumps and limits;
        real default contributors retain disjoint tops when an endpoint occupies another top;
        activation checks inspect collected zones and routing without inherited legacy top avoidance;
        two planners with different external widths retain the same W attachment and route
*/
namespace {
namespace zones = slic3r_api::LayerExtrusionEdit::AdvancedTravel;
namespace travel = slic3r_api::LayerExtrusionEdit::TravelConnection;
namespace ac = slic3r_api::LayerExtrusionEdit::AvoidCrossing;
namespace work = slic3r_api::PluginCommon::polygon_work;

// Build an owned counter-clockwise rectangle using millimeter coordinates.
work::ExPolygon rectangle(double x, double y, double width, double height);
// Build an auxiliary travel with exact scaled platter endpoints and the requested tool.
travel::PendingTravel pending(double x1, double y1, double x2, double y2, extruder_index_t tool = 0);
// Check every segment's retained length after subtracting original material, allowing only rounding-sized differences.
void check_clear(const std::vector<c_point> &path, const work::ExPolygons &areas, Slic3r::PluginStorage &storage);
// Check every segment stays inside a fixed bed contour, allowing pieces that lie on its boundary.
void check_contained(const std::vector<c_point> &path, const work::ExPolygon &bed, Slic3r::PluginStorage &storage);
// Reproduce the planner's per-component W geometry for assertions about ring attachments.
work::ExPolygons withdrawn_components(const work::ExPolygons &components, double nozzle_diameter, coord_t withdrawal,
                                      Slic3r::PluginStorage &storage);
// Return the nearest point on a component contour or hole with stable insertion-order ties.
work::Point nearest_boundary_point(const work::ExPolygon &component, const work::Point &point);
// Return whether a route contains the exact scaled point selected by the planner or a bridge search.
bool contains_exact_point(const std::vector<c_point> &route, const work::Point &point);
// Check whether a route exceeds the chord between its ends by more than the scaled tolerance.
bool is_detour(const std::vector<c_point> &route);
// Check the expected clearance of the exit segments and require the core to avoid all kept obstacles.
void check_exits(const std::vector<c_point> &path, const work::ExPolygons &kept,
    const work::ExPolygons &other_source, const work::ExPolygons &other_target,
    bool source_exit, bool target_exit, Slic3r::PluginStorage &storage, bool exits_clear = true);

struct FixedZones : zones::ZoneContributor {
    std::vector<zones::TravelZone> areas;
    // Copy existing synthetic zones; for legacy fixtures, infer missing attractor provenance from the first real in-part endpoint.
    // New provenance scenes set source_layer and printing_catalog_id explicitly before collection.
    void contribute(const zones::ZoneContributorInput &input, std::vector<zones::TravelZone> &out) const override
    {
        const travel::TravelEndpoint *provenance = nullptr;
        for (const travel::PendingTravel &travel : input.travels) {
            for (const travel::TravelEndpoint *endpoint : {&travel.source, &travel.target})
                if (endpoint->region_island.valid() && endpoint->region_island.region_count() != 0 &&
                    endpoint->printing_catalog_id != SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE) {
                    provenance = endpoint;
                    break;
                }
            if (provenance != nullptr) break;
        }
        for (zones::TravelZone zone : areas) {
            if (zone.role == zones::ZoneRole::Attractor && provenance != nullptr && zone.source_layer == nullptr &&
                zone.scope.printing_catalog_id == SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE) {
                zone.source_layer = provenance->region_island.region(0).layer().handle();
                zone.scope.printing_catalog_id = provenance->printing_catalog_id;
            }
            out.push_back(std::move(zone));
        }
    }
};

struct Owner {
    Slic3r::Print print;
    Slic3r::Model model;
    Slic3r::DynamicPrintConfig config;
    Slic3r::PluginStorage storage;
    plugin_run_context context{};
    slic3r_api::Print print_view{reinterpret_cast<const print_handle *>(&print)};
    slic3r_api::PrintingPlan plan;
    ac::LayerInputCache cache;
    travel::LayerTravelChronology chronology;
    size_t checks = 0, cancel_at = 0;

    // Initialize public API storage and a two-instance cube; slicing is deferred until regional tests request it.
    Owner()
    {
        Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
        config = Slic3r::DynamicPrintConfig::full_print_config();
        config.set_deserialize_strict(Slic3r::Steps::plugin_exclusive_group_option_key(
            Slic3r::Orchestrator::instance(), STEP_LAYER_EXTRUSION_EDIT, "layer_extrusion_edit.travel"),
            "layer_extrusion_edit.travel.advanced");
        config.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
            {"perimeters", "2"}, {"fill_density", "20%"}, {"top_solid_layers", "1"},
            {"bottom_solid_layers", "1"}, {"skirts", "0"}, {"brim_width", "0"},
            {"gap_fill_enabled", "0"}, {"thin_walls", "0"}, {"thin_perimeters", "0"},
            {"avoid_crossing_perimeters", "1"}, {"avoid_crossing_not_first_layer", "0"},
            {"avoid_crossing_perimeters_max_detour", "0"}, {"nozzle_diameter", "0.4,1.2"},
            {"bed_shape", "-100x-100,300x-100,300x300,-100x300"}});
        Slic3r::ModelObject *object = model.add_object();
        object->add_volume(Slic3r::make_cube(20., 20., 0.6), Slic3r::ModelVolumeType::MODEL_PART, false);
        object->add_instance();
        object->add_instance()->set_offset(Slic3r::Vec3d(60., 30., 0.));
        object->ensure_on_bed();
        print.apply(model, config);
        print.set_status_silent();
        plan = slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&print.mutable_printing_plan()));
        context.plugin_storage = reinterpret_cast<storage_handle *>(&storage);
        context.host_context = reinterpret_cast<plugin_host_context *>(this);
        context.is_cancelled = [](plugin_host_context *host) {
            Owner &owner = *reinterpret_cast<Owner *>(host);
            return int(owner.cancel_at != 0 && ++owner.checks >= owner.cancel_at);
        };
    }
    // Refresh the plan view and return borrowed contributor inputs; unsliced scenes get an empty group with no printed instances.
    zones::ZoneContributorInput input()
    {
        Slic3r::Printing::PrintingPlan &current = print.mutable_printing_plan();
        plan = slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&current));
        if (current.groups.empty()) current.groups.resize(1);
        return {print_view, plan, context, cache, chronology.travels};
    }
    // Borrow applied print settings; auxiliary avoidance is passed separately to the planner.
    slic3r_api::Config settings() const
    { return print_view.config(); }
    // Process the owned print and refresh the borrowed plan view because processing can replace the native plan.
    void slice()
    {
        print.process();
        plan = slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&print.mutable_printing_plan()));
    }
    // Supply object 0 / last normal layer / island 0 provenance for classification and spacing, and retain the gap for fixture provenance.
    // Coordinates stay unchanged; this does not find the island geometrically containing either point.
    travel::PendingTravel in_part(travel::PendingTravel gap)
    {
        if (print.object(0).layers().empty()) {
            print.mutable_printing_plan().groups.clear();
            slice();
        }
        const slic3r_api::Layer layer(reinterpret_cast<const layer_handle *>(&print.object(0).layers().back()));
        gap.source.region_island = gap.target.region_island = layer.island(0).get_or_create_full_region_island(0);
        gap.source.printing_catalog_id = gap.target.printing_catalog_id =
            print.mutable_printing_plan().find_or_add_object_instance(print.object(0), 0);
        chronology.travels.push_back(gap);
        return gap;
    }
};

work::ExPolygon rectangle(double x, double y, double width, double height)
{
    work::ExPolygon area;
    area.contour.points = {{Slic3r::scale_i(x), Slic3r::scale_i(y)},
        {Slic3r::scale_i(x + width), Slic3r::scale_i(y)},
        {Slic3r::scale_i(x + width), Slic3r::scale_i(y + height)},
        {Slic3r::scale_i(x), Slic3r::scale_i(y + height)}};
    return area;
}

travel::PendingTravel pending(double x1, double y1, double x2, double y2, extruder_index_t tool)
{
    travel::PendingTravel gap;
    gap.source.position = {Slic3r::scale_i(x1), Slic3r::scale_i(y1), 0, true};
    gap.target.position = {Slic3r::scale_i(x2), Slic3r::scale_i(y2), 0, true};
    gap.extruder_id = tool;
    return gap;
}

void check_clear(const std::vector<c_point> &path, const work::ExPolygons &areas, Slic3r::PluginStorage &storage)
{
    for (size_t i = 1; i < path.size(); ++i) {
        work::Polyline line;
        line.points = {work::Point(path[i - 1]), work::Point(path[i])};
        const work::Polylines outside = slic3r_api::PluginCommon::clipper::diff_pl(
            reinterpret_cast<storage_handle *>(&storage), {line}, areas);
        double retained = 0.;
        for (const work::Polyline &part : outside) retained += part.length();
        CHECK(retained == Approx(line.length()).margin(2.));
    }
}

// Clip each route segment against the bed and require every retained outside piece to be contour contact.
void check_contained(const std::vector<c_point> &path, const work::ExPolygon &bed, Slic3r::PluginStorage &storage)
{
    const auto on_boundary = [&bed](const work::Point &point) {
        for (size_t edge = 0; edge < bed.contour.points.size(); ++edge)
            if (work::Line(bed.contour.points[edge], bed.contour.points[(edge + 1) % bed.contour.points.size()])
                    .distance_to_squared(point) <= 4.)
                return true;
        return false;
    };
    for (size_t segment = 1; segment < path.size(); ++segment) {
        work::Polyline line;
        line.points = {work::Point(path[segment - 1]), work::Point(path[segment])};
        const work::Polylines outside = slic3r_api::PluginCommon::clipper::diff_pl(
            reinterpret_cast<storage_handle *>(&storage), {line}, {bed});
        for (const work::Polyline &part : outside)
            for (size_t point = 1; point < part.points.size(); ++point) {
                const work::Point middle((part.points[point - 1].x() + part.points[point].x()) / 2,
                                         (part.points[point - 1].y() + part.points[point].y()) / 2);
                const bool boundary = on_boundary(part.points[point - 1]) && on_boundary(part.points[point]) &&
                    on_boundary(middle);
                CHECK(boundary);
            }
    }
}

work::ExPolygons withdrawn_components(const work::ExPolygons &components, const double nozzle_diameter,
                                      const coord_t withdrawal, Slic3r::PluginStorage &storage)
{
    work::ExPolygons withdrawn;
    for (const work::ExPolygon &component : components) {
        const slic3r_api::StoredExPolygonCollection stored = slic3r_api::PluginCommon::clipper::to_stored_expolygons(
            reinterpret_cast<storage_handle *>(&storage), {component});
        const slic3r_api::StoredExPolygonCollection result =
            slic3r_api::XYSizeCompensationPlugin::detail::elephant_foot_compensation(
                reinterpret_cast<storage_handle *>(&storage), stored.readonly().at(0),
                std::min(0.01, nozzle_diameter / 10.), withdrawal);
        const work::ExPolygons copied = slic3r_api::PluginCommon::clipper::from_expolygon_collection_view(result.readonly());
        REQUIRE(copied.size() == 1);
        withdrawn.push_back(copied.front());
    }
    return withdrawn;
}

work::Point nearest_boundary_point(const work::ExPolygon &component, const work::Point &point)
{
    work::Point nearest = point;
    double best = (std::numeric_limits<double>::max)();
    for (size_t ring = 0; ring <= component.holes.size(); ++ring) {
        const work::Points &points = ring == 0 ? component.contour.points : component.holes[ring - 1].points;
        for (size_t edge = 0; edge < points.size(); ++edge) {
            work::Point candidate;
            const double distance = work::Line(points[edge], points[(edge + 1) % points.size()]).distance_to_squared(point, &candidate);
            if (distance < best) { best = distance; nearest = candidate; }
        }
    }
    return nearest;
}

bool contains_exact_point(const std::vector<c_point> &route, const work::Point &point)
{
    return std::any_of(route.begin(), route.end(), [&point](const c_point &candidate) {
        return candidate.x == point.x() && candidate.y == point.y();
    });
}

TEST_CASE("Zone planner skips an unmapped free platter component", "[plugins][zone-planner][endpoint-groups]")
{
    Owner owner;
    owner.config.set_deserialize_strict("bed_shape", "0x0,20x0,20x20,0x20");
    owner.print.apply(owner.model, owner.config);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden,
        {rectangle(0.2004, -1., 21., 22.), rectangle(-0.00005, -1., 0.00075, 22.)}, 0, SIZE_MAX, {}});
    const travel::PendingTravel gap = owner.in_part(pending(0.0002, 10., 0.1, 10.));
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context,
        owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> path = planner.path(0, gap);
    REQUIRE(path.size() == 3);
    CHECK(path[1].x == Slic3r::scale_i(0.0004));
    CHECK(path[1].y == Slic3r::scale_i(10.));
    CHECK(planner.fallbacks() == 0);
}

TEST_CASE("Zone planner joins coincident exits from distinct attractors", "[plugins][zone-planner][endpoint-groups]")
{
    Owner owner;
    owner.config.set_deserialize_strict({{"avoid_travel_island", "1"}, {"avoid_travel_island_weight", "0"}});
    owner.print.apply(owner.model, owner.config);
    const bool separate = GENERATE(false, true);
    const double y = separate ? 2. : 0.;
    travel::PendingTravel gap = owner.in_part(pending(0.8, y, -0.8, -y));
    gap.target.printing_catalog_id = owner.print.mutable_printing_plan().find_or_add_object_instance(owner.print.object(0), 1);
    owner.chronology.travels = {gap};
    FixedZones contributor;
    work::ExPolygon upper;
    upper.contour.points = {{0, 0}, {1000000, 0}, {1000000, 10000000}, {-3000, 10000000}, {-3000, 1000000}};
    work::ExPolygon lower = upper;
    for (work::Point &point : lower.contour.points) point = work::Point(-point.x(), -point.y());
    contributor.areas.push_back({zones::ZoneRole::Attractor, {upper}, 0, SIZE_MAX, {}});
    contributor.areas.push_back({zones::ZoneRole::Attractor, {lower}, 0, SIZE_MAX, {}});
    contributor.areas[0].source_layer = gap.source.region_island.region(0).layer().handle();
    contributor.areas[0].scope.printing_catalog_id = gap.source.printing_catalog_id;
    contributor.areas[1].source_layer = gap.target.region_island.region(0).layer().handle();
    contributor.areas[1].scope.printing_catalog_id = gap.target.printing_catalog_id;
    contributor.areas.push_back({zones::ZoneRole::Forbidden,
        {rectangle(0., 0., 1., 10.), rectangle(-1., -10., 1., 10.)}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context,
        owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> path = planner.path(0, gap);
    REQUIRE(path.size() >= 2);
    CHECK(path.front().x == gap.source.position.x);
    CHECK(path.back().x == gap.target.position.x);
    CHECK(planner.fallbacks() == 0);
    CHECK(planner.bridges() == 1);
    CHECK(planner.domain_routes() == 1);
    CHECK(contains_exact_point(path, work::Point(0, 0)));
    CHECK(path[1].x == 0);
    CHECK(path[1].y == Slic3r::scale_i(y));
    CHECK(path[path.size() - 2].x == 0);
    CHECK(path[path.size() - 2].y == Slic3r::scale_i(-y));
}

TEST_CASE("Zone planner rejects re-entry after a nonzero exit", "[plugins][zone-planner][endpoint-groups]")
{
    Owner owner;
    const bool reversed = GENERATE(false, true);
    const work::ExPolygon lobe = rectangle(0., 0., 10., 2.);
    const work::ExPolygon thin = rectangle(10., -3., 0.00075, 3.);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {lobe, thin}, 0, SIZE_MAX, {}});
    travel::PendingTravel gap = pending(1., 0.5, 12., 0.5);
    if (reversed) std::swap(gap.source, gap.target);
    gap = owner.in_part(gap);
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context,
        owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> path = planner.path(0, gap);
    REQUIRE(path.size() == 2);
    CHECK(planner.fallbacks() == 0);
}

TEST_CASE("Zone planner filters a leaving chord by physical membership at an obstacle border", "[plugins][zone-planner][endpoint-groups]")
{
    Owner owner;
    travel::PendingTravel gap = owner.in_part(pending(0., 8., 9., 8.));
    gap.source.printing_catalog_id = owner.print.mutable_printing_plan().find_or_add_object_instance(owner.print.object(0), 1);
    owner.chronology.travels = {gap};
    work::ExPolygon u;
    for (const std::pair<double, double> &point : std::vector<std::pair<double, double>>{
             {0., 0.}, {10., 0.}, {10., 10.}, {7., 10.}, {7., 3.}, {3., 3.}, {3., 10.}, {0., 10.}})
        u.contour.points.emplace_back(Slic3r::scale_i(point.first), Slic3r::scale_i(point.second));
    const work::ExPolygon obstacle = rectangle(-2., 6., 2., 3.);
    REQUIRE(u.contains(work::Point(gap.source.position.x, gap.source.position.y), true));
    REQUIRE(obstacle.contains(work::Point(gap.source.position.x, gap.source.position.y), true));
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {u}, 0, SIZE_MAX, {}});
    contributor.areas.back().source_layer = gap.target.region_island.region(0).layer().handle();
    contributor.areas.back().scope.printing_catalog_id = gap.target.printing_catalog_id;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context,
        owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> path = planner.path(0, gap);
    REQUIRE(path.size() == 2);
    CHECK(planner.fallbacks() == 1);
}

TEST_CASE("Zone planner scores keeping or omitting one endpoint obstacle", "[plugins][zone-planner][endpoint-groups]")
{
    Owner owner;
    const bool reversed = GENERATE(false, true);
    const bool keep = GENERATE(false, true);
    const work::ExPolygon obstacle = rectangle(0., 0., 10., 2.);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    travel::PendingTravel gap = keep ? pending(1., 0.5, 12., 0.5) : pending(9.5, 0.49, 12., 0.49);
    if (reversed) std::swap(gap.source, gap.target);
    gap = owner.in_part(gap);
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context,
        owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> path = planner.path(0, gap);
    REQUIRE(path.size() >= 2);
    CHECK(path.front().x == gap.source.position.x);
    CHECK(path.front().y == gap.source.position.y);
    CHECK(path.back().x == gap.target.position.x);
    CHECK(path.back().y == gap.target.position.y);
    CHECK(planner.fallbacks() == 0);
    CHECK(planner.domain_routes() == 0);
    CHECK(is_detour(path) == keep);
    if (keep) {
        REQUIRE(path.size() > 2);
        const c_point reached = reversed ? path[path.size() - 2] : path[1];
        CHECK(reached.x == Slic3r::scale_i(1.));
        CHECK(reached.y == Slic3r::scale_i(-0.2));
        check_exits(path, {obstacle}, {}, {}, !reversed, reversed, owner.storage);
    } else {
        CHECK(path.size() == 2);
    }
}

TEST_CASE("Zone planner compares all four choices for separate endpoint groups", "[plugins][zone-planner][endpoint-groups]")
{
    Owner owner;
    const bool keep_source = GENERATE(false, true);
    const bool keep_target = GENERATE(false, true);
    const work::ExPolygon left = keep_source ? rectangle(-1., 0., 11., 2.) : rectangle(-10., 0., 10.51, 2.);
    const work::ExPolygon right = keep_target ? rectangle(14., 0., 11., 2.) : rectangle(23.49, 0., 10.51, 2.);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {left, right}, 0, SIZE_MAX, {}});
    const travel::PendingTravel gap = owner.in_part(pending(0., 0.5, 24., 0.5));
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context,
        owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> path = planner.path(0, gap);
    REQUIRE(path.size() >= 2);
    CHECK(planner.fallbacks() == 0);
    CHECK(is_detour(path) == (keep_source || keep_target));
    const work::Point source_exit(Slic3r::scale_i(0.), Slic3r::scale_i(-0.2));
    const work::Point target_exit(Slic3r::scale_i(24.), Slic3r::scale_i(-0.2));
    CHECK(contains_exact_point(path, source_exit) == keep_source);
    CHECK(contains_exact_point(path, target_exit) == keep_target);
    work::ExPolygons kept;
    if (keep_source) kept.push_back(left);
    if (keep_target) kept.push_back(right);
    check_exits(path, kept, keep_target ? work::ExPolygons{right} : work::ExPolygons{},
        keep_source ? work::ExPolygons{left} : work::ExPolygons{}, keep_source, keep_target, owner.storage);
}

TEST_CASE("Zone planner keeps or omits the shared endpoint group as a unit", "[plugins][zone-planner][endpoint-groups]")
{
    Owner owner;
    const bool keep = GENERATE(false, true);
    const work::ExPolygon obstacle = rectangle(0., 0., 10., 2.);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    const travel::PendingTravel gap = owner.in_part(keep ? pending(1., 0.5, 9., 0.5) : pending(1., 0.5, 2., 0.5));
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context,
        owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> path = planner.path(0, gap);
    CHECK(planner.fallbacks() == 0);
    if (keep) {
        REQUIRE(path.size() == 4);
        CHECK(path[1].y == Slic3r::scale_i(-0.2));
        CHECK(path[2].y == Slic3r::scale_i(-0.2));
        check_exits(path, {obstacle}, {}, {}, true, true, owner.storage);
    } else {
        CHECK(path.size() == 2);
    }
}

TEST_CASE("Zone planner keeps the first combination when scores tie", "[plugins][zone-planner][endpoint-groups]")
{
    Owner owner;
    const bool reversed = GENERATE(false, true);
    const work::ExPolygon obstacle = rectangle(0., 0., 10., 2.);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    travel::PendingTravel gap = pending(9.5, 1., 12., 1.);
    if (reversed) std::swap(gap.source, gap.target);
    gap = owner.in_part(gap);
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context,
        owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> path = planner.path(0, gap);
    REQUIRE(path.size() == 3);
    CHECK(path[1].x == Slic3r::scale_i(10.2));
    CHECK(path[1].y == Slic3r::scale_i(1.));
    CHECK(planner.fallbacks() == 0);
    CHECK_FALSE(is_detour(path));
    check_exits(path, {obstacle}, {}, {}, !reversed, reversed, owner.storage);
}

TEST_CASE("Zone planner allows an exit through two overlapping pieces of its group", "[plugins][zone-planner][endpoint-groups]")
{
    Owner owner;
    const bool reversed = GENERATE(false, true);
    const work::ExPolygon upper = rectangle(0., 0., 10., 2.);
    const work::ExPolygon lower = rectangle(0., -0.4, 10., 0.6);
    const work::ExPolygon other = rectangle(5., 3., 2., 2.);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {upper, lower, other}, 0, SIZE_MAX, {}});
    travel::PendingTravel gap = pending(1., 0.5, 12., 0.5);
    if (reversed) std::swap(gap.source, gap.target);
    gap = owner.in_part(gap);
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context,
        owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> path = planner.path(0, gap);
    REQUIRE(path.size() > 2);
    const c_point reached = reversed ? path[path.size() - 2] : path[1];
    CHECK(reached.x == Slic3r::scale_i(1.));
    CHECK(reached.y == Slic3r::scale_i(-0.4) - Slic3r::scale_i(0.2));
    check_exits(path, {upper, lower, other}, {other}, {other}, !reversed, reversed, owner.storage);
    // A fabricated exit crosses the third obstacle, but its remaining path is clear.
    const std::vector<c_point> crossed{{Slic3r::scale_i(6.), Slic3r::scale_i(2.)},
        {Slic3r::scale_i(6.), Slic3r::scale_i(6.)}, {Slic3r::scale_i(8.), Slic3r::scale_i(6.)}};
    const std::vector<c_point> backward(crossed.rbegin(), crossed.rend());
    check_exits(reversed ? backward : crossed, {upper, lower, other}, {other}, {other},
        !reversed, reversed, owner.storage, false);
    CHECK(planner.fallbacks() == 0);
    CHECK(is_detour(path));
}

TEST_CASE("Zone planner rejects an exit across a separate thin obstacle", "[plugins][zone-planner][endpoint-groups]")
{
    Owner owner;
    const bool reversed = GENERATE(false, true);
    const work::ExPolygon obstacle = rectangle(0., 0., 10., 2.);
    const work::ExPolygon thin = rectangle(0., -0.1, 10., 0.00075);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle, thin}, 0, SIZE_MAX, {}});
    travel::PendingTravel gap = pending(1., 0.5, 12., 0.5);
    if (reversed) std::swap(gap.source, gap.target);
    gap = owner.in_part(gap);
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context,
        owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const work::ExPolygons expanded = slic3r_api::PluginCommon::clipper::offset_ex(
        owner.context.plugin_storage, {thin}, double(Slic3r::scale_i(0.2)));
    REQUIRE(expanded.empty());
    const std::vector<c_point> path = planner.path(0, gap);
    REQUIRE(path.size() == 2);
    check_clear(path, {thin}, owner.storage);
    CHECK(planner.fallbacks() == 0);
    CHECK_FALSE(is_detour(path));
}

TEST_CASE("Zone planner can omit an endpoint group when its margins cover the bed", "[plugins][zone-planner][endpoint-groups]")
{
    Owner owner;
    owner.config.set_deserialize_strict("bed_shape", "0x0,20x0,20x20,0x20");
    owner.print.apply(owner.model, owner.config);
    const bool reversed = GENERATE(false, true);
    const work::ExPolygon obstacle = rectangle(0.1, 0.1, 19.8, 19.8);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    travel::PendingTravel gap = pending(10., 10., 19.95, 10.);
    if (reversed) std::swap(gap.source, gap.target);
    gap = owner.in_part(gap);
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context,
        owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> path = planner.path(0, gap);
    REQUIRE(path.size() == 2);
    CHECK(planner.fallbacks() == 0);
    CHECK(planner.domain_routes() == 0);
}

TEST_CASE("Zone planner rejects a straight exit across a notch in the bed", "[plugins][zone-planner][endpoint-groups]")
{
    Owner owner;
    owner.config.set_deserialize_strict("bed_shape", "0x0,10x0,10x2,2x2,2x10,0x10");
    owner.print.apply(owner.model, owner.config);
    const bool reversed = GENERATE(false, true);
    const work::ExPolygon obstacle = rectangle(-1., -1., 3.1, 5.);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    travel::PendingTravel gap = pending(1., 2.4, 8., 1.9);
    if (reversed) std::swap(gap.source, gap.target);
    gap = owner.in_part(gap);
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context,
        owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    work::ExPolygon bed = rectangle(0., 0., 10., 2.);
    bed.contour.points.pop_back();
    bed.contour.points.emplace_back(Slic3r::scale_i(2.), Slic3r::scale_i(2.));
    bed.contour.points.emplace_back(Slic3r::scale_i(2.), Slic3r::scale_i(10.));
    bed.contour.points.emplace_back(0, Slic3r::scale_i(10.));
    const std::vector<c_point> path = planner.path(0, gap);
    CHECK(planner.fallbacks() == 0);
    CHECK(contains_exact_point(path, work::Point(Slic3r::scale_i(2.), Slic3r::scale_i(2.))));
    check_contained(path, bed, owner.storage);
}

TEST_CASE("Zone planner exits to the nearest withdrawn component of a split slice", "[plugins][zone-planner][endpoint-groups]")
{
    Owner owner;
    owner.config.set_deserialize_strict({{"avoid_travel_island", "1"}, {"avoid_travel_island_weight", "0"}});
    owner.print.apply(owner.model, owner.config);
    const bool reversed = GENERATE(false, true);
    const work::ExPolygon slice = rectangle(0., -5., 10., 10.);
    const work::ExPolygon obstacle = rectangle(-1., 0., 12., 2.);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {slice}, 0, SIZE_MAX, {}});
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    travel::PendingTravel gap = pending(1., 0.5, 9., -2.);
    if (reversed) std::swap(gap.source, gap.target);
    gap = owner.in_part(gap);
    const travel::TravelEndpoint &inside = reversed ? gap.target : gap.source;
    const ac::LayerInput &input = *owner.cache.get(owner.print_view, inside.region_island.region(0).layer(),
        &owner.context).target;
    const work::ExPolygons pieces = slic3r_api::PluginCommon::clipper::diff_ex(
        owner.context.plugin_storage, {slice}, {obstacle});
    REQUIRE(pieces.size() == 2);
    const work::ExPolygons withdrawn = withdrawn_components(pieces, 0.4,
        coord_t(1.5 * double(input.perimeter_spacing)), owner.storage);
    const work::Point endpoint(inside.position.x, inside.position.y);
    const work::Point q0 = nearest_boundary_point(withdrawn[0], endpoint);
    const work::Point q1 = nearest_boundary_point(withdrawn[1], endpoint);
    const work::Point nearest = endpoint.distance_to(q0) < endpoint.distance_to(q1) ? q0 : q1;
    REQUIRE(nearest.y() < Slic3r::scale_i(-0.2));
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context,
        owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> path = planner.path(0, gap);
    REQUIRE(path.size() > 2);
    const c_point reached = reversed ? path[path.size() - 2] : path[1];
    CHECK(reached.x == nearest.x());
    CHECK(reached.y == nearest.y());
    CHECK(planner.fallbacks() == 0);
    CHECK(planner.domain_routes() == 1);
    CHECK(planner.bridges() == 0);
    check_exits(path, {obstacle}, {}, {}, !reversed, reversed, owner.storage);
}

TEST_CASE("Zone planner can use an attractor inside its endpoint's platter piece", "[plugins][zone-planner][endpoint-groups]")
{
    Owner owner;
    owner.config.set_deserialize_strict("avoid_crossing_perimeters_max_detour", "0.01");
    owner.print.apply(owner.model, owner.config);
    owner.slice();
    Slic3r::Printing::PrintingPlan &plan = owner.print.mutable_printing_plan();
    plan.groups.resize(1);
    plan.groups.front().printing_catalog_ids = {plan.find_or_add_object_instance(owner.print.object(0), 0)};
    const Slic3r::Layer &layer = owner.print.object(0).layers().back();
    const work::ExPolygons blockers = ac::platter_blockers(owner.context.plugin_storage, owner.plan, 0, layer.scaled_print_z());
    REQUIRE(blockers.size() == 1);
    const work::Point min = blockers.front().contour.bounding_box().min;
    const double x = Slic3r::unscaled(min.x()), y = Slic3r::unscaled(min.y());
    const bool reversed = GENERATE(false, true);
    travel::PendingTravel gap = owner.in_part(pending(x - 4., y + 0.8, x + 1., y + 0.8));
    gap.source.region_island = {};
    gap.source.printing_catalog_id = SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE;
    if (reversed) std::swap(gap.source, gap.target);
    owner.chronology.travels = {gap};
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {blockers.front()}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, layer.scaled_print_z());
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context,
        owner.print_view, owner.cache, owner.plan, 0, layer.scaled_print_z());
    const std::vector<c_point> path = planner.path(0, gap);
    REQUIRE(path.size() > 2);
    CHECK(planner.fallbacks() == 0);
    CHECK(planner.domain_routes() == 1);
    CHECK(planner.bridges() == 0);
    CHECK_FALSE(is_detour(path));
}

TEST_CASE("Zone planner tries omitting an endpoint wall that cuts the free bed", "[plugins][zone-planner][endpoint-groups]")
{
    Owner owner;
    owner.config.set_deserialize_strict("bed_shape", "0x0,20x0,20x20,0x20");
    owner.print.apply(owner.model, owner.config);
    const bool reversed = GENERATE(false, true);
    const work::ExPolygon wall = rectangle(8., -1., 4., 22.);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {wall}, 0, SIZE_MAX, {}});
    travel::PendingTravel gap = pending(9., 10., 17., 10.);
    if (reversed) std::swap(gap.source, gap.target);
    gap = owner.in_part(gap);
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context,
        owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> path = planner.path(0, gap);
    REQUIRE(path.size() == 2);
    CHECK(planner.fallbacks() == 0);
    CHECK(planner.domain_routes() == 0);
    CHECK_FALSE(is_detour(path));
}

TEST_CASE("Zone planner exits a platter piece without an attractor for a regionless endpoint", "[plugins][zone-planner][endpoint-groups]")
{
    Owner owner;
    owner.slice();
    Slic3r::Printing::PrintingPlan &plan = owner.print.mutable_printing_plan();
    plan.groups.resize(1);
    plan.groups.front().printing_catalog_ids = {plan.find_or_add_object_instance(owner.print.object(0), 0)};
    const Slic3r::Layer &layer = owner.print.object(0).layers().back();
    const work::ExPolygons blockers = ac::platter_blockers(owner.context.plugin_storage, owner.plan, 0, layer.scaled_print_z());
    REQUIRE(blockers.size() == 1);
    const work::BoundingBox box = blockers.front().contour.bounding_box();
    const double x = Slic3r::unscaled(box.min.x()), y = Slic3r::unscaled(box.min.y());
    const bool reversed = GENERATE(false, true);
    travel::PendingTravel gap = pending(x + 1., y + 0.5, Slic3r::unscaled(box.max.x()) + 2., y + 0.5);
    if (reversed) std::swap(gap.source, gap.target);
    owner.chronology.travels = {gap};
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {rectangle(x, y - 3., 20., 5.)}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, layer.scaled_print_z());
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context,
        owner.print_view, owner.cache, owner.plan, 0, layer.scaled_print_z());
    const std::vector<c_point> path = planner.path(0, gap);
    REQUIRE(path.size() > 2);
    const c_point reached = reversed ? path[path.size() - 2] : path[1];
    CHECK(reached.x == Slic3r::scale_i(x + 1.));
    CHECK(reached.y == box.min.y() - Slic3r::scale_i(0.2));
    check_exits(path, blockers, {}, {}, !reversed, reversed, owner.storage);
    CHECK(planner.fallbacks() == 0);
    CHECK(planner.domain_routes() == 0);
}

TEST_CASE("Zone planner accepts a zero length exit from a thin obstacle border", "[plugins][zone-planner][endpoint-groups]")
{
    Owner owner;
    const bool reversed = GENERATE(false, true);
    const work::ExPolygon obstacle = rectangle(0., 0., 10., 0.00075);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    contributor.areas.push_back({zones::ZoneRole::Attractor, {obstacle}, 0, SIZE_MAX, {}});
    travel::PendingTravel gap = pending(1., 0., 9., -4.);
    if (reversed) std::swap(gap.source, gap.target);
    gap = owner.in_part(gap);
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context,
        owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const work::ExPolygons expanded = slic3r_api::PluginCommon::clipper::offset_ex(
        owner.context.plugin_storage, {obstacle}, double(Slic3r::scale_i(0.2)));
    REQUIRE(expanded.empty());
    const std::vector<c_point> path = planner.path(0, gap);
    CHECK(path.size() == 2);
    check_exits(path, {obstacle}, {}, {}, false, false, owner.storage);
    CHECK(planner.fallbacks() == 0);
    CHECK(planner.domain_routes() == 0);
}

TEST_CASE("Zone planner does not let the core cross a kept thin obstacle", "[plugins][zone-planner][endpoint-groups]")
{
    Owner owner;
    const bool reversed = GENERATE(false, true);
    const work::ExPolygon obstacle = rectangle(0., 0., 10., 0.00075);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    contributor.areas.push_back({zones::ZoneRole::Attractor, {obstacle}, 0, SIZE_MAX, {}});
    travel::PendingTravel gap = pending(1., 0.000375, 12., 4.);
    if (reversed) std::swap(gap.source, gap.target);
    gap = owner.in_part(gap);
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context,
        owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> path = planner.path(0, gap);
    REQUIRE(path.size() > 2);
    CHECK(planner.fallbacks() == 0);
    CHECK(planner.domain_routes() == 1);
}

TEST_CASE("Zone planner publishes counters only for the winning endpoint combination", "[plugins][zone-planner][endpoint-groups]")
{
    Owner owner;
    owner.config.set_deserialize_strict({{"avoid_travel_island", "1"}, {"avoid_travel_island_weight", "0"}});
    owner.print.apply(owner.model, owner.config);
    const work::ExPolygon left = rectangle(-1., 0., 11., 2.);
    const work::ExPolygon right = rectangle(14., 0., 11., 2.);
    FixedZones attractors, obstacles;
    attractors.areas.push_back({zones::ZoneRole::Attractor, {left, right}, 0, SIZE_MAX, {}});
    obstacles.areas.push_back({zones::ZoneRole::Forbidden, {left, right}, 0, SIZE_MAX, {}});
    const travel::PendingTravel gap = owner.in_part(pending(0., 0.5, 24., 0.5));
    zones::TravelZoneView view({std::cref(attractors), std::cref(obstacles)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context,
        owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> path = planner.path(0, gap);
    check_exits(path, {left, right}, {right}, {left}, true, true, owner.storage);
    CHECK(planner.fallbacks() == 0);
    CHECK(is_detour(path));
    CHECK(planner.domain_routes() == 0);
    CHECK(planner.bridges() == 0);

    // Without forbidden zones the bridge is admissible, so it can compete with the winning platter route.
    zones::TravelZoneView clear_view({std::cref(attractors)}, owner.input());
    zones::ZonePlanner clear(compatibility, clear_view, owner.settings(), true, &owner.context,
        owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    CHECK(clear.path(0, gap).size() > 2);
    CHECK(clear.bridges() == 1);
}

TEST_CASE("Zone planner applies the platter reference of each endpoint combination", "[plugins][zone-planner][endpoint-groups]")
{
    Owner owner;
    owner.config.set_deserialize_strict({{"avoid_travel_island", "1"}, {"avoid_travel_island_weight", "0"}});
    owner.print.apply(owner.model, owner.config);
    const bool reversed = GENERATE(false, true);
    const work::ExPolygon slice = rectangle(0., -5., 10., 10.);
    const work::ExPolygon obstacle = rectangle(2., 0., 6., 2.);
    FixedZones attractors, obstacles;
    attractors.areas.push_back({zones::ZoneRole::Attractor, {slice}, 0, SIZE_MAX, {}});
    obstacles.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    travel::PendingTravel gap = pending(3., 0.5, 9., 0.5);
    if (reversed) std::swap(gap.source, gap.target);
    gap = owner.in_part(gap);
    zones::TravelZoneView clear_view({std::cref(attractors)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner clear(compatibility, clear_view, owner.settings(), true, &owner.context,
        owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    REQUIRE(clear.path(0, gap).size() == 2);
    CHECK(clear.domain_routes() == 1);
    zones::TravelZoneView view({std::cref(attractors), std::cref(obstacles)}, owner.input());
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context,
        owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> path = planner.path(0, gap);
    REQUIRE(path.size() > 2);
    CHECK(is_detour(path));
    CHECK(planner.fallbacks() == 0);
    check_exits(path, {obstacle}, {}, {}, !reversed, reversed, owner.storage);
}

bool is_detour(const std::vector<c_point> &route)
{
    if (route.size() <= 2) return false;
    double length = 0.;
    for (size_t i = 1; i < route.size(); ++i)
        length += work::Line(work::Point(route[i - 1]), work::Point(route[i])).length();
    return length > work::Line(work::Point(route.front()), work::Point(route.back())).length() + SCALED_EPSILON;
}

void check_exits(const std::vector<c_point> &path, const work::ExPolygons &kept,
    const work::ExPolygons &other_source, const work::ExPolygons &other_target,
    const bool source_exit, const bool target_exit, Slic3r::PluginStorage &storage, const bool exits_clear)
{
    REQUIRE(path.size() > size_t(source_exit) + size_t(target_exit));
    const auto clear = [&storage](const c_point &from, const c_point &to, const work::ExPolygons &mask) {
        if (mask.empty()) return true;
        work::Polyline line;
        line.points = {work::Point(from), work::Point(to)};
        double retained = 0.;
        for (const work::Polyline &part : slic3r_api::PluginCommon::clipper::diff_pl(
                 reinterpret_cast<storage_handle *>(&storage), {line}, mask))
            retained += part.length();
        return retained == Approx(line.length()).margin(2.);
    };
    if (source_exit) CHECK(clear(path[0], path[1], other_source) == exits_clear);
    if (target_exit) CHECK(clear(path.back(), path[path.size() - 2], other_target) == exits_clear);
    check_clear(std::vector<c_point>(path.begin() + source_exit, path.end() - target_exit), kept, storage);
}

TEST_CASE("Zone planner routes filled obstacles and preserves exact endpoints", "[plugins][zone-planner]")
{
    Owner owner;
    FixedZones contributor;
    work::ExPolygons areas{rectangle(4., -2., 2., 4.)};
    travel::PendingTravel gap = pending(0., 0., 10., 0.);
    bool expect_detour = true;
    SECTION("No forbidden zones") { areas.clear(); expect_detour = false; }
    SECTION("Single top") {}
    SECTION("Touching zones") { areas.push_back(rectangle(6., -1., 2., 2.)); }
    SECTION("Nested zones") { areas.push_back(rectangle(4.5, -1., 1., 2.)); }
    SECTION("Obstacle beyond the first exit") { areas.push_back(rectangle(7.5, -1., 0.6, 2.)); }
    SECTION("Diamond") {
        areas[0].contour.points = {{Slic3r::scale_i(3.), Slic3r::scale_i(5.)},
            {Slic3r::scale_i(5.), Slic3r::scale_i(3.)}, {Slic3r::scale_i(7.), Slic3r::scale_i(5.)},
            {Slic3r::scale_i(5.), Slic3r::scale_i(7.)}};
        const bool noninteger_exit = GENERATE(false, true);
        gap = noninteger_exit ? pending(0., 4., 10., 6.3) : pending(0., 5., 10., 5.);
    }
    SECTION("Rotated square") {
        areas[0].contour.points = {{Slic3r::scale_i(3.), Slic3r::scale_i(-1.)},
            {Slic3r::scale_i(6.), Slic3r::scale_i(-2.)}, {Slic3r::scale_i(7.), Slic3r::scale_i(1.)},
            {Slic3r::scale_i(4.), Slic3r::scale_i(2.)}};
        const bool noninteger_exit = GENERATE(false, true);
        if (noninteger_exit) gap = pending(0., 1., 10., 1.);
    }
    SECTION("Curved top") {
        areas[0].contour.points.clear();
        for (size_t i = 0; i < 32; ++i) {
            const double angle = double(i) * 2. * 3.14159265358979323846 / 32.;
            areas[0].contour.points.emplace_back(Slic3r::scale_i(5. + 2. * std::cos(angle)),
                                                  Slic3r::scale_i(2. * std::sin(angle)));
        }
    }
    SECTION("Margin closes a passage") {
        areas = {rectangle(4., -3., 2., 2.9), rectangle(4., 0.1, 2., 2.9)};
    }
    SECTION("Endpoint zone ignored before margins merge") {
        areas = {rectangle(-1., -1., 2., 2.), rectangle(1.1, -1., 2., 2.)};
    }
    // These cases exercise the complete-path material check: a real crossing must
    // be rejected, while a hole, a vertex contact and only the expanded margin stay usable.
    SECTION("Crossing only a hole") {
        areas = {rectangle(-2., -4., 14., 8.)};
        work::Polygon hole = rectangle(-1., -3., 12., 6.).contour;
        std::reverse(hole.points.begin(), hole.points.end());
        areas[0].holes.push_back(hole);
        expect_detour = false;
    }
    SECTION("Zone in another zone's hole") {
        work::ExPolygon ring = rectangle(-2., -4., 14., 8.);
        work::Polygon hole = rectangle(-1., -3., 12., 6.).contour;
        std::reverse(hole.points.begin(), hole.points.end());
        ring.holes.push_back(hole);
        areas.push_back(ring);
    }
    SECTION("Multi-lobe component") {
        areas = {rectangle(3., -2., 1., 4.), rectangle(6., -2., 1., 4.), rectangle(3., 1., 4., 1.)};
    }
    SECTION("Contact at an expanded obstacle vertex") { gap = pending(0., -8.4, 10., 1.6); expect_detour = false; }
    SECTION("Travel through only the expanded margin") { gap = pending(0., -2.2, 10., -2.2); expect_detour = false; }
    for (const work::ExPolygon &area : areas)
        contributor.areas.push_back({zones::ZoneRole::Forbidden, {area}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> result = planner.path(0, gap);
    REQUIRE(result.size() >= 2);
    CHECK(result.front().x == gap.source.position.x);
    CHECK(result.front().y == gap.source.position.y);
    CHECK(result.back().x == gap.target.position.x);
    CHECK(result.back().y == gap.target.position.y);
    CHECK(planner.fallbacks() == 0);
    CHECK(is_detour(result) == expect_detour);
    work::ExPolygons remaining;
    for (const work::ExPolygon &area : areas)
        if (!area.contains(work::Point(result.front())) && !area.contains(work::Point(result.back()))) remaining.push_back(area);
    check_clear(result, remaining, owner.storage);
}

// A source layer mismatch prevents a point in M from borrowing a neighboring attractor.
TEST_CASE("Zone planner requires the endpoint source layer", "[plugins][zone-planner]")
{
    Owner owner;
    owner.slice();
    const slic3r_api::Layer target_layer(reinterpret_cast<const layer_handle *>(&owner.print.object(0).layers().back()));
    const slic3r_api::Layer other_layer(reinterpret_cast<const layer_handle *>(&owner.print.object(0).layers().front()));
    const uint32_t catalog = owner.print.mutable_printing_plan().find_or_add_object_instance(owner.print.object(0), 0);
    travel::PendingTravel gap = pending(2., 5., 8., 5.);
    gap.source.region_island = gap.target.region_island = target_layer.island(0).get_or_create_full_region_island(0);
    gap.source.printing_catalog_id = gap.target.printing_catalog_id = catalog;
    owner.chronology.travels = {gap};
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {rectangle(0., 0., 10., 10.)}, 0, SIZE_MAX, {}, other_layer.handle()});
    contributor.areas.front().scope.printing_catalog_id = catalog;
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view,
        owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() == 2);
    CHECK(planner.domain_routes() == 0);
    CHECK(planner.fallbacks() == 0);
}

// Touching raw slices stay active before subtraction, but the first-instance endpoint cannot borrow the second source.
TEST_CASE("Zone planner does not borrow a touching instance source", "[plugins][zone-planner]")
{
    Owner owner;
    owner.slice();
    const slic3r_api::Layer layer(reinterpret_cast<const layer_handle *>(&owner.print.object(0).layers().back()));
    const uint32_t first_catalog = owner.print.mutable_printing_plan().find_or_add_object_instance(owner.print.object(0), 0);
    const uint32_t second_catalog = owner.print.mutable_printing_plan().find_or_add_object_instance(owner.print.object(0), 1);
    const work::ExPolygon first_slice = rectangle(0., 0., 5., 10.);
    const work::ExPolygon second_slice = rectangle(5., 0., 5., 10.);
    const work::ExPolygon forbidden = rectangle(0., 0., 4.8, 10.);
    const work::ExPolygons expanded = slic3r_api::PluginCommon::clipper::offset_ex(
        owner.context.plugin_storage, {forbidden}, double(Slic3r::scale_i(0.2)));
    const work::ExPolygons m = slic3r_api::PluginCommon::clipper::diff_ex(
        owner.context.plugin_storage, {first_slice, second_slice}, expanded);
    REQUIRE(m.size() == 1);
    const work::Point source(Slic3r::scale_i(5.05), Slic3r::scale_i(5.));
    CHECK(m.front().contains(source, true));
    CHECK(slic3r_api::PluginCommon::clipper::intersection_ex(
        owner.context.plugin_storage, {m.front()}, {first_slice}).empty());
    const work::ExPolygons surviving_slice = slic3r_api::PluginCommon::clipper::intersection_ex(
        owner.context.plugin_storage, {m.front()}, {second_slice});
    REQUIRE(surviving_slice.size() == 1);
    CHECK(surviving_slice.front().area() > 0.);
    travel::PendingTravel gap = pending(5.05, 5., 14., 5.);
    gap.source.region_island = gap.target.region_island = layer.island(0).get_or_create_full_region_island(0);
    gap.source.printing_catalog_id = first_catalog;
    const travel::PlannedPosition target_position = gap.target.position;
    gap.target = travel::TravelEndpoint{};
    gap.target.position = target_position;
    gap.target.printing_catalog_id = second_catalog;
    owner.chronology.travels = {gap};
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {first_slice}, 0, SIZE_MAX,
                                 {EXTRUDER_ID_UNKNOWN, first_catalog}, layer.handle()});
    contributor.areas.push_back({zones::ZoneRole::Attractor, {second_slice}, 0, SIZE_MAX,
                                 {EXTRUDER_ID_UNKNOWN, second_catalog}, layer.handle()});
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {forbidden}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    const zones::ActiveZones &active = view.query(0, gap);
    REQUIRE(active.attractor_indexes.size() == 2);
    CHECK(view.zone(active.attractor_indexes[0]).scope.printing_catalog_id == first_catalog);
    CHECK(view.zone(active.attractor_indexes[1]).scope.printing_catalog_id == second_catalog);
    REQUIRE(active.forbidden_indexes.size() == 1);
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view,
        owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    CHECK(route.size() == 2);
    CHECK(planner.domain_routes() == 0);
    CHECK(planner.fallbacks() == 0);
}

// Two raw slices that form one M component remain separate sources for both endpoint tests.
TEST_CASE("Zone planner keeps every source slice of one M component", "[plugins][zone-planner]")
{
    Owner owner;
    const travel::PendingTravel gap = owner.in_part(pending(1., 5., 9., 5.));
    const layer_handle *layer = gap.source.region_island.region(0).layer().handle();
    const uint32_t catalog = gap.source.printing_catalog_id;
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {rectangle(0., 0., 5., 10.)}, 0, SIZE_MAX, {}, layer});
    contributor.areas.push_back({zones::ZoneRole::Attractor, {rectangle(5., 0., 5., 10.)}, 0, SIZE_MAX, {}, layer});
    for (zones::TravelZone &zone : contributor.areas) zone.scope.printing_catalog_id = catalog;
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    const zones::ActiveZones &active = view.query(0, gap);
    REQUIRE(active.attractor_indexes.size() == 2);
    CHECK(view.zone(active.attractor_indexes[0]).source_layer == layer);
    CHECK(view.zone(active.attractor_indexes[1]).source_layer == layer);
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view,
        owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() == 2);
    CHECK(planner.domain_routes() == 1);
    CHECK(planner.bridges() == 0);
}

// One raw source split by a forbidden wall supplies both M components and keeps the bridge admissible.
TEST_CASE("Zone planner retains one source across fragmented M components", "[plugins][zone-planner]")
{
    Owner owner;
    const travel::PendingTravel gap = owner.in_part(pending(2., 5., 8., 5.));
    const layer_handle *layer = gap.source.region_island.region(0).layer().handle();
    const uint32_t catalog = gap.source.printing_catalog_id;
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {rectangle(0., 0., 10., 10.)}, 0, SIZE_MAX, {}, layer});
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {rectangle(4.5, -1., 1., 12.)}, 0, SIZE_MAX, {}});
    contributor.areas.front().scope.printing_catalog_id = catalog;
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view,
        owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() > 2);
    CHECK(planner.domain_routes() == 1);
    CHECK(planner.bridges() == 1);
    CHECK(planner.fallbacks() == 0);
    check_clear(route, {rectangle(4.5, -1., 1., 12.)}, owner.storage);
}

// A target in its own M keeps its own W link even when a nearer thin wall is another component.
TEST_CASE("Zone planner does not use a nearer thin neighboring wall", "[plugins][zone-planner]")
{
    Owner owner;
    bool source_outside = false;
    travel::PendingTravel gap;
    SECTION("Target in M reaches its nearest W point") {
        gap = owner.in_part(pending(2., 5., 9.95, 5.));
    }
    SECTION("Source outside both domains has no candidate") {
        source_outside = true;
        gap = owner.in_part(pending(10.06, 5., 14., 5.));
    }
    const layer_handle *layer = gap.source.region_island.region(0).layer().handle();
    const uint32_t catalog = gap.source.printing_catalog_id;
    work::ExPolygon own = rectangle(0., 0., 10., 10.);
    work::Polygon hole = rectangle(4., 2., 4., 6.).contour;
    std::reverse(hole.points.begin(), hole.points.end());
    own.holes.push_back(hole);
    const work::ExPolygon thin = rectangle(10.1, 0., 0.1, 10.);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {own}, 0, SIZE_MAX, {}, layer});
    contributor.areas.push_back({zones::ZoneRole::Attractor, {thin}, 0, SIZE_MAX, {}, layer});
    for (zones::TravelZone &zone : contributor.areas) zone.scope.printing_catalog_id = catalog;
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view,
        owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    const ac::Input &layer_input = owner.cache.get(owner.print_view,
        gap.source.region_island.region(0).layer(), &owner.context);
    const coord_t withdrawal = coord_t(1.5 * double(layer_input.target->perimeter_spacing));
    const work::ExPolygons own_w = withdrawn_components({own}, 0.4, withdrawal, owner.storage);
    const work::ExPolygons thin_w = withdrawn_components({thin}, 0.4, withdrawal, owner.storage);
    REQUIRE(own_w.size() == 1);
    REQUIRE(thin_w.size() == 1);
    const work::Point endpoint(gap.target.position.x, gap.target.position.y);
    const work::Point source(gap.source.position.x, gap.source.position.y);
    const work::Point expected = nearest_boundary_point(own_w.front(),
        endpoint);
    const work::Point neighboring = nearest_boundary_point(thin_w.front(), endpoint);
    if (source_outside) {
        CHECK_FALSE(own.contains(source, true));
        CHECK_FALSE(thin.contains(source, true));
        CHECK(route.size() == 2);
        CHECK(planner.domain_routes() == 0);
        CHECK_FALSE(contains_exact_point(route, expected));
        CHECK_FALSE(contains_exact_point(route, neighboring));
        CHECK(planner.fallbacks() == 0);
        return;
    }
    CHECK(own.contains(endpoint, true));
    CHECK_FALSE(own_w.front().contains(endpoint, true));
    CHECK(endpoint.distance_to(neighboring) < endpoint.distance_to(expected));
    REQUIRE(route.size() > 2);
    CHECK(planner.domain_routes() == 1);
    CHECK(planner.fallbacks() == 0);
    CHECK(contains_exact_point(route, expected));
    CHECK_FALSE(contains_exact_point(route, neighboring));
}

// A component narrower than the requested withdrawal retains W equal to M and still routes by its provenance.
TEST_CASE("Zone planner keeps a thin component when W equals M", "[plugins][zone-planner]")
{
    Owner owner;
    const travel::PendingTravel gap = owner.in_part(pending(0.25, 2., 0.25, 8.));
    const layer_handle *layer = gap.source.region_island.region(0).layer().handle();
    const uint32_t catalog = gap.source.printing_catalog_id;
    const work::ExPolygon thin = rectangle(0., 0., 0.5, 10.);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {thin}, 0, SIZE_MAX, {}, layer});
    contributor.areas.front().scope.printing_catalog_id = catalog;
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view,
        owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() == 2);
    CHECK(planner.domain_routes() == 1);
    CHECK(planner.fallbacks() == 0);
    const ac::Input &layer_input = owner.cache.get(owner.print_view,
        gap.source.region_island.region(0).layer(), &owner.context);
    const work::ExPolygons expected = withdrawn_components({thin}, 0.4,
        coord_t(1.5 * double(layer_input.target->perimeter_spacing)), owner.storage);
    REQUIRE(expected.size() == 1);
    CHECK(expected.front().area() == Approx(thin.area()));
}

// Tool-specific minimum widths must keep exact W points separate, while external width alone must not change them.
TEST_CASE("Zone planner keeps W independent of external perimeter width", "[plugins][zone-planner]")
{
    Owner owner;
    owner.slice();
    const slic3r_api::Layer layer(reinterpret_cast<const layer_handle *>(&owner.print.object(0).layers().back()));
    const travel::PendingTravel gap = owner.in_part(pending(0.04, 7., 4.01, 7.));
    const uint32_t catalog = gap.source.printing_catalog_id;
    const work::ExPolygons finger = slic3r_api::PluginCommon::clipper::union_ex(owner.context.plugin_storage,
        {rectangle(0., 0., 0.05, 8.), rectangle(4., 0., 0.05, 8.), rectangle(0., 0., 4.05, 0.05)});
    REQUIRE(finger.size() == 1);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, finger, 0, SIZE_MAX, {}, layer.handle()});
    contributor.areas.front().scope.printing_catalog_id = catalog;
    Slic3r::DynamicPrintConfig tool_config = owner.config;
    tool_config.set_deserialize_strict("nozzle_diameter", "0.4,0.1,0.05");
    const slic3r_api::Config tool_settings(Slic3r::ApiHost::to_config_handle(&tool_config));
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, tool_settings, true, &owner.context,
        owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<double> nozzle_diameters{0.4, 0.1, 0.05};
    std::vector<work::Point> source_w_points, target_w_points;
    const ac::Input &layer_input = owner.cache.get(owner.print_view, layer, &owner.context);
    for (size_t tool = 0; tool < nozzle_diameters.size(); ++tool) {
        travel::PendingTravel tool_gap = gap;
        tool_gap.extruder_id = extruder_index_t(tool);
        const std::vector<c_point> route = planner.path(tool, tool_gap);
        const work::ExPolygons expected_w = withdrawn_components(finger, nozzle_diameters[tool],
            coord_t(1.5 * double(layer_input.target->perimeter_spacing)), owner.storage);
        REQUIRE(expected_w.size() == 1);
        const work::Point source_w = nearest_boundary_point(expected_w.front(),
            work::Point(tool_gap.source.position.x, tool_gap.source.position.y));
        const work::Point target_w = nearest_boundary_point(expected_w.front(),
            work::Point(tool_gap.target.position.x, tool_gap.target.position.y));
        source_w_points.push_back(source_w);
        target_w_points.push_back(target_w);
        REQUIRE(route.size() >= 3);
        CHECK(route.front().x == tool_gap.source.position.x);
        CHECK(route.front().y == tool_gap.source.position.y);
        CHECK(route.back().x == tool_gap.target.position.x);
        CHECK(route.back().y == tool_gap.target.position.y);
        CHECK(contains_exact_point(route, source_w));
        CHECK(contains_exact_point(route, target_w));
        CHECK(planner.domain_routes() == tool + 1);
        CHECK(planner.fallbacks() == 0);
    }
    CHECK((source_w_points[0].x() != source_w_points[2].x() || source_w_points[0].y() != source_w_points[2].y()));
    CHECK((target_w_points[0].x() != target_w_points[2].x() || target_w_points[0].y() != target_w_points[2].y()));
    Slic3r::DynamicPrintConfig narrow_config = tool_config;
    Slic3r::DynamicPrintConfig wide_config = tool_config;
    narrow_config.set_deserialize_strict("external_perimeter_extrusion_width", "0.1");
    wide_config.set_deserialize_strict("external_perimeter_extrusion_width", "0.8");
    const slic3r_api::Config narrow_settings(Slic3r::ApiHost::to_config_handle(&narrow_config));
    const slic3r_api::Config wide_settings(Slic3r::ApiHost::to_config_handle(&wide_config));
    zones::TravelZoneView narrow_view({std::cref(contributor)}, owner.input());
    zones::TravelZoneView wide_view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner narrow_compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    ac::CompatibilityPlanner wide_compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner narrow_planner(narrow_compatibility, narrow_view, narrow_settings, true, &owner.context,
        owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner wide_planner(wide_compatibility, wide_view, wide_settings, true, &owner.context,
        owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> narrow_route = narrow_planner.path(0, gap);
    const std::vector<c_point> wide_route = wide_planner.path(0, gap);
    REQUIRE(narrow_route.size() >= 3);
    REQUIRE(wide_route.size() == narrow_route.size());
    for (size_t point = 0; point < narrow_route.size(); ++point) {
        CHECK(wide_route[point].x == narrow_route[point].x);
        CHECK(wide_route[point].y == narrow_route[point].y);
    }
    CHECK(narrow_planner.domain_routes() == 1);
    CHECK(wide_planner.domain_routes() == 1);
    const work::ExPolygons expected_w = withdrawn_components(finger, 0.4,
        coord_t(1.5 * double(layer_input.target->perimeter_spacing)), owner.storage);
    REQUIRE(expected_w.size() == 1);
    const work::Point expected = nearest_boundary_point(expected_w.front(),
        work::Point(gap.target.position.x, gap.target.position.y));
    CHECK(contains_exact_point(narrow_route, expected));
    CHECK(contains_exact_point(wide_route, expected));
}

TEST_CASE("Zone planner caches tool margins and endpoint ignored sets independently", "[plugins][zone-planner]")
{
    Owner owner;
    FixedZones contributor;
    const work::ExPolygon area = rectangle(4., -2., 2., 4.);
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {area}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    CHECK(planner.path(0, pending(5., 0., 10., 0.)).size() == 2);
    const std::vector<c_point> small = planner.path(1, pending(0., 0., 10., 0., 0));
    const std::vector<c_point> large = planner.path(2, pending(0., 0., 10., 0., 1));
    REQUIRE(small.size() > 2);
    REQUIRE(large.size() > 2);
    CHECK(std::abs(large[2].y) > std::abs(small[2].y));
    CHECK(view.query(2, pending(0., 0., 10., 0.)).id == view.query(2, pending(0., 0., 10., 0., 1)).id);
    CHECK(is_detour(small));
    CHECK(is_detour(large));
    CHECK(planner.fallbacks() == 0);
    check_clear(small, {area}, owner.storage);
    check_clear(large, {area}, owner.storage);
}

TEST_CASE("Zone planner falls back when a ring encloses either endpoint", "[plugins][zone-planner]")
{
    const bool target_inside = GENERATE(false, true);
    Owner owner;
    work::ExPolygon ring = rectangle(2., -4., 6., 8.);
    work::Polygon hole = rectangle(3., -3., 4., 6.).contour;
    std::reverse(hole.points.begin(), hole.points.end());
    ring.holes.push_back(hole);
    FixedZones contributor;
    const work::ExPolygon retained = rectangle(8.8, -0.5, 0.4, 1.);
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {ring, retained}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> result = planner.path(0,
        target_inside ? pending(0., 0., 5., 0.) : pending(5., 0., 10., 0.));
    REQUIRE(result.size() == 2);
    CHECK(planner.fallbacks() == 1);
    CHECK_FALSE(is_detour(result));
}

// Endpoints in the expanded margin use platter links even when they are outside W_a.
TEST_CASE("Zone planner leaves endpoints outside M without attractor attachments", "[plugins][zone-planner]")
{
    Owner owner;
    const bool budget_rejects = GENERATE(false, true);
    owner.config.set_deserialize_strict("avoid_crossing_perimeters_max_detour", budget_rejects ? "0.1" : "0");
    owner.print.apply(owner.model, owner.config);
    const travel::PendingTravel gap = owner.in_part(pending(0.5, 10.3, 9.5, 10.3));
    work::ExPolygon domain;
    for (const std::pair<double, double> &point : std::vector<std::pair<double, double>>{
             {0., 0.}, {10., 0.}, {10., 10.}, {9., 10.}, {9., 1.}, {1., 1.}, {1., 10.}, {0., 10.}})
        domain.contour.points.push_back({Slic3r::scale_i(point.first), Slic3r::scale_i(point.second)});
    work::ExPolygon ring = rectangle(-0.2, -0.2, 10.4, 10.4);
    work::Polygon hole = rectangle(0., 0., 10., 10.).contour;
    std::reverse(hole.points.begin(), hole.points.end());
    ring.holes.push_back(hole);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {domain}, 0, SIZE_MAX, {}});
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {ring}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    CHECK((route.size() > 2) == !budget_rejects);
    CHECK(planner.domain_routes() == 0);
    CHECK(planner.fallbacks() == size_t(budget_rejects));
    CHECK(is_detour(route) == !budget_rejects);
    check_clear(route, {}, owner.storage);
}

TEST_CASE("Zone planner routes when a crossed blocker reaches an endpoint margin", "[plugins][zone-planner]")
{
    Owner owner;
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {rectangle(4., -2., 2., 4.)}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const bool target_inside = GENERATE(false, true);
    const std::vector<c_point> result = planner.path(0,
        target_inside ? pending(0., 0., 6.1, 0.) : pending(3.9, 0., 10., 0.));
    CHECK(result.size() > 2);
    CHECK(planner.fallbacks() == 0);
    CHECK(is_detour(result));
}

// Real object slices supply an enclosing platter component; repeated calls exercise the ignored-component cache identity.
TEST_CASE("Zone planner falls back for an enclosing platter component", "[plugins][zone-planner]")
{
    Owner owner;
    owner.model.clear_objects();
    Slic3r::TriangleMesh mesh = Slic3r::make_cube(20., 2., 0.6);
    Slic3r::TriangleMesh top = Slic3r::make_cube(20., 2., 0.6);
    top.translate(0., 18., 0.); mesh.merge(top);
    for (double x : {0., 18.}) {
        Slic3r::TriangleMesh side = Slic3r::make_cube(2., 16., 0.6);
        side.translate(x, 2., 0.); mesh.merge(side);
    }
    Slic3r::ModelObject *object = owner.model.add_object();
    object->add_volume(mesh, Slic3r::ModelVolumeType::MODEL_PART, false);
    object->add_instance();
    object->ensure_on_bed();
    owner.print.apply(owner.model, owner.config);
    owner.slice();
    Slic3r::Printing::PrintingPlan &plan = owner.print.mutable_printing_plan();
    plan.groups.resize(1);
    plan.groups[0].printing_catalog_ids = {plan.find_or_add_object_instance(owner.print.object(0), 0)};
    FixedZones contributor;
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    const work::ExPolygons slices = ac::platter_blockers(owner.context.plugin_storage, owner.plan, 0, Slic3r::scale_i(0.6));
    REQUIRE(slices.size() == 1);
    REQUIRE(slices.front().holes.size() == 1);
    double x = 0., y = 0.;
    for (const work::Point &point : slices.front().contour.points) {
        x += Slic3r::unscaled(point.x()); y += Slic3r::unscaled(point.y());
    }
    x /= slices.front().contour.points.size(); y /= slices.front().contour.points.size();
    const travel::PendingTravel gap = pending(x, y, x + 14., y);
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    for (size_t t = 0; t < 2; ++t) {
        const std::vector<c_point> route = planner.path(t, gap);
        REQUIRE(route.size() == 2);
        CHECK(route.front().x == gap.source.position.x);
        CHECK(route.back().x == gap.target.position.x);
        CHECK(planner.fallbacks() == t + 1);
    }
}

// Zone and platter blockers remain active; when no hypothesis passes, the planner counts one straight fallback.
TEST_CASE("Zone planner falls back when zone and platter blockers remain", "[plugins][zone-planner]")
{
    Owner owner;
    owner.slice();
    Slic3r::Printing::PrintingPlan &plan = owner.print.mutable_printing_plan();
    plan.groups.resize(1);
    plan.groups[0].printing_catalog_ids = {plan.find_or_add_object_instance(owner.print.object(0), 0)};
    const Slic3r::BoundingBox box = owner.print.object(0).layers().back().lslices().front().contour.bounding_box();
    const c_point shift = owner.print_view.object(0).instance_shift(0);
    const double x = Slic3r::unscaled(box.min.x() + shift.x), y = Slic3r::unscaled(box.min.y() + shift.y);
    work::ExPolygon ring = rectangle(x, y - 2., 26., 24.);
    work::Polygon hole = rectangle(x + 1., y - 1., 24., 22.).contour;
    std::reverse(hole.points.begin(), hole.points.end());
    ring.holes.push_back(hole);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {ring}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, pending(x - 2., y + 10., x + 22., y + 10.));
    REQUIRE(route.size() == 2);
    CHECK(planner.fallbacks() == 1);
}

// Nested enclosing rings remain active, so each travel uses the counted straight fallback.
TEST_CASE("Zone planner keeps nested rings and counts each fallback", "[plugins][zone-planner]")
{
    const size_t count = GENERATE(size_t(1), size_t(3));
    Owner owner;
    FixedZones contributor;
    work::ExPolygons rings;
    for (size_t i = 0; i < count; ++i) {
        const double radius = 2. + 2. * i;
        work::ExPolygon ring = rectangle(-radius, -radius, 2. * radius, 2. * radius);
        work::Polygon hole = rectangle(-radius + 0.5, -radius + 0.5, 2. * radius - 1., 2. * radius - 1.).contour;
        std::reverse(hole.points.begin(), hole.points.end());
        ring.holes.push_back(hole);
        rings.push_back(ring);
    }
    contributor.areas.push_back({zones::ZoneRole::Forbidden, rings, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    for (size_t t = 0; t < 2; ++t) {
        CHECK(planner.path(t, pending(0., 0., 10., 0.)).size() == 2);
        CHECK(planner.fallbacks() == t + 1);
    }
}

// Budget-only rejection must not authorize crossing walls; repeated calls preserve both geometry and diagnostics.
TEST_CASE("Zone planner keeps stacked walls when only the budget rejects detours", "[plugins][zone-planner]")
{
    const size_t walls = GENERATE(size_t(0), size_t(1), size_t(4));
    Owner owner;
    owner.config.set_deserialize_strict("avoid_crossing_perimeters_max_detour", "0.1");
    owner.print.apply(owner.model, owner.config);
    FixedZones contributor;
    work::ExPolygons areas;
    for (size_t i = 0; i < walls; ++i) areas.push_back(rectangle(2. + 2. * i, -100., 0.5, 200.));
    const work::ExPolygon retained = rectangle(4., 2., 1., 1.);
    areas.push_back(retained);
    contributor.areas.push_back({zones::ZoneRole::Forbidden, areas, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const travel::PendingTravel gap = pending(0., 0., 12., 0.);
    const std::vector<c_point> first = planner.path(0, gap);
    REQUIRE(first.size() == 2);
    CHECK(planner.fallbacks() == size_t(walls != 0));
    check_clear(first, {retained}, owner.storage);
    const std::vector<c_point> second = planner.path(1, gap);
    REQUIRE(second.size() == first.size());
    for (size_t i = 0; i < first.size(); ++i) {
        CHECK(first[i].x == second[i].x);
        CHECK(first[i].y == second[i].y);
    }
}

TEST_CASE("Zone planner accepts real global adhesion provenance without an object instance", "[plugins][zone-planner]")
{
    const bool brim = GENERATE(false, true);
    Owner owner;
    owner.config.set_deserialize_strict({{"skirts", "2"}, {"brim_width", brim ? "2" : "0"}});
    owner.print.apply(owner.model, owner.config);
    owner.slice();
    owner.print.pipeline_progress().reset();
    Slic3r::Steps::StepExtrusionOrdering::clear(owner.print);
    Slic3r::Steps::StepExtrusionOrdering::prepare(owner.print);
    Slic3r::Steps::StepExtrusionOrdering::run_step(Slic3r::Orchestrator::instance(), owner.print);
    owner.plan = slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&owner.print.mutable_printing_plan()));
    travel::TravelEndpoint auxiliary;
    for (const Slic3r::Printing::PrintingGroup &group : owner.print.printing_plan()->groups)
        for (const Slic3r::Printing::PrintingLayerGroup &layer : group.layers)
            for (const Slic3r::Printing::PrintingToolGroup &tool : layer.tool_groups)
                for (const Slic3r::Printing::PrintingExtrusion &extrusion : tool.extrusions)
                    if (extrusion.printing_catalog_id == SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE && extrusion.region_island != nullptr) {
                        auxiliary.region_island = slic3r_api::LayerRegionIsland(
                            reinterpret_cast<const layer_region_island_handle *>(extrusion.region_island));
                    }
    REQUIRE(auxiliary.region_island.valid());
    REQUIRE(auxiliary.region_island.region_count() > 0);
    const slic3r_api::Layer target_layer(reinterpret_cast<const layer_handle *>(&owner.print.object(0).layers().back()));
    travel::TravelEndpoint object;
    object.region_island = target_layer.island(0).get_or_create_full_region_island(0);
    object.printing_catalog_id = owner.print.mutable_printing_plan().find_or_add_object_instance(owner.print.object(0), 0);
    const Slic3r::Point local = owner.print.object(0).layers().back().lslices().front().contour.bounding_box().center();
    const c_point shift = owner.print_view.object(0).instance_shift(0);
    object.position = {local.x() + shift.x, local.y() + shift.y, 0, true};
    FixedZones contributor;
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    REQUIRE(compatibility.endpoint_enables_avoidance(object));
    CHECK_FALSE(compatibility.endpoint_enables_avoidance(auxiliary));
    CHECK(planner.eligible(auxiliary, object));
    CHECK(planner.eligible(object, auxiliary));
    CHECK(planner.eligible(auxiliary, auxiliary));
    travel::TravelEndpoint invalid = auxiliary;
    invalid.printing_catalog_id = uint32_t(owner.plan.object_instance_count());
    CHECK(planner.eligible(invalid, object));
    zones::ZonePlanner disabled_defaults(compatibility, view,
        owner.settings(), false, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    CHECK_FALSE(disabled_defaults.eligible(auxiliary, auxiliary));
    CHECK(disabled_defaults.eligible(auxiliary, object));
    REQUIRE(ac::is_off_object_layer(auxiliary.region_island.region(0).layer()));
    const Slic3r::BoundingBox box = owner.print.object(0).layers().back().lslices().front().contour.bounding_box();
    travel::PendingTravel gap;
    gap.source = gap.target = auxiliary;
    gap.source.position = {box.min.x() + shift.x - Slic3r::scale_i(2.), local.y() + shift.y, Slic3r::scale_i(0.6), true};
    gap.target.position = {box.max.x() + shift.x + Slic3r::scale_i(2.), local.y() + shift.y, Slic3r::scale_i(0.6), true};
    const work::ExPolygons blockers = ac::platter_blockers(owner.context.plugin_storage, owner.plan, 0, Slic3r::scale_i(0.6));
    REQUIRE_FALSE(blockers.empty());
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() > 2);
    CHECK(planner.domain_routes() == 0);
    CHECK(planner.fallbacks() == 0);
    check_clear(route, blockers, owner.storage);
}

// Outside endpoints never attach to nearby domains; platter geometry is cached separately from in-part hypotheses on the same chord.
TEST_CASE("Zone planner routes regionless travels around platter parts without domain attachments", "[plugins][zone-planner]")
{
    Owner owner;
    owner.slice();
    Slic3r::Printing::PrintingPlan &plan = owner.print.mutable_printing_plan();
    plan.groups.resize(1);
    plan.groups.front().printing_catalog_ids.clear();
    for (size_t instance = 0; instance < 2; ++instance)
        plan.groups.front().printing_catalog_ids.push_back(plan.find_or_add_object_instance(owner.print.object(0), instance));
    const Slic3r::Layer &layer = owner.print.object(0).layers().back();
    const Slic3r::BoundingBox box = layer.lslices().front().contour.bounding_box();
    const c_point shift = owner.print_view.object(0).instance_shift(0);
    const double x = Slic3r::unscaled(box.min.x() + shift.x), y = Slic3r::unscaled(box.min.y() + shift.y);
    const travel::PendingTravel outside = pending(x - 2., y + 10., x + 22., y + 10.);
    const travel::PendingTravel inside = owner.in_part(outside);
    REQUIRE(owner.plan.group_count() == 1);
    REQUIRE(owner.plan.group(0).object_instance_count() == 2);
    REQUIRE(owner.plan.object_instance_count() == 2);
    REQUIRE(owner.plan.object_instance(0).has_object());
    const slic3r_api::Object object_view = owner.plan.object_instance(0).object();
    REQUIRE(object_view.layer_count() > 0);
    INFO("blocker z=" << layer.scaled_print_z() << " first=" << object_view.layer(0).print_z()
        << " last=" << object_view.layer(object_view.layer_count() - 1).print_z());
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {rectangle(x - 3., y - 3., 26., 26.)}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, layer.scaled_print_z());
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache,
        owner.plan, 0, layer.scaled_print_z());
    const work::ExPolygons blockers = ac::platter_blockers(owner.context.plugin_storage, owner.plan, 0, layer.scaled_print_z());
    REQUIRE_FALSE(blockers.empty());
    const std::vector<c_point> route = planner.path(0, outside);
    REQUIRE(route.size() > 2);
    CHECK(planner.domain_routes() == 0);
    check_clear(route, blockers, owner.storage);
    CHECK(planner.path(1, inside).size() == 2);
    CHECK(planner.domain_routes() == 1);
    const std::vector<c_point> repeated = planner.path(2, outside);
    REQUIRE(repeated.size() == route.size());
    for (size_t i = 0; i < route.size(); ++i) {
        CHECK(repeated[i].x == route[i].x);
        CHECK(repeated[i].y == route[i].y);
    }
    CHECK(planner.domain_routes() == 1);
    CHECK(planner.fallbacks() == 0);
}

TEST_CASE("Zone planner applies auxiliary eligibility detour budgets and cancellation", "[plugins][zone-planner]")
{
    const char *budget = GENERATE("0", "0.1", "1%", "100", "1000%");
    Owner owner;
    owner.config.set_deserialize_strict({{"avoid_crossing_perimeters_max_detour", budget}});
    bool enabled = true, cancelled = false;
    SECTION("Enabled") {}
    SECTION("Disabled") { enabled = false; owner.config.set_deserialize_strict({{"avoid_crossing_perimeters", "0"}}); }
    SECTION("Cancelled") { cancelled = true; }
    owner.print.apply(owner.model, owner.config);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {rectangle(4., -2., 2., 4.)}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    const slic3r_api::Config defaults(print_get_default_region_config(owner.print_view.handle()));
    const bool auxiliary_avoidance = defaults.bool_or_default("avoid_crossing_perimeters", false);
    zones::ZonePlanner planner(compatibility, view, owner.settings(), auxiliary_avoidance, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const travel::PendingTravel gap = pending(0., 0., 10., 0.);
    CHECK(planner.eligible(gap.source, gap.target) == enabled);
    CHECK(zones::ZonePlanner::eligible(compatibility, owner.settings(), auxiliary_avoidance,
                                     gap.source, gap.target) == enabled);
    if (cancelled) {
        owner.cancel_at = GENERATE(size_t(1), size_t(10), size_t(20));
        CHECK_THROWS_AS(planner.path(0, gap), slic3r_api::PluginCancelled);
        return;
    }
    const std::vector<c_point> result = planner.path(0, gap);
    const bool rejected = std::string(budget) == "0.1" || std::string(budget) == "1%";
    CHECK((result.size() > 2) == (enabled && !rejected));
    CHECK(planner.fallbacks() == size_t(enabled && rejected));
}

// Check real top components independently so an endpoint's own top cannot erase a disjoint obstacle.
TEST_CASE("Default top contributors retain other tops when an endpoint occupies one", "[plugins][zone-planner][travel-zones]")
{
    Owner owner;
    const bool use_attractors = GENERATE(false, true);
    const bool use_tops = GENERATE(false, true);
    owner.model.clear_objects();
    Slic3r::TriangleMesh mesh = Slic3r::make_cube(4., 4., 0.6);
    Slic3r::TriangleMesh middle = Slic3r::make_cube(4., 4., 0.6);
    middle.translate(8., 0., 0.); mesh.merge(middle);
    Slic3r::TriangleMesh tower = Slic3r::make_cube(4., 4., 1.);
    tower.translate(16., 0., 0.); mesh.merge(tower);
    Slic3r::ModelObject *object = owner.model.add_object();
    object->add_volume(mesh, Slic3r::ModelVolumeType::MODEL_PART, false);
    object->add_instance();
    object->ensure_on_bed();
    owner.config.set_deserialize_strict({{"avoid_crossing_top", "1"}});
    owner.print.apply(owner.model, owner.config);
    owner.slice();
    owner.plan = slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&owner.print.mutable_printing_plan()));
    Slic3r::Layer &layer = owner.print.object(0).layers()[2];
    const slic3r_api::Layer layer_view(reinterpret_cast<const layer_handle *>(&layer));
    const Slic3r::Point minimum = Slic3r::get_extents(layer.lslices()).min;
    const c_point shift = owner.print_view.object(0).instance_shift(0);
    travel::PendingTravel gap = pending(2., 2., 18., 2.);
    gap.source.position.x += minimum.x() + shift.x; gap.source.position.y += minimum.y() + shift.y;
    gap.target.position.x += minimum.x() + shift.x; gap.target.position.y += minimum.y() + shift.y;
    const uint32_t id = owner.print.mutable_printing_plan().find_or_add_object_instance(owner.print.object(0), 0);
    gap.source.printing_catalog_id = gap.target.printing_catalog_id = id;
    for (size_t i = 0; i < layer.lslices().size(); ++i) {
        const Slic3r::ExPolygon &island = layer.lslices()[i];
        if (island.contains(minimum + Slic3r::Point(Slic3r::scale_i(2.), Slic3r::scale_i(2.))))
            gap.source.region_island = layer_view.island(i).get_or_create_full_region_island(0);
        if (island.contains(minimum + Slic3r::Point(Slic3r::scale_i(18.), Slic3r::scale_i(2.))))
            gap.target.region_island = layer_view.island(i).get_or_create_full_region_island(0);
    }
    REQUIRE(gap.source.region_island.valid());
    REQUIRE(gap.target.region_island.valid());
    owner.chronology.travels = {gap};
    const zones::DefaultAttractorContributor attractors;
    const zones::DefaultForbiddenTopsContributor tops;
    Slic3r::Test::TransitionPipeline::ScopedActivePlugins activation({
        "layer_extrusion_edit.travel.advanced",
        "layer_extrusion_edit.travel.advanced.zones.island_attractors",
        "layer_extrusion_edit.travel.advanced.zones.forbidden_tops"});
    Slic3r::Orchestrator &host = Slic3r::Orchestrator::instance();
    REQUIRE(host.set_plugin_active("layer_extrusion_edit.travel.advanced.zones.island_attractors", use_attractors));
    REQUIRE(host.set_plugin_active("layer_extrusion_edit.travel.advanced.zones.forbidden_tops", use_tops));
    std::vector<std::reference_wrapper<const zones::ZoneContributor>> contributors;
    if (orchestrator_find_plugin(reinterpret_cast<orchestrator_handle *>(&host),
        "layer_extrusion_edit.travel.advanced.zones.island_attractors")) contributors.push_back(std::cref(attractors));
    if (orchestrator_find_plugin(reinterpret_cast<orchestrator_handle *>(&host),
        "layer_extrusion_edit.travel.advanced.zones.forbidden_tops")) contributors.push_back(std::cref(tops));
    zones::TravelZoneView view(contributors, owner.input());
    const zones::ActiveZones &active = view.query(0, gap);
    CHECK(active.attractors.empty() == !use_attractors);
    REQUIRE(active.forbidden_indexes.size() == (use_tops ? 2 : 0));
    work::ExPolygons other_tops;
    size_t containing_source = 0;
    for (size_t index : active.forbidden_indexes) {
        const work::ExPolygons &area = view.zone_area(index);
        REQUIRE(area.size() == 1);
        if (area.front().contains({gap.source.position.x, gap.source.position.y})) ++containing_source;
        else other_tops.insert(other_tops.end(), area.begin(), area.end());
    }
    REQUIRE(containing_source == size_t(use_tops));
    REQUIRE(other_tops.size() == size_t(use_tops));
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> result = planner.path(0, gap);
    REQUIRE(is_detour(result) == use_tops);
    CHECK(planner.domain_routes() == size_t(use_attractors));
    if (use_tops) REQUIRE(result.size() > 2);
    else {
        double length = 0.;
        for (size_t p = 1; p < result.size(); ++p) length += work::Line(work::Point(result[p - 1]), work::Point(result[p])).length();
        CHECK(length == Approx(work::Line(work::Point(result.front()), work::Point(result.back())).length()).margin(2.));
    }
    if (use_tops) check_clear(result, other_tops, owner.storage);
}

// Only published forbidden zones restrict domain routing, even when regional top avoidance is enabled.
TEST_CASE("Domain routing does not inherit compatibility top avoidance", "[plugins][zone-planner][travel-zones]")
{
    Owner owner;
    owner.config.set_deserialize_strict("avoid_crossing_top", "1");
    // Keep a perimeter band wider than the crossing test's 1.5-nozzle-radius inset, independently of the perimeter generator.
    owner.config.set_deserialize_strict("external_perimeter_extrusion_width", "0.8");
    owner.print.apply(owner.model, owner.config);
    owner.slice();
    owner.plan = slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&owner.print.mutable_printing_plan()));
    Slic3r::Layer &layer = owner.print.object(0).layers().back();
    const slic3r_api::Layer layer_view(reinterpret_cast<const layer_handle *>(&layer));
    const uint32_t catalog = owner.print.mutable_printing_plan().find_or_add_object_instance(owner.print.object(0), 0);
    const c_point shift = owner.print_view.object(0).instance_shift(0);
    const Slic3r::Point minimum = layer.lslices().front().contour.bounding_box().min;
    // Keep endpoints in the perimeter band so the chord, not its endpoints, crosses the top.
    travel::PendingTravel gap = pending(0.2, 10., 19.8, 10.);
    gap.source.position.x += shift.x + minimum.x(); gap.source.position.y += shift.y + minimum.y();
    gap.target.position.x += shift.x + minimum.x(); gap.target.position.y += shift.y + minimum.y();
    gap.source.printing_catalog_id = gap.target.printing_catalog_id = catalog;
    gap.source.region_island = gap.target.region_island = layer_view.island(0).get_or_create_full_region_island(0);
    owner.chronology.travels = {gap};
    const zones::DefaultAttractorContributor attractors;
    zones::TravelZoneView view({std::cref(attractors)}, owner.input());
    CHECK(view.query(0, gap).forbidden_indexes.empty());
    REQUIRE_FALSE(view.query(0, gap).attractors.empty());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> expected = compatibility.path(gap.source, gap.target, 0);
    const std::vector<c_point> result = planner.path(0, gap);
    REQUIRE(compatibility.avoidance_enabled(gap.source, gap.target));
    REQUIRE(expected.size() > 2);
    double legacy_length = 0.;
    for (size_t p = 1; p < expected.size(); ++p)
        legacy_length += work::Line(work::Point(expected[p - 1]), work::Point(expected[p])).length();
    CHECK(legacy_length > work::Line(work::Point(gap.source.position.x, gap.source.position.y),
        work::Point(gap.target.position.x, gap.target.position.y)).length() + SCALED_EPSILON);
    double length = 0.;
    for (size_t p = 1; p < result.size(); ++p) length += work::Line(work::Point(result[p - 1]), work::Point(result[p])).length();
    CHECK(length == Approx(work::Line(work::Point(result.front()), work::Point(result.back())).length()).margin(2.));
    CHECK(result.front().x == gap.source.position.x);
    CHECK(result.back().x == gap.target.position.x);
    CHECK(planner.domain_routes() == 1);
}

TEST_CASE("Zone planner routes shifted domains but forbidden material wins", "[plugins][zone-planner]")
{
    Owner owner;
    owner.slice();
    owner.plan = slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&owner.print.mutable_printing_plan()));
    Slic3r::Layer &layer = owner.print.object(0).layers().back();
    const slic3r_api::Layer layer_view(reinterpret_cast<const layer_handle *>(&layer));
    const uint32_t catalog = owner.print.mutable_printing_plan().find_or_add_object_instance(owner.print.object(0), 1);
    const c_point shift = owner.print_view.object(0).instance_shift(1);
    const Slic3r::Point local_min = layer.lslices().front().contour.bounding_box().min;
    travel::PendingTravel gap = pending(2., 10., 18., 10.);
    gap.source.position.x += shift.x + local_min.x(); gap.source.position.y += shift.y + local_min.y();
    gap.target.position.x += shift.x + local_min.x(); gap.target.position.y += shift.y + local_min.y();
    gap.source.printing_catalog_id = gap.target.printing_catalog_id = catalog;
    gap.source.region_island = gap.target.region_island = layer_view.island(0).get_or_create_full_region_island(0);
    const int scenario = GENERATE(0, 1, 2);
    const bool forbidden = scenario != 0;
    const bool separate_attractors = scenario == 2;
    FixedZones contributor;
    work::ExPolygon attractor = rectangle(-1., -1., 22., 22.);
    for (work::Point &point : attractor.contour.points) { point.x() += shift.x + local_min.x(); point.y() += shift.y + local_min.y(); }
    contributor.areas.push_back({zones::ZoneRole::Attractor, {attractor}, 0, SIZE_MAX, {}});
    work::ExPolygon obstacle = rectangle(8., 8., 4., 4.);
    for (work::Point &point : obstacle.contour.points) { point.x() += shift.x + local_min.x(); point.y() += shift.y + local_min.y(); }
    if (separate_attractors) {
        const c_point source_shift = owner.print_view.object(0).instance_shift(0);
        gap.source.position.x += source_shift.x - shift.x;
        gap.source.position.y += source_shift.y - shift.y;
        gap.source.printing_catalog_id = owner.print.mutable_printing_plan().find_or_add_object_instance(owner.print.object(0), 0);
        work::ExPolygon source_attractor = attractor;
        for (work::Point &point : source_attractor.contour.points) {
            point.x() += source_shift.x - shift.x;
            point.y() += source_shift.y - shift.y;
        }
        contributor.areas.push_back({zones::ZoneRole::Attractor, {source_attractor}, 0, SIZE_MAX, {}});
        const coord_t center_x = (gap.source.position.x + gap.target.position.x) / 2;
        const coord_t center_y = (gap.source.position.y + gap.target.position.y) / 2;
        obstacle = rectangle(-2., -2., 4., 4.);
        for (work::Point &point : obstacle.contour.points) { point.x() += center_x; point.y() += center_y; }
    }
    if (forbidden) contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    contributor.areas[0].source_layer = reinterpret_cast<const layer_handle *>(&layer);
    contributor.areas[0].scope.printing_catalog_id = catalog;
    if (separate_attractors) {
        contributor.areas[1].source_layer = reinterpret_cast<const layer_handle *>(&layer);
        contributor.areas[1].scope.printing_catalog_id = gap.source.printing_catalog_id;
    }
    owner.chronology.travels = {gap};
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    REQUIRE(compatibility.avoidance_enabled(gap.source, gap.target));
    REQUIRE(planner.eligible(gap.source, gap.target));
    if (separate_attractors) {
        const zones::ActiveZones &active = view.query(0, gap);
        REQUIRE(active.attractors.size() == 2);
        for (const work::ExPolygon &area : active.attractors)
            CHECK_FALSE((area.contains(work::Point(gap.source.position.x, gap.source.position.y)) &&
                         area.contains(work::Point(gap.target.position.x, gap.target.position.y))));
    }
    const std::vector<c_point> expected = compatibility.path(gap.source, gap.target, 0);
    const std::vector<c_point> result = planner.path(0, gap);
    if (separate_attractors) {
        // A forbidden top between the two islands changes only the zone planner's route.
        bool compatibility_crosses_top = false;
        for (size_t i = 1; i < expected.size(); ++i) {
            work::Polyline segment;
            segment.points = {work::Point(expected[i - 1]), work::Point(expected[i])};
            const work::Polylines outside = slic3r_api::PluginCommon::clipper::diff_pl(
                owner.context.plugin_storage, {segment}, {obstacle});
            double retained = 0.;
            for (const work::Polyline &part : outside) retained += part.length();
            compatibility_crosses_top |= retained + 2. < segment.length();
        }
        CHECK(compatibility_crosses_top);
    }
    REQUIRE(result.size() >= 2);
    CHECK(result.front().x == gap.source.position.x);
    CHECK(result.back().x == gap.target.position.x);
    if (!forbidden) {
        REQUIRE(result.size() == 2);
        CHECK(planner.domain_routes() == 1);
    } else {
        CHECK(planner.domain_routes() == 1);
        CHECK(is_detour(result));
        CHECK(planner.fallbacks() == 0);
        check_clear(result, {obstacle}, owner.storage);
    }
    travel::TravelEndpoint auxiliary;
    CHECK(planner.eligible(auxiliary, gap.target));
    CHECK(planner.eligible(gap.source, auxiliary));
    travel::TravelEndpoint first_target = gap.target;
    const slic3r_api::Layer first_layer(reinterpret_cast<const layer_handle *>(&owner.print.object(0).layers().front()));
    first_target.region_island = first_layer.island(0).get_or_create_full_region_island(0);
    Slic3r::DynamicPrintConfig first_settings = owner.config;
    first_settings.set_deserialize_strict({{"avoid_crossing_not_first_layer", "1"}});
    const slic3r_api::Config first_excluded(Slic3r::ApiHost::to_config_handle(&first_settings));
    zones::ZonePlanner excluding(compatibility, view, first_excluded, true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    CHECK_FALSE(excluding.eligible(auxiliary, first_target));
    CHECK(excluding.eligible(first_target, auxiliary));
}
// Exercise synthetic domain containment and attachments using real normal-layer provenance.
TEST_CASE("Zone planner keeps interior points and leaves nearby free endpoints unattached", "[plugins][zone-planner]")
{
    Owner owner;
    FixedZones contributor;
    work::ExPolygon domain = rectangle(0., 0., 10., 10.);
    travel::PendingTravel gap = pending(2., 5., 8., 5.);
    bool direct = true, inside = true, near = false, far = false;
    SECTION("Interior segment") {}
    SECTION("Interior near the edge") { gap = pending(0.01, 0.01, 9.99, 0.01); }
    SECTION("Z-only with a nonempty domain") { gap = pending(2., 5., 2., 5.); gap.target.position.z = Slic3r::scale_i(0.2); }
    SECTION("Nearby free endpoint") { gap = pending(-0.1, 5., 8., 8.); inside = false; near = true; }
    SECTION("Both endpoints far use obstacle routing") {
        domain = rectangle(0., 100., 10., 10.);
        contributor.areas.push_back({zones::ZoneRole::Forbidden, {rectangle(4., 4., 2., 2.)}, 0, SIZE_MAX, {}});
        inside = false; direct = false; far = true;
    }
    SECTION("Forbidden hole is bypassed") {
        contributor.areas.push_back({zones::ZoneRole::Forbidden, {rectangle(4., 4., 2., 2.)}, 0, SIZE_MAX, {}});
        direct = false;
    }
    gap = owner.in_part(gap);
    contributor.areas.push_back({zones::ZoneRole::Attractor, {domain}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() >= 2);
    CHECK(route.front().x == gap.source.position.x);
    CHECK(route.front().y == gap.source.position.y);
    CHECK(route.back().x == gap.target.position.x);
    CHECK(route.back().y == gap.target.position.y);
    CHECK(planner.fallbacks() == 0);
    if (direct) CHECK(route.size() == 2);
    if (inside)
        for (size_t i = 1; i < route.size(); ++i) {
            work::Polyline line;
            line.points = {work::Point(route[i - 1]), work::Point(route[i])};
            const work::Polylines outside = slic3r_api::PluginCommon::clipper::diff_pl(owner.context.plugin_storage, {line}, {domain});
            double length = 0.;
            for (const work::Polyline &part : outside) length += part.length();
            CHECK(length <= 2.);
        }
    if (near) CHECK(planner.domain_routes() == 0);
    if (!direct) {
        REQUIRE(route.size() > 2);
        CHECK(planner.domain_routes() == size_t(!far));
        if (near) {
            CHECK(std::abs(route[1].x) <= Slic3r::SCALED_EPSILON);
            CHECK(route[1].y == Slic3r::scale_i(5.));
        } else check_clear(route, {rectangle(4., 4., 2., 2.)}, owner.storage);
    }
    const std::vector<c_point> repeated = planner.path(1, gap);
    REQUIRE(repeated.size() == route.size());
    for (size_t p = 0; p < route.size(); ++p) {
        CHECK(repeated[p].x == route[p].x);
        CHECK(repeated[p].y == route[p].y);
    }
}

// At w=0 the clipped outside length makes the mixed connector cheaper than the direct chord.
TEST_CASE("Zone planner weights an outside connector over an attractor", "[plugins][zone-planner]")
{
    const double weight = 0.;
    Owner owner;
    owner.config.set_deserialize_strict({{"avoid_travel_island", "1"},
                                         {"avoid_travel_island_weight", std::to_string(weight)}});
    owner.print.apply(owner.model, owner.config);
    owner.slice();
    const slic3r_api::Layer layer(reinterpret_cast<const layer_handle *>(&owner.print.object(0).layers().back()));
    travel::PendingTravel gap = owner.in_part(pending(0., 0., 10.3, 1.));
    const travel::PlannedPosition source_position = gap.source.position;
    gap.source = travel::TravelEndpoint{};
    gap.source.position = source_position;
    owner.chronology.travels = {gap};
    owner.print.mutable_printing_plan().groups.clear();
    owner.plan = slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(
        &owner.print.mutable_printing_plan()));
    const uint32_t catalog = gap.target.printing_catalog_id;
    const layer_handle *source_layer = gap.target.region_island.region(0).layer().handle();
    const work::ExPolygon domain = rectangle(10., -10., 20., 20.);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {domain}, 0, SIZE_MAX,
                                 {EXTRUDER_ID_UNKNOWN, catalog}, source_layer});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    const zones::ActiveZones &active = view.query(0, gap);
    REQUIRE(active.attractor_indexes.size() == 1);
    CHECK(view.zone(active.attractor_indexes.front()).source_layer == source_layer);
    CHECK(view.zone(active.attractor_indexes.front()).scope.printing_catalog_id == catalog);
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view,
        owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    CHECK(planner.eligible(gap.source, gap.target));
    const ac::Input &layer_input = owner.cache.get(owner.print_view, layer, &owner.context);
    REQUIRE(layer_input.target->regions.size() == 1);
    const work::ExPolygons target_w = withdrawn_components({domain}, 0.4,
        coord_t(1.5 * double(layer_input.target->perimeter_spacing)), owner.storage);
    REQUIRE(target_w.size() == 1);
    const work::Point source(gap.source.position.x, gap.source.position.y);
    const work::Point target(gap.target.position.x, gap.target.position.y);
    const work::Point entry = nearest_boundary_point(target_w.front(), source);
    const work::Point target_entry = nearest_boundary_point(target_w.front(), target);
    CHECK_FALSE(domain.contains(source, true));
    CHECK(domain.contains(target, true));
    CHECK_FALSE(target_w.front().contains(target, true));
    work::Polyline direct, mixed_outside_segment;
    direct.points = {source, target};
    mixed_outside_segment.points = {source, entry};
    const auto outside_length = [&owner, &domain](const work::Polyline &path) {
        double length = 0.;
        for (const work::Polyline &part : slic3r_api::PluginCommon::clipper::diff_pl(
                 owner.context.plugin_storage, {path}, {domain})) length += part.length();
        return length;
    };
    const double direct_outside = outside_length(direct);
    const double mixed_outside = outside_length(mixed_outside_segment);
    const double direct_length = direct.length();
    const double mixed_length = mixed_outside_segment.length();
    const double mixed_inside_length = entry.distance_to(target_entry) + target.distance_to(target_entry);
    // The helper withdraws this square to W at x=10.623494 mm. The typed estimates are:
    //   mixed: outside(S->entry) = 10.000000 mm, then 1.323494 mm of inside segments at w=0;
    //   no-domain: outside(S->T) = 10.047019 mm.
    // The old OutsidePart treatment would use the complete adjacent segments instead:
    //   mixed = |S->entry| = 10.623494 mm, no-domain = |S->T| = 10.348430 mm.
    // Thus the current formula selects the mixed route, while the old one selected the chord.
    CHECK(mixed_outside < direct_outside);
    CHECK(mixed_outside < direct_outside + Slic3r::SCALED_EPSILON);
    CHECK(mixed_inside_length > Slic3r::scale_i(1.));
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() > 2);
    CHECK(route.front().x == gap.source.position.x);
    CHECK(route.front().y == gap.source.position.y);
    CHECK(route.back().x == gap.target.position.x);
    CHECK(route.back().y == gap.target.position.y);
    CHECK(contains_exact_point(route, entry));
    work::Polyline planned;
    for (const c_point &point : route) planned.points.emplace_back(point);
    const double planned_outside = outside_length(planned);
    CHECK(planned_outside <= direct_outside + Slic3r::SCALED_EPSILON);
    CHECK(mixed_length > direct_length);
    CHECK(planner.domain_routes() == 1);
    CHECK(planner.fallbacks() == 0);
}

// A reference that crosses original forbidden material must not reject an otherwise valid domain route.
TEST_CASE("Zone planner accepts a domain route when the reference crosses its obstacle", "[plugins][zone-planner]")
{
    Owner owner;
    const work::ExPolygon obstacle = rectangle(4., -2., 2., 4.);
    const travel::PendingTravel gap = owner.in_part(pending(3.9, 0., 10., 0.));
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {rectangle(-2., -5., 14., 10.)}, 0, SIZE_MAX, {}});
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view,
        owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() > 2);
    CHECK(route.front().x == gap.source.position.x);
    CHECK(route.front().y == gap.source.position.y);
    CHECK(route.back().x == gap.target.position.x);
    CHECK(route.back().y == gap.target.position.y);
    CHECK(planner.domain_routes() == 1);
    CHECK(planner.fallbacks() == 0);
    check_clear(route, {obstacle}, owner.storage);
}

// A chord clear of original material remains a platter route when it only crosses Z4.
TEST_CASE("Zone planner keeps a clear chord through the expanded margin", "[plugins][zone-planner]")
{
    Owner owner;
    const work::ExPolygon obstacle = rectangle(4., 4., 2., 2.);
    const work::ExPolygons attractors{rectangle(0., 0., 4., 10.), rectangle(6., 0., 4., 10.)};
    const travel::PendingTravel gap = owner.in_part(pending(3.9, 3.9, 6.1, 3.9));
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, attractors, 0, SIZE_MAX, {}});
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache,
        owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    const work::ExPolygons expanded = slic3r_api::PluginCommon::clipper::offset_ex(
        owner.context.plugin_storage, {obstacle}, double(Slic3r::scale_i(0.2)));
    const work::Point source(gap.source.position.x, gap.source.position.y);
    const work::Point target(gap.target.position.x, gap.target.position.y);
    REQUIRE(expanded.size() == 1);
    REQUIRE(expanded.front().contains(source, false));
    REQUIRE(expanded.front().contains(target, false));
    work::Polyline direct;
    direct.points = {source, target};
    double direct_clear = 0.;
    for (const work::Polyline &part : slic3r_api::PluginCommon::clipper::diff_pl(
             owner.context.plugin_storage, {direct}, {obstacle})) direct_clear += part.length();
    const work::ExPolygons domains = slic3r_api::PluginCommon::clipper::diff_ex(
        owner.context.plugin_storage, attractors, expanded);
    work::Polyline planned;
    for (const c_point &point : route) planned.points.emplace_back(point);
    FixedZones reference_contributor;
    reference_contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    zones::TravelZoneView reference_view({std::cref(reference_contributor)}, owner.input());
    zones::ZonePlanner reference_planner(compatibility, reference_view, owner.settings(), true, &owner.context,
        owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    reference_planner.path(0, gap);
    double planned_outside = 0.;
    for (const work::Polyline &part : slic3r_api::PluginCommon::clipper::diff_pl(
             owner.context.plugin_storage, {planned}, domains)) planned_outside += part.length();
    CHECK(route.size() > 2);
    CHECK(direct_clear == Approx(direct.length()).margin(2.));
    CHECK(planner.domain_routes() == 0);
    CHECK(planner.fallbacks() == 0);
    CHECK(reference_planner.domain_routes() == 0);
    CHECK(reference_planner.fallbacks() == 0);
    CHECK(planned_outside > Slic3r::SCALED_EPSILON);
    check_clear(route, {obstacle}, owner.storage);
}

// A chord clear of original material remains direct when only its expanded margin is crossed.
TEST_CASE("Zone planner keeps a direct route through an expanded margin", "[plugins][zone-planner]")
{
    Owner owner;
    owner.config.set_deserialize_strict("nozzle_diameter", "0.399996,1.2");
    owner.print.apply(owner.model, owner.config);
    work::ExPolygon obstacle;
    obstacle.contour.points = {{0, 333330}, {2750004, 4000002}, {-2750004, 4000002}};
    const work::ExPolygon attractor = rectangle(-4., -1., 8., 6.);
    travel::PendingTravel gap = pending(0., 0., 0., 0.);
    gap.source.position = {-371, 665, 0, true};
    gap.target.position = {371, 665, 0, true};
    gap = owner.in_part(gap);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {attractor}, 0, SIZE_MAX, {}});
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache,
        owner.plan, 0, Slic3r::scale_i(0.6));
    const work::ExPolygons expanded = slic3r_api::PluginCommon::clipper::offset_ex(
        owner.context.plugin_storage, {obstacle}, double(Slic3r::scale_i(0.199998)));
    REQUIRE(expanded.size() == 1);
    const work::Point expanded_tip(0, 0);
    REQUIRE(std::find(expanded.front().contour.points.begin(), expanded.front().contour.points.end(), expanded_tip) !=
        expanded.front().contour.points.end());
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() == 2);
    CHECK(route.front().x == gap.source.position.x);
    CHECK(route.front().y == gap.source.position.y);
    CHECK(route.back().x == gap.target.position.x);
    CHECK(route.back().y == gap.target.position.y);
    CHECK(planner.fallbacks() == 0);
    CHECK(planner.domain_routes() == 1);
    check_clear(route, {obstacle}, owner.storage);
}

// Preserve an interior detour vertex when it passes near the travel's source endpoint.
TEST_CASE("Zone planner preserves an interior arc vertex near an endpoint", "[plugins][zone-planner]")
{
    Owner owner;
    owner.config.set_deserialize_strict("nozzle_diameter", "0.4,1.2");
    owner.print.apply(owner.model, owner.config);
    work::ExPolygon obstacle;
    obstacle.contour.points = {{-800000, -800000}, {800129, -800000}, {800129, 800000},
        {200129, 800000}, {200129, -200000}, {-200000, -200000}, {-200000, 9800000}, {-800000, 9800000}};
    travel::PendingTravel gap = pending(0., 0., 0., 0.);
    gap.source.position = {1, 1000000, 0, true};
    gap.target.position = {-500000, -2000000, 0, true};
    gap = owner.in_part(gap);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache,
        owner.plan, 0, Slic3r::scale_i(0.6));
    const work::ExPolygons expanded = slic3r_api::PluginCommon::clipper::offset_ex(
        owner.context.plugin_storage, {obstacle}, double(Slic3r::scale_i(0.2)));
    REQUIRE(expanded.size() == 1);
    const work::Points &contour = expanded.front().contour.points;
    const work::Point arc_start(129, 0), arc_vertex(129, 1000000);
    REQUIRE(std::find(contour.begin(), contour.end(), arc_start) != contour.end());
    REQUIRE(std::find(contour.begin(), contour.end(), arc_vertex) != contour.end());
    const work::Point source(gap.source.position.x, gap.source.position.y);
    CHECK(work::to_vec2d(source - arc_vertex).norm() <= double(Slic3r::SCALED_EPSILON + 2));
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() > 2);
    CHECK(route.front().x == gap.source.position.x);
    CHECK(route.front().y == gap.source.position.y);
    CHECK(route.back().x == gap.target.position.x);
    CHECK(route.back().y == gap.target.position.y);
    CHECK(planner.fallbacks() == 0);
    check_clear(route, {obstacle}, owner.storage);
}

// Keep an outside endpoint free while retaining a provenance-matched endpoint on M's border; the original M check admits its border link.
TEST_CASE("Zone planner requires M membership for endpoint candidates", "[plugins][zone-planner]")
{
    Owner owner;
    bool target_on_perimeter = false;
    SECTION("Outside endpoints stay without a domain") {}
    SECTION("Endpoint on perimeter reaches the nearest W point") { target_on_perimeter = true; }
    owner.config.set_deserialize_strict("avoid_travel_island", "1");
    owner.print.apply(owner.model, owner.config);
    owner.slice();
    const slic3r_api::Layer layer(reinterpret_cast<const layer_handle *>(&owner.print.object(0).layers().back()));
    const ac::Input &layer_input = owner.cache.get(owner.print_view, layer, &owner.context);
    travel::PendingTravel gap = pending(-0.1, 2., target_on_perimeter ? 0. : -0.1, 8.);
    gap.source.region_island = layer.island(0).get_or_create_full_region_island(0);
    gap.target.region_island = gap.source.region_island;
    gap.source.printing_catalog_id = owner.print.mutable_printing_plan().find_or_add_object_instance(owner.print.object(0), 0);
    gap.target.printing_catalog_id = gap.source.printing_catalog_id;
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {rectangle(0., 0., 10., 10.)}, 0, SIZE_MAX, {}});
    owner.chronology.travels = {gap};
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() >= 2);
    CHECK(route.front().x == gap.source.position.x);
    CHECK(route.front().y == gap.source.position.y);
    CHECK(route.back().x == gap.target.position.x);
    CHECK(route.back().y == gap.target.position.y);
    CHECK(planner.domain_routes() == size_t(target_on_perimeter));
    CHECK(planner.fallbacks() == 0);
    work::Polyline candidate;
    for (const c_point &point : route) candidate.points.emplace_back(point);
    const work::Polylines outside = slic3r_api::PluginCommon::clipper::diff_pl(
        owner.context.plugin_storage, {candidate}, {rectangle(0., 0., 10., 10.)});
    double outside_length = 0.;
    for (const work::Polyline &part : outside) outside_length += part.length();
    // With no obstacles and a chord outside the rectangle, the reference's outside length is the chord length.
    CHECK(outside_length <= work::Point(route.front()).distance_to(work::Point(route.back())) + Slic3r::SCALED_EPSILON);
    if (target_on_perimeter) {
        REQUIRE(route.size() >= 3);
        CHECK(planner.domain_routes() == 1);
        const work::ExPolygons withdrawn = withdrawn_components({rectangle(0., 0., 10., 10.)}, 0.4,
            coord_t(1.5 * double(layer_input.target->perimeter_spacing)), owner.storage);
        const work::Point expected = nearest_boundary_point(withdrawn.front(),
            work::Point(gap.target.position.x, gap.target.position.y));
        CHECK(contains_exact_point(route, expected));
    } else CHECK(route.size() == 2);
    const std::vector<c_point> repeated = planner.path(1, gap);
    REQUIRE(repeated.size() == route.size());
    for (size_t i = 0; i < route.size(); ++i) {
        CHECK(repeated[i].x == route[i].x);
        CHECK(repeated[i].y == route[i].y);
    }
}

// Use the configured contour as the fixed initial platter, including its concave border.
TEST_CASE("Zone planner confines platter routes to a concave bed shape", "[plugins][zone-planner]")
{
    Owner owner;
    owner.config.set_deserialize_strict("bed_shape", "0x0,20x0,20x10,10x10,10x20,0x20");
    owner.print.apply(owner.model, owner.config);
    const work::ExPolygon obstacle = rectangle(12., 7.5, 2., 2.);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view,
        owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    work::ExPolygon bed;
    for (const std::pair<double, double> &point : std::vector<std::pair<double, double>>{
             {0., 0.}, {20., 0.}, {20., 10.}, {10., 10.}, {10., 20.}, {0., 20.}})
        bed.contour.points.emplace_back(Slic3r::scale_i(point.first), Slic3r::scale_i(point.second));
    const std::vector<travel::PendingTravel> gaps{
        pending(2., 15., 18., 8.), pending(0., 10., 20., 10.), pending(0., 0., 0., 20.),
        pending(0., 0., 20., 10.)};
    for (size_t t = 0; t < gaps.size(); ++t) {
        const std::vector<c_point> route = planner.path(t, gaps[t]);
        REQUIRE((route.size() > 2) == (t == 0));
        CHECK(planner.fallbacks() == 0);
        check_contained(route, bed, owner.storage);
        const work::ExPolygons expanded = slic3r_api::PluginCommon::clipper::offset_ex(
            owner.context.plugin_storage, {obstacle}, double(Slic3r::scale_i(0.2)));
        check_clear(route, expanded, owner.storage);
    }
}

// A route whose endpoints lie on an oblique bed edge remains direct and exact.
TEST_CASE("Zone planner accepts a travel on a diagonal bed edge", "[plugins][zone-planner]")
{
    Owner owner;
    owner.config.set_deserialize_strict("bed_shape", "0x0,20x20,0x40,-20x20");
    owner.print.apply(owner.model, owner.config);
    FixedZones contributor;
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view,
        owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const travel::PendingTravel gap = pending(5., 5., 15., 15.);
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() == 2);
    CHECK(route.front().x == gap.source.position.x);
    CHECK(route.front().y == gap.source.position.y);
    CHECK(route.back().x == gap.target.position.x);
    CHECK(route.back().y == gap.target.position.y);
    CHECK(planner.fallbacks() == 0);
    CHECK(planner.domain_routes() == 0);
    work::ExPolygon bed;
    for (const std::pair<double, double> &point : std::vector<std::pair<double, double>>{
             {0., 0.}, {20., 20.}, {0., 40.}, {-20., 20.}})
        bed.contour.points.emplace_back(Slic3r::scale_i(point.first), Slic3r::scale_i(point.second));
    check_contained(route, bed, owner.storage);
}

// An endpoint outside the fixed contour returns the straight path and counts one fallback.
TEST_CASE("Zone planner falls back for an endpoint outside bed shape", "[plugins][zone-planner]")
{
    Owner owner;
    owner.config.set_deserialize_strict("bed_shape", "0x0,20x0,20x20,0x20");
    owner.print.apply(owner.model, owner.config);
    FixedZones contributor;
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view,
        owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const travel::PendingTravel gap = pending(2., 10., 21., 10.);
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() == 2);
    CHECK(route.front().x == gap.source.position.x);
    CHECK(route.back().x == gap.target.position.x);
    CHECK(planner.domain_routes() == 0);
    CHECK(planner.fallbacks() == 1);
}

// Joined nozzle margins split the free platter even though the original obstacles leave a narrow passage.
TEST_CASE("Zone planner rejects platter pairs with different components", "[plugins][zone-planner]")
{
    Owner owner;
    owner.config.set_deserialize_strict("bed_shape", "0x0,20x0,20x20,0x20");
    owner.print.apply(owner.model, owner.config);
    const work::ExPolygon lower = rectangle(9., 0., 2., 10.);
    const work::ExPolygon upper = rectangle(9., 10.2, 2., 9.8);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {lower, upper}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view,
        owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const travel::PendingTravel gap = pending(5., 10.1, 15., 10.1);
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() == 2);
    CHECK(planner.domain_routes() == 0);
    CHECK(planner.fallbacks() == 1);
}

// A nozzle margin that covers the whole fixed contour leaves W_p empty and removes platter candidates.
TEST_CASE("Zone planner has no platter candidate when Wp is empty", "[plugins][zone-planner]")
{
    Owner owner;
    owner.config.set_deserialize_strict("bed_shape", "0x0,20x0,20x20,0x20");
    owner.print.apply(owner.model, owner.config);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {rectangle(0.1, 0.1, 19.8, 19.8)}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view,
        owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const travel::PendingTravel gap = pending(0.05, 0.05, 19.95, 0.05);
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() == 2);
    CHECK(planner.domain_routes() == 0);
    CHECK(planner.fallbacks() == 1);
}

// Ignoring one endpoint component keeps a separate component of the same source zone in Z3.
TEST_CASE("Zone planner ignores only the endpoint obstacle group", "[plugins][zone-planner]")
{
    Owner owner;
    const travel::PendingTravel gap = owner.in_part(pending(5., 4.5, 20., 4.5));
    const work::ExPolygon endpoint_obstacle = rectangle(4., 4., 2., 2.);
    const work::ExPolygon retained_obstacle = rectangle(12., 4., 2., 2.);
    const work::ExPolygon exit_blocker = rectangle(4.9, 3.9, 0.2, 0.00075);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {endpoint_obstacle, retained_obstacle}, 0, SIZE_MAX, {}});
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {exit_blocker}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view,
        owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() > 2);
    CHECK(planner.domain_routes() == 0);
    CHECK(planner.fallbacks() == 0);
    check_clear(route, {retained_obstacle}, owner.storage);
    check_clear(route, {exit_blocker}, owner.storage);
    CHECK_FALSE(contains_exact_point(route, work::Point(Slic3r::scale_i(5.), Slic3r::scale_i(3.8))));
}

// Obstacle pieces that meet at a single vertex are ignored as one endpoint component.
TEST_CASE("Zone planner groups endpoint obstacles touching at a vertex", "[plugins][zone-planner]")
{
    Owner owner;
    owner.config.set_deserialize_strict("avoid_crossing_perimeters_max_detour", "0.01");
    owner.print.apply(owner.model, owner.config);
    const work::ExPolygon first = rectangle(4., 4., 2., 2.);
    const work::ExPolygon second = rectangle(6., 6., 2., 2.);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {first, second}, 0, SIZE_MAX, {}});
    const std::vector<travel::PendingTravel> gaps{
        owner.in_part(pending(5., 5., 9., 9.)), owner.in_part(pending(7., 7., 3., 3.))};
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view,
        owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    for (size_t t = 0; t < gaps.size(); ++t) {
        const std::vector<c_point> route = planner.path(t, gaps[t]);
        REQUIRE(route.size() == 2);
        CHECK(route.front().x == gaps[t].source.position.x);
        CHECK(route.back().x == gaps[t].target.position.x);
    }
    CHECK(planner.domain_routes() == 0);
    CHECK(planner.fallbacks() == 0);
}

// A hole vertex on the middle of another piece's edge joins the two endpoint obstacles.
TEST_CASE("Zone planner groups endpoint obstacles touching at a hole vertex", "[plugins][zone-planner]")
{
    Owner owner;
    work::ExPolygon first = rectangle(0., 0., 20., 20.);
    work::Polygon hole;
    for (const std::pair<double, double> &point : std::vector<std::pair<double, double>>{
             {2., 2.}, {18., 2.}, {18., 18.}, {2., 18.}, {2., 12.}, {10., 10.}, {2., 8.}})
        hole.points.emplace_back(Slic3r::scale_i(point.first), Slic3r::scale_i(point.second));
    std::reverse(hole.points.begin(), hole.points.end());
    first.holes.push_back(hole);
    const work::ExPolygon second = rectangle(10., 4., 6., 12.);
    const travel::PendingTravel gap = pending(5., 10., 25., 10.);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {first}, 0, SIZE_MAX, {}});
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {second}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view,
        owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() == 2);
    CHECK(route.front().x == gap.source.position.x);
    CHECK(route.back().x == gap.target.position.x);
    CHECK(planner.domain_routes() == 0);
    CHECK(planner.fallbacks() == 0);
}

// A zone and a translated platter piece that overlap are one ignored endpoint obstacle.
TEST_CASE("Zone planner groups an overlapping zone and platter piece", "[plugins][zone-planner]")
{
    Owner owner;
    owner.config.set_deserialize_strict("avoid_crossing_perimeters_max_detour", "0.01");
    owner.print.apply(owner.model, owner.config);
    owner.slice();
    Slic3r::Printing::PrintingPlan &plan = owner.print.mutable_printing_plan();
    plan.groups.resize(1);
    plan.groups.front().printing_catalog_ids = {plan.find_or_add_object_instance(owner.print.object(0), 0)};
    const Slic3r::Layer &layer = owner.print.object(0).layers().back();
    const work::ExPolygons blockers = ac::platter_blockers(owner.context.plugin_storage, owner.plan, 0,
        layer.scaled_print_z());
    REQUIRE(blockers.size() == 1);
    const work::BoundingBox box = blockers.front().contour.bounding_box();
    const double min_x = Slic3r::unscaled(box.min.x()), max_x = Slic3r::unscaled(box.max.x());
    const double center_y = Slic3r::unscaled((box.min.y() + box.max.y()) / 2);
    const work::ExPolygon zone = rectangle(min_x - 2., center_y - 1., 3., 2.);
    const travel::PendingTravel gap = pending(min_x - 1., center_y, max_x + 2., center_y);
    REQUIRE(zone.contains({gap.source.position.x, gap.source.position.y}, true));
    REQUIRE_FALSE(blockers.front().contains({gap.source.position.x, gap.source.position.y}, true));
    REQUIRE_FALSE(zone.contains({gap.target.position.x, gap.target.position.y}, true));
    REQUIRE(!slic3r_api::PluginCommon::clipper::intersection_ex(owner.context.plugin_storage,
        {zone}, blockers).empty());
    owner.chronology.travels = {gap};
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {zone}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, layer.scaled_print_z());
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view,
        owner.cache, owner.plan, 0, layer.scaled_print_z());
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() == 2);
    CHECK(planner.domain_routes() == 0);
    CHECK(planner.fallbacks() == 0);
}

// A mixed route follows W_a to the point nearest the free endpoint's reached W_p point.
TEST_CASE("Zone planner connects a mixed route to the platter link", "[plugins][zone-planner]")
{
    Owner owner;
    // Keep the mixed W_a/W_p candidate below the platter-only detour budget.
    owner.config.set_deserialize_strict({{"avoid_crossing_perimeters_max_detour", "0.12"}});
    owner.print.apply(owner.model, owner.config);
    const bool reverse = GENERATE(false, true);
    const travel::PendingTravel gap = owner.in_part(reverse ? pending(10.1, 5.1, 2., 5.) : pending(2., 5., 10.1, 5.1));
    const work::ExPolygon obstacle = rectangle(8., 3., 4., 2.);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {rectangle(0., 0., 8., 10.)}, 0, SIZE_MAX, {}});
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view,
        owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() > 2);
    CHECK(planner.domain_routes() == 1);
    CHECK(planner.fallbacks() == 0);
    const work::ExPolygons expanded = slic3r_api::PluginCommon::clipper::offset_ex(
        owner.context.plugin_storage, {obstacle}, double(Slic3r::scale_i(0.2)));
    const work::ExPolygons platter = slic3r_api::PluginCommon::clipper::diff_ex(
        owner.context.plugin_storage, {rectangle(-100., -100., 400., 400.)}, expanded);
    const work::Point free_point(reverse ? gap.source.position.x : gap.target.position.x,
                                 reverse ? gap.source.position.y : gap.target.position.y);
    work::Point expected = free_point;
    double nearest = (std::numeric_limits<double>::max)();
    for (const work::ExPolygon &component : platter) {
        const work::Point candidate = nearest_boundary_point(component, free_point);
        const double distance = free_point.distance_to(candidate);
        if (distance < nearest) { nearest = distance; expected = candidate; }
    }
    const work::ExPolygons attractor_w = withdrawn_components({rectangle(0., 0., 8., 10.)}, 0.4,
        coord_t(1.5 * double(owner.cache.get(owner.print_view,
            gap.source.region_island.region(0).layer(), &owner.context).target->perimeter_spacing)), owner.storage);
    REQUIRE(attractor_w.size() == 1);
    const work::Point expected_exit = nearest_boundary_point(attractor_w.front(), expected);
    const work::Point free_exit = nearest_boundary_point(attractor_w.front(), free_point);
    REQUIRE(expected_exit != free_exit);
    CHECK(contains_exact_point(route, expected_exit));
    CHECK_FALSE(contains_exact_point(route, free_exit));
    CHECK(contains_exact_point(route, expected));
    check_clear(route, {obstacle}, owner.storage);
}

// When both endpoints share one M_a component but their chord leaves it, platter pairs are removed.
TEST_CASE("Zone planner keeps the attractor route for a leaving same-component chord", "[plugins][zone-planner]")
{
    Owner owner;
    owner.config.set_deserialize_strict({{"avoid_travel_island", "1"}, {"avoid_travel_island_weight", "2"}});
    owner.print.apply(owner.model, owner.config);
    const travel::PendingTravel gap = owner.in_part(pending(1., 8., 9., 8.));
    work::ExPolygon u;
    for (const std::pair<double, double> &point : std::vector<std::pair<double, double>>{
             {0., 0.}, {10., 0.}, {10., 10.}, {7., 10.}, {7., 3.}, {3., 3.}, {3., 10.}, {0., 10.}})
        u.contour.points.emplace_back(Slic3r::scale_i(point.first), Slic3r::scale_i(point.second));
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {u}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view,
        owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() > 2);
    CHECK(planner.domain_routes() == 1);
    CHECK(planner.fallbacks() == 0);
    work::Polyline path;
    for (const c_point &point : route) path.points.emplace_back(point);
    double outside = 0.;
    for (const work::Polyline &part : slic3r_api::PluginCommon::clipper::diff_pl(
             owner.context.plugin_storage, {path}, {u})) outside += part.length();
    CHECK(outside <= Slic3r::SCALED_EPSILON);
}

// A blocked attractor connector is rejected instead of being counted as a domain route.
TEST_CASE("Zone planner rejects a blocked attractor candidate", "[plugins][zone-planner]")
{
    Owner owner;
    const travel::PendingTravel gap = owner.in_part(pending(1., 5., 9., 5.));
    const work::ExPolygon obstacle = rectangle(4.999, -1., 0.00075, 12.);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor,
                                 {rectangle(0., 0., 4., 10.), rectangle(6., 0., 4., 10.)}, 0, SIZE_MAX, {}});
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view,
        owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() == 2);
    CHECK(planner.domain_routes() == 0);
    CHECK(planner.fallbacks() == 1);
}

// Endpoints outside M do not attach to the hypothetical bridge on the same scene.
TEST_CASE("Zone planner leaves a clear chord when its bridge endpoints are outside M", "[plugins][zone-planner]")
{
    Owner owner;
    const work::ExPolygons domains{rectangle(-1., 0.5, 2., 2.), rectangle(9., 0.5, 2., 2.)};
    const work::ExPolygon obstacle = rectangle(4., 0.25, 2., 0.5);
    const travel::PendingTravel gap = owner.in_part(pending(0., 0., 10., 0.));
    const coord_t spacing = Slic3r::scale_i(0.4);
    ac::Boundary boundary;
    ac::build_boundary(owner.context.plugin_storage, work::ExPolygons(domains), 2 * spacing, boundary);
    for (const work::ExPolygon &domain : domains) boundary.boundary_growth.emplace_back(domain, domain);
    ac::RouterCounts counts;
    const work::Polyline candidate = ac::route_on_boundary(boundary, work::Point(gap.source.position.x, gap.source.position.y),
        work::Point(gap.target.position.x, gap.target.position.y), spacing, 2 * spacing, false, 0., owner.context.plugin_storage, counts, []() {});
    const work::Polylines outside = slic3r_api::PluginCommon::clipper::diff_pl(owner.context.plugin_storage, {candidate}, {obstacle});
    double retained = 0.;
    for (const work::Polyline &part : outside) retained += part.length();
    REQUIRE(retained + 2. < candidate.length());
    check_clear({{gap.source.position.x, gap.source.position.y}, {gap.target.position.x, gap.target.position.y}}, {obstacle}, owner.storage);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, domains, 0, SIZE_MAX, {}});
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    owner.chronology.travels = {gap};
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() == 2);
    CHECK(route.front().x == gap.source.position.x);
    CHECK(route.front().y == gap.source.position.y);
    CHECK(route.back().x == gap.target.position.x);
    CHECK(route.back().y == gap.target.position.y);
    CHECK(planner.fallbacks() == 0);
    CHECK(planner.domain_routes() == 0);
    CHECK_FALSE(is_detour(route));
    check_clear(route, {obstacle}, owner.storage);
}

// Reuse active zones while alternating endpoint membership, retaining the obstacle only in the matching cached domain.
TEST_CASE("Zone planner caches domains separately for ignored forbidden sets", "[plugins][zone-planner]")
{
    Owner owner;
    owner.in_part(pending(0., 0., 10., 0.));
    FixedZones contributor;
    const work::ExPolygon obstacle = rectangle(4., -2., 2., 4.);
    contributor.areas.push_back({zones::ZoneRole::Attractor, {rectangle(-2., -5., 14., 10.)}, 0, SIZE_MAX, {}});
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> ignored = planner.path(0, owner.in_part(pending(5., 0., 10., 0.)));
    const std::vector<c_point> route = planner.path(1, owner.in_part(pending(0., 0., 10., 0.)));
    const std::vector<c_point> ignored_again = planner.path(2, owner.in_part(pending(5., 0., 10., 0.)));
    const std::vector<c_point> repeated = planner.path(3, owner.in_part(pending(0., 0., 10., 0.)));
    REQUIRE(ignored.size() == 2);
    REQUIRE(ignored_again.size() == 2);
    REQUIRE(route.size() > 2);
    REQUIRE(repeated.size() == route.size());
    check_clear(route, {obstacle}, owner.storage);
    check_clear(repeated, {obstacle}, owner.storage);
    for (size_t point = 0; point < ignored.size(); ++point) {
        CHECK(ignored_again[point].x == ignored[point].x);
        CHECK(ignored_again[point].y == ignored[point].y);
    }
    for (size_t point = 0; point < route.size(); ++point) {
        CHECK(repeated[point].x == route[point].x);
        CHECK(repeated[point].y == route[point].y);
    }
    CHECK(planner.domain_routes() == 4);
    CHECK(planner.fallbacks() == 0);
}

// Reuse one platter obstacle with two tools, retaining each tool's nozzle margin in its own cache entry.
TEST_CASE("Zone planner caches platter margins separately for tools", "[plugins][zone-planner]")
{
    Owner owner;
    FixedZones contributor;
    const work::ExPolygon obstacle = rectangle(4., -2., 2., 4.);
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    const travel::PendingTravel gap = owner.in_part(pending(0., 0., 10., 0.));
    owner.chronology.travels = {gap, gap, gap};
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    travel::PendingTravel small_gap = gap;
    small_gap.extruder_id = 0;
    travel::PendingTravel large_gap = gap;
    large_gap.extruder_id = 1;
    const std::vector<c_point> route = planner.path(0, small_gap);
    const std::vector<c_point> larger = planner.path(1, large_gap);
    const std::vector<c_point> repeated = planner.path(2, small_gap);
    REQUIRE(route.size() > 2);
    REQUIRE(larger.size() > 2);
    REQUIRE(repeated.size() == route.size());
    check_clear(larger, {obstacle}, owner.storage);
    check_clear(route, {obstacle}, owner.storage);
    coord_t small_extent = 0, large_extent = 0;
    for (const c_point &point : route) small_extent = std::max(small_extent, std::abs(point.y));
    for (const c_point &point : larger) large_extent = std::max(large_extent, std::abs(point.y));
    CAPTURE(small_extent, large_extent);
    CHECK(large_extent > small_extent);
    for (size_t point = 0; point < route.size(); ++point) {
        CHECK(repeated[point].x == route[point].x);
        CHECK(repeated[point].y == route[point].y);
    }
    CHECK(planner.domain_routes() == 0);
    CHECK(planner.fallbacks() == 0);
}

// Reject an excessive complete domain path, bypass inactive routing, and propagate cancellation before publishing a route.
TEST_CASE("Zone planner applies budgets eligibility and cancellation to domain paths", "[plugins][zone-planner]")
{
    Owner owner;
    bool reject = false, enabled = true, cancelled = false;
    SECTION("Unlimited") {}
    SECTION("Rejected domain detour") { reject = true; owner.config.set_deserialize_strict("avoid_crossing_perimeters_max_detour", "0.1"); }
    SECTION("Disabled avoidance") { enabled = false; owner.config.set_deserialize_strict("avoid_crossing_perimeters", "0"); }
    SECTION("Cancelled domain construction") { cancelled = true; }
    owner.print.apply(owner.model, owner.config);
    const travel::PendingTravel gap = owner.in_part(pending(2., 5., 8., 5.));
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {rectangle(0., 0., 10., 10.)}, 0, SIZE_MAX, {}});
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {rectangle(4., 4., 2., 2.)}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), enabled, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    if (cancelled) {
        owner.checks = 0;
        owner.cancel_at = 3;
        CHECK_THROWS_AS(planner.path(0, gap), slic3r_api::PluginCancelled);
        return;
    }
    const std::vector<c_point> route = planner.path(0, gap);
    CHECK((route.size() > 2) == (enabled && !reject));
    CHECK(planner.fallbacks() == size_t(reject));
    CHECK(planner.domain_routes() == size_t(enabled && !reject));
}

// A 0.00075 mm wall is below Clipper's short-edge offset limit (0.005 times the
// 0.2 mm nozzle radius, or 0.001 mm), so the expanded mask is empty.
// The wall survives the smaller inward offset, and crossing that material is rejected.
TEST_CASE("Zone planner rejects a route crossing original material", "[plugins][zone-planner]")
{
    Owner owner;
    const work::ExPolygon obstacle = rectangle(4., -2., 0.00075, 4.);
    REQUIRE_FALSE(slic3r_api::PluginCommon::clipper::offset_ex(
        owner.context.plugin_storage, {obstacle}, -double(SCALED_EPSILON)).empty());
    const travel::PendingTravel gap = owner.in_part(pending(0., 0., 10., 0.));
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() == 2);
    CHECK(route.front().x == gap.source.position.x);
    CHECK(route.front().y == gap.source.position.y);
    CHECK(route.back().x == gap.target.position.x);
    CHECK(route.back().y == gap.target.position.y);
    CHECK(planner.fallbacks() == 1);
    CHECK(planner.domain_routes() == 0);
    CHECK_FALSE(is_detour(route));
}

// A wall thinner than twice the offset can disappear from the material check while remaining in the original union.
TEST_CASE("Zone planner accepts a crossing when the thin wall disappears from the material mask", "[plugins][zone-planner]")
{
    Owner owner;
    const work::ExPolygon obstacle = rectangle(4., -2., 0.00020, 4.);
    const work::ExPolygons original = slic3r_api::PluginCommon::clipper::union_ex(owner.context.plugin_storage, {obstacle});
    REQUIRE(original.size() == 1);
    REQUIRE(original.front().contains({Slic3r::scale_i(4.0001), 0}, false));
    REQUIRE(slic3r_api::PluginCommon::clipper::offset_ex(
        owner.context.plugin_storage, original, -double(SCALED_EPSILON)).empty());
    const travel::PendingTravel gap = owner.in_part(pending(0., 0., 10., 0.));
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view,
        owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() == 2);
    CHECK(work::Point(route.front()) == work::Point(gap.source.position.x, gap.source.position.y));
    CHECK(work::Point(route.back()) == work::Point(gap.target.position.x, gap.target.position.y));
    CHECK(planner.fallbacks() == 0);
}

// A thin wall can disappear from Z4, so the platter reference crosses the shrunken wall while the attractor route remains valid.
TEST_CASE("Zone planner ignores an invalid platter reference", "[plugins][zone-planner]")
{
    Owner owner;
    const work::ExPolygon attractor = rectangle(0., 0., 6., 6.);
    const work::ExPolygon wall = rectangle(5., 0., 0.00075, 5.);
    const travel::PendingTravel gap = owner.in_part(pending(2., 2., 8., 4.));
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {attractor}, 0, SIZE_MAX, {}});
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {wall}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view,
        owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const work::ExPolygons expanded = slic3r_api::PluginCommon::clipper::offset_ex(
        owner.context.plugin_storage, {wall}, double(Slic3r::scale_i(0.2)));
    REQUIRE(expanded.empty());
    work::Polyline direct;
    direct.points = {work::Point(gap.source.position.x, gap.source.position.y),
                     work::Point(gap.target.position.x, gap.target.position.y)};
    double direct_outside = 0.;
    for (const work::Polyline &part : slic3r_api::PluginCommon::clipper::diff_pl(
             owner.context.plugin_storage, {direct}, {wall})) direct_outside += part.length();
    CHECK(direct_outside > Slic3r::SCALED_EPSILON);
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() > 2);
    CHECK(planner.domain_routes() == 1);
    CHECK(planner.fallbacks() == 0);
    check_clear(route, {wall}, owner.storage);
    const work::ExPolygons m = slic3r_api::PluginCommon::clipper::diff_ex(
        owner.context.plugin_storage, {attractor}, {wall});
    work::Polyline rendered;
    for (const c_point &point : route) rendered.points.emplace_back(point);
    double route_outside = 0.;
    for (const work::Polyline &part : slic3r_api::PluginCommon::clipper::diff_pl(
             owner.context.plugin_storage, {rendered}, m)) route_outside += part.length();
    CAPTURE(route_outside);
    CHECK(route_outside > Slic3r::SCALED_EPSILON);
}

// Reject the upper bridge when its outside-M_a length exceeds the lower platter reference.
TEST_CASE("Zone planner rejects a bridge that exceeds the platter reference bound", "[plugins][zone-planner]")
{
    Owner owner;
    // With an exit, the bridge spends 24.147 mm outside the attractors, beyond the platter reference's 10.505 mm.
    // Keeping the obstacle scores 26.619 forward and 23.756 in reverse; ignoring it scores 34.000 in both directions.
    const bool with_exit = GENERATE(false, true);
    const bool reverse = GENERATE(false, true);
    owner.config.set_deserialize_strict({{"avoid_travel_island", "1"}, {"avoid_travel_island_weight", "0"},
                                         {"bed_shape", "-10x-10,60x-10,60x60,-10x60"}});
    owner.print.apply(owner.model, owner.config);
    owner.slice();
    work::ExPolygon left, right;
    for (const std::pair<double, double> &point : std::vector<std::pair<double, double>>{
             {0., 0.}, {10., 0.}, {10., 16.}, {14., 16.}, {14., 20.}, {0., 20.}})
        left.contour.points.emplace_back(Slic3r::scale_i(point.first), Slic3r::scale_i(point.second));
    for (const std::pair<double, double> &point : std::vector<std::pair<double, double>>{
             {30., 0.}, {30., 20.}, {16., 20.}, {16., 16.}, {20., 16.}, {20., 0.}})
        right.contour.points.emplace_back(Slic3r::scale_i(point.first), Slic3r::scale_i(point.second));
    const work::ExPolygon wall = rectangle(14.8, 8., 0.4, 22.);
    travel::PendingTravel gap = pending(with_exit ? 3. : 5., 4., 25., 4.);
    if (reverse) std::swap(gap.source, gap.target);
    gap = owner.in_part(gap);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {left}, 0, SIZE_MAX, {}});
    contributor.areas.push_back({zones::ZoneRole::Attractor, {right}, 0, SIZE_MAX, {}});
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {wall}, 0, SIZE_MAX, {}});
    const work::ExPolygon endpoint_obstacle = rectangle(2., 3.5, 7., 1.);
    if (with_exit)
        contributor.areas.push_back({zones::ZoneRole::Forbidden, {endpoint_obstacle}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view,
        owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() > 2);
    CHECK(planner.bridges() == 0);
    CHECK(planner.domain_routes() == 1);
    CHECK(planner.fallbacks() == 0);
    check_clear(route, {wall}, owner.storage);
    if (with_exit) {
        const c_point &reached = reverse ? route[route.size() - 2] : route[1];
        CHECK((reached.y <= Slic3r::scale_i(3.5) || reached.y >= Slic3r::scale_i(4.5)));
        check_exits(route, {wall, endpoint_obstacle}, {wall}, {wall}, !reverse, reverse, owner.storage);
        work::Polyline travelled;
        for (const c_point &point : route) travelled.points.emplace_back(point);
        double inside = travelled.length();
        for (const work::Polyline &part : slic3r_api::PluginCommon::clipper::diff_pl(
                 owner.context.plugin_storage, {travelled}, {endpoint_obstacle})) inside -= part.length();
        CHECK(inside <= Slic3r::scale_i(0.5) + Slic3r::SCALED_EPSILON);
    }
    for (const c_point &point : route) CHECK(point.y <= Slic3r::scale_i(7.8));
    const work::ExPolygons m = slic3r_api::PluginCommon::clipper::diff_ex(
        owner.context.plugin_storage, {left, right}, with_exit ? work::ExPolygons{wall, endpoint_obstacle} : work::ExPolygons{wall});
    const auto outside_m = [&owner, &m](const std::vector<c_point> &points) {
        work::Polyline path;
        for (const c_point &point : points) path.points.emplace_back(point);
        double length = 0.;
        for (const work::Polyline &part : slic3r_api::PluginCommon::clipper::diff_pl(
                 owner.context.plugin_storage, {path}, m)) length += part.length();
        return length;
    };
    const double route_outside = outside_m(route);
    // The accepted lower route measures 10.0 mm outside M_a, matching the platter reference.
    // Without this bound, the selected bridge measured 19.646988 mm outside M_a and rose above y=7.8 mm.
    if (!with_exit) {
        CHECK(route_outside <= Slic3r::scale_i(10.) + Slic3r::SCALED_EPSILON);
    } else {
        FixedZones forbidden_only;
        forbidden_only.areas.push_back({zones::ZoneRole::Forbidden, {wall, endpoint_obstacle}, 0, SIZE_MAX, {}});
        zones::TravelZoneView platter_view({std::cref(forbidden_only)}, owner.input());
        zones::ZonePlanner platter(compatibility, platter_view, owner.settings(), true, &owner.context,
            owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
        const std::vector<c_point> reference = platter.path(0, gap);
        REQUIRE(platter.fallbacks() == 0);
        REQUIRE(reference.size() > 2);
        const c_point &exit = reverse ? reference[reference.size() - 2] : reference[1];
        REQUIRE(exit.x == Slic3r::scale_i(3.));
        REQUIRE((exit.y == Slic3r::scale_i(3.3) || exit.y == Slic3r::scale_i(4.7)));
        check_exits(reference, {wall, endpoint_obstacle}, {wall}, {wall}, !reverse, reverse, owner.storage);
        CHECK(route_outside <= outside_m(reference) + Slic3r::SCALED_EPSILON);
    }
}

// The descending chord touches the wall's lower-left original vertex and then stays below the wall.
// The same short-edge limit leaves only the original-material contact for `blocked` to inspect.
TEST_CASE("Zone planner accepts contact at an original material vertex", "[plugins][zone-planner]")
{
    Owner owner;
    const work::ExPolygon obstacle = rectangle(4., 0., 0.00075, 4.);
    const travel::PendingTravel gap = owner.in_part(pending(0., 4., 10., -6.));
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view,
                               owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const work::ExPolygons expanded = slic3r_api::PluginCommon::clipper::offset_ex(
        owner.context.plugin_storage, {obstacle}, double(Slic3r::scale_i(0.2)));
    REQUIRE(expanded.empty());
    const work::Point source(gap.source.position.x, gap.source.position.y);
    const work::Point target(gap.target.position.x, gap.target.position.y);
    const work::Point corner(Slic3r::scale_i(4.), Slic3r::scale_i(0.));
    work::Point contact;
    REQUIRE(work::Line(source, target).distance_to_squared(corner, &contact) == 0.);
    REQUIRE(contact == corner);
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() == 2);
    CHECK(route.front().x == gap.source.position.x);
    CHECK(route.front().y == gap.source.position.y);
    CHECK(route.back().x == gap.target.position.x);
    CHECK(route.back().y == gap.target.position.y);
    CHECK(planner.fallbacks() == 0);
    CHECK_FALSE(is_detour(route));
    check_clear(route, {obstacle}, owner.storage);
}

// Keep the typed bridge when its selected points come from M_a after subtracting original Z3.
TEST_CASE("Zone planner contours bridges from original M_a boundaries", "[plugins][zone-planner]")
{
    Owner owner;
    work::ExPolygons attractors{rectangle(0., -5., 8.8, 4.8), rectangle(11.2, 0.5, 8.8, 4.5)};
    work::ExPolygon obstacle = rectangle(9., 0., 2., 2.);
    travel::PendingTravel gap = pending(2., -1., 18., 3.);
    bool vertex = false;
    SECTION("Bridge endpoints on contour edges") {}
    SECTION("Bridge starts at a contour vertex") {
        attractors = {rectangle(0., -5., 8.8, 4.8), rectangle(11.2, 0.5, 8.8, 4.5)};
        obstacle = rectangle(9., 0., 2., 2.);
        gap = pending(2., -1., 18., 1.);
        vertex = true;
    }
    gap = owner.in_part(gap);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, attractors, 0, SIZE_MAX, {}});
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    CAPTURE(vertex, route.size(), planner.bridges(), planner.domain_routes(), planner.fallbacks());
    REQUIRE(planner.bridges() == 1);
    CHECK(planner.domain_routes() == 1);
    CHECK(planner.fallbacks() == 0);
    check_clear(route, {obstacle}, owner.storage);
    const work::ExPolygons domains = slic3r_api::PluginCommon::clipper::diff_ex(
        owner.context.plugin_storage, attractors, {obstacle});
    if (vertex) {
        REQUIRE(domains.size() == 2);
        const ac::Input &layer_input = owner.cache.get(owner.print_view,
            gap.source.region_island.region(0).layer(), &owner.context);
        const work::ExPolygons withdrawn = withdrawn_components(domains, 0.4,
            coord_t(1.5 * double(layer_input.target->perimeter_spacing)), owner.storage);
        ac::Boundary w_boundary;
        ac::build_boundary(owner.context.plugin_storage, work::ExPolygons(withdrawn),
                           2 * layer_input.target->perimeter_spacing, w_boundary);
        work::Point start = nearest_boundary_point(withdrawn.front(),
            work::Point(gap.source.position.x, gap.source.position.y));
        work::Point end = nearest_boundary_point(withdrawn.back(),
            work::Point(gap.target.position.x, gap.target.position.y));
        ac::RouterCounts counts;
        const std::optional<ac::IslandBridge> expected_bridge = ac::select_island_bridge(
            w_boundary, 0, 1, start, end, 0., counts, []() {});
        REQUIRE(expected_bridge.has_value());
        CHECK(contains_exact_point(route, expected_bridge->from));
        CHECK(contains_exact_point(route, expected_bridge->to));
    }
    FixedZones reference_contributor;
    reference_contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    zones::TravelZoneView reference_view({std::cref(reference_contributor)}, owner.input());
    zones::ZonePlanner reference_planner(compatibility, reference_view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    work::Polyline path, reference;
    for (const c_point &point : route) path.points.emplace_back(point);
    for (const c_point &point : reference_planner.path(0, gap)) reference.points.emplace_back(point);
    REQUIRE(reference_planner.fallbacks() == 0);
    double outside = 0., reference_outside = 0.;
    for (const work::Polyline &part : slic3r_api::PluginCommon::clipper::diff_pl(owner.context.plugin_storage, {path}, domains)) outside += part.length();
    for (const work::Polyline &part : slic3r_api::PluginCommon::clipper::diff_pl(owner.context.plugin_storage, {reference}, domains)) reference_outside += part.length();
    CHECK(outside <= reference_outside + Slic3r::SCALED_EPSILON);
}

// At weight 0 the bridge is selected; at weights 1 and 2 the route keeps the platter option.
TEST_CASE("Zone planner contours selected bridges between component ring routes", "[plugins][zone-planner]")
{
    Owner owner;
    const double weight = GENERATE(0., 1., 2.);
    CAPTURE(weight);
    owner.config.set_deserialize_strict("avoid_travel_island", "1");
    owner.config.set_deserialize_strict("avoid_travel_island_weight", std::to_string(weight));
    owner.print.apply(owner.model, owner.config);
    owner.slice();
    const slic3r_api::Layer layer(reinterpret_cast<const layer_handle *>(&owner.print.object(0).layers().back()));
    travel::PendingTravel gap = pending(0.2, 1., 15.2, 9.);
    gap.source.region_island = gap.target.region_island = layer.island(0).get_or_create_full_region_island(0);
    gap.source.printing_catalog_id = gap.target.printing_catalog_id =
        owner.print.mutable_printing_plan().find_or_add_object_instance(owner.print.object(0), 0);
    const work::ExPolygons domains{rectangle(0., 0., 5., 10.), rectangle(15., 0., 5., 10.)};
    const work::ExPolygon obstacle = rectangle(9., 3., 2., 4.);
    const ac::Input &layer_input = owner.cache.get(owner.print_view, layer, &owner.context);
    const coord_t withdrawal = coord_t(1.5 * double(layer_input.target->perimeter_spacing));
    const work::ExPolygons withdrawn = withdrawn_components(domains,
        owner.settings().vector_float_or_default("nozzle_diameter", 0, 0.4), withdrawal, owner.storage);
    ac::Boundary boundary;
    ac::build_boundary(owner.context.plugin_storage, work::ExPolygons(withdrawn), 2 * layer_input.target->perimeter_spacing, boundary);
    work::Point start = nearest_boundary_point(withdrawn[0], work::Point(gap.source.position.x, gap.source.position.y));
    work::Point end = nearest_boundary_point(withdrawn[1], work::Point(gap.target.position.x, gap.target.position.y));
    ac::RouterCounts counts;
    const std::optional<ac::IslandBridge> bridge = ac::select_island_bridge(boundary, 0, 1, start, end, weight, counts, []() {});
    REQUIRE(bridge.has_value());
    work::Point zero_start(gap.source.position.x, gap.source.position.y), zero_end(gap.target.position.x, gap.target.position.y);
    const std::optional<ac::IslandBridge> zero = ac::select_island_bridge(boundary, 0, 1, zero_start, zero_end, 0., counts, []() {});
    REQUIRE(zero.has_value());
    if (weight > 0.) CHECK((bridge->from != zero->from || bridge->to != zero->to));
    work::Point high_start(gap.source.position.x, gap.source.position.y), high_end(gap.target.position.x, gap.target.position.y);
    const std::optional<ac::IslandBridge> high = ac::select_island_bridge(boundary, 0, 1, high_start, high_end, 2., counts, []() {});
    REQUIRE(high.has_value());
    CHECK((high->from != zero->from || high->to != zero->to));
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, domains, 0, SIZE_MAX, {}});
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    owner.chronology.travels = {gap};
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    const zones::ActiveZones &active = view.query(0, gap);
    REQUIRE(active.attractor_indexes.size() == 1);
    CHECK(view.zone(active.attractor_indexes.front()).source_layer == gap.source.region_island.region(0).layer().handle());
    CHECK(view.zone(active.attractor_indexes.front()).scope.printing_catalog_id == gap.source.printing_catalog_id);
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    const bool bridge_expected = weight == 0.;
    CHECK(planner.fallbacks() == 0);
    CHECK(planner.domain_routes() == size_t(bridge_expected));
    CHECK(planner.bridges() == size_t(bridge_expected));
    check_clear(route, {obstacle}, owner.storage);
    work::Polyline path;
    for (const c_point &point : route) path.points.emplace_back(point);
    if (bridge_expected) {
        const work::Points::const_iterator exit = std::find(path.points.cbegin(), path.points.cend(), bridge->from);
        const work::Points::const_iterator entry = std::find(path.points.cbegin(), path.points.cend(), bridge->to);
        REQUIRE(exit != path.points.cend());
        REQUIRE(entry != path.points.cend());
        REQUIRE(exit < entry);
        work::Polyline approach, departure;
        approach.points.assign(path.points.cbegin(), exit + 1);
        departure.points.assign(entry, path.points.cend());
        double approach_outside = 0., departure_outside = 0.;
        for (const work::Polyline &part : slic3r_api::PluginCommon::clipper::diff_pl(owner.context.plugin_storage, {approach}, {domains[0]}))
            approach_outside += part.length();
        for (const work::Polyline &part : slic3r_api::PluginCommon::clipper::diff_pl(owner.context.plugin_storage, {departure}, {domains[1]}))
            departure_outside += part.length();
        CHECK(approach_outside <= Slic3r::SCALED_EPSILON);
        CHECK(departure_outside <= Slic3r::SCALED_EPSILON);
    }
    FixedZones only_obstacle;
    only_obstacle.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    zones::TravelZoneView reference_view({std::cref(only_obstacle)}, owner.input());
    zones::ZonePlanner reference_planner(compatibility, reference_view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    work::Polyline reference;
    for (const c_point &point : reference_planner.path(0, gap)) reference.points.emplace_back(point);
    REQUIRE(reference_planner.fallbacks() == 0);
    double outside = 0., reference_outside = 0.;
    for (const work::Polyline &part : slic3r_api::PluginCommon::clipper::diff_pl(owner.context.plugin_storage, {path}, domains)) outside += part.length();
    for (const work::Polyline &part : slic3r_api::PluginCommon::clipper::diff_pl(owner.context.plugin_storage, {reference}, domains)) reference_outside += part.length();
    CHECK(outside <= reference_outside + Slic3r::SCALED_EPSILON);
}

// Retain the nearest W exit/entry when the other endpoint has no component, in both directions.
TEST_CASE("Zone planner connects a far free endpoint through the nearest W point", "[plugins][zone-planner]")
{
    Owner owner;
    const bool reverse = GENERATE(false, true);
    const travel::PendingTravel gap = owner.in_part(reverse ? pending(20., 9., 2., 9.) : pending(2., 9., 20., 9.));
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {rectangle(0., 0., 5., 10.)}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    const ac::Input &layer_input = owner.cache.get(owner.print_view,
        gap.source.region_island.region(0).layer(), &owner.context);
    const work::ExPolygons withdrawn = withdrawn_components({rectangle(0., 0., 5., 10.)},
        owner.settings().vector_float_or_default("nozzle_diameter", 0, 0.4),
        coord_t(1.5 * double(layer_input.target->perimeter_spacing)), owner.storage);
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() >= 3);
    CHECK(route.front().x == gap.source.position.x);
    CHECK(route.back().x == gap.target.position.x);
    const work::Point free_endpoint = reverse ? work::Point(gap.source.position.x, gap.source.position.y) :
        work::Point(gap.target.position.x, gap.target.position.y);
    const work::Point expected = nearest_boundary_point(withdrawn.front(), free_endpoint);
    CHECK(contains_exact_point(route, expected));
    for (const c_point &point : route) CHECK(point.y == Slic3r::scale_i(9.));
    CHECK(planner.domain_routes() == 1);
    CHECK(planner.bridges() == 0);
    CHECK(planner.fallbacks() == 0);
}

// Route a mixed travel when its selected in-part exit lies on the expanded forbidden border.
TEST_CASE("Zone planner detours a mixed connector from expanded forbidden material", "[plugins][zone-planner]")
{
    Owner owner;
    const work::ExPolygon obstacle = rectangle(8., -1., 4., 12.);
    const work::ExPolygon attractor = rectangle(0., 0., 8., 10.);
    const travel::PendingTravel gap = owner.in_part(pending(2., 5., 12.9, 5.));
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {attractor}, 0, SIZE_MAX, {}});
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache,
        owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    const work::Point expanded_edge(Slic3r::scale_i(7.8), Slic3r::scale_i(5.));
    REQUIRE(route.size() > 2);
    CHECK(planner.domain_routes() == 1);
    CHECK(planner.fallbacks() == 0);
    CHECK(std::find_if(route.begin(), route.end(), [&expanded_edge](const c_point &point) {
        return point.x == expanded_edge.x() && point.y == expanded_edge.y();
    }) != route.end());
    check_clear(route, {obstacle}, owner.storage);
}

// Skip a bridge whose box-based hole choice cannot seed its ring, then keep the source component's W link available.
TEST_CASE("Zone planner skips an unavailable bridge and tries a mixed hypothesis", "[plugins][zone-planner]")
{
    Owner owner;
    owner.config.set_deserialize_strict("avoid_travel_island_weight", "0");
    owner.print.apply(owner.model, owner.config);
    work::ExPolygon outer = rectangle(0., 0., 20., 20.);
    work::Polygon bent_hole;
    for (const std::pair<double, double> &point : {std::pair<double, double>{2., 2.}, {18., 2.}, {18., 4.}, {4., 4.}, {4., 18.}, {2., 18.}})
        bent_hole.points.emplace_back(Slic3r::scale_i(point.first), Slic3r::scale_i(point.second));
    std::reverse(bent_hole.points.begin(), bent_hole.points.end());
    work::Polygon near_hole = rectangle(10., 10., 4., 4.).contour;
    std::reverse(near_hole.points.begin(), near_hole.points.end());
    outer.holes = {bent_hole, near_hole};
    const work::ExPolygons domains = slic3r_api::PluginCommon::clipper::union_ex(owner.context.plugin_storage,
        {outer, rectangle(11., 11., 2., 2.)});
    const travel::PendingTravel gap = owner.in_part(pending(15., 12., 12., 12.));
    const ac::Input &layer_input = owner.cache.get(owner.print_view,
        gap.source.region_island.region(0).layer(), &owner.context);
    const work::ExPolygons withdrawn = withdrawn_components(domains,
        owner.settings().vector_float_or_default("nozzle_diameter", 0, 0.4),
        coord_t(1.5 * double(layer_input.target->perimeter_spacing)), owner.storage);
    ac::Boundary boundary;
    ac::build_boundary(owner.context.plugin_storage, work::ExPolygons(withdrawn),
        2 * layer_input.target->perimeter_spacing, boundary);
    size_t source_component = SIZE_MAX, target_component = SIZE_MAX, source_component_index = SIZE_MAX,
        target_component_index = SIZE_MAX, ring = 0, component_index = 0;
    for (const work::ExPolygon &component : domains) {
        if (component.contains({gap.source.position.x, gap.source.position.y})) {
            source_component = ring;
            source_component_index = component_index;
        }
        if (component.contains({gap.target.position.x, gap.target.position.y})) {
            target_component = ring;
            target_component_index = component_index;
        }
        ring += 1 + component.holes.size();
        ++component_index;
    }
    REQUIRE(source_component != SIZE_MAX);
    REQUIRE(target_component != SIZE_MAX);
    work::Point start = nearest_boundary_point(withdrawn[source_component_index],
        work::Point(gap.source.position.x, gap.source.position.y));
    work::Point end = nearest_boundary_point(withdrawn[target_component_index],
        work::Point(gap.target.position.x, gap.target.position.y));
    ac::RouterCounts counts;
    CHECK_FALSE(ac::select_island_bridge(boundary, source_component, target_component, start, end, 0., counts, []() {}).has_value());
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, domains, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() > 2);
    CHECK(route.front().x == gap.source.position.x);
    CHECK(route.front().y == gap.source.position.y);
    CHECK(route.back().x == gap.target.position.x);
    CHECK(route.back().y == gap.target.position.y);
    const work::Point expected_source_w = nearest_boundary_point(withdrawn[source_component_index],
        work::Point(gap.target.position.x, gap.target.position.y));
    CHECK(contains_exact_point(route, expected_source_w));
    CHECK(planner.bridges() == 0);
    CHECK(planner.domain_routes() == 1);
    CHECK(planner.fallbacks() == 0);
}

// Construct explicit advanced bridges with and without forbidden material between the domains.
TEST_CASE("Zone planner constructs bridges and validates forbidden material", "[plugins][zone-planner]")
{
    Owner owner;
    const bool forbidden = GENERATE(false, true);
    owner.config.set_deserialize_strict("avoid_travel_island", "1");
    // Zero interior weight keeps the two-domain bridge competitive with the platter chord.
    owner.config.set_deserialize_strict("avoid_travel_island_weight", "0");
    owner.print.apply(owner.model, owner.config);
    owner.slice();
    const slic3r_api::Layer layer(reinterpret_cast<const layer_handle *>(&owner.print.object(0).layers().back()));
    travel::PendingTravel gap = pending(2., 2., 18., 8.);
    gap.target.region_island = layer.island(0).get_or_create_full_region_island(0);
    gap.target.printing_catalog_id = owner.print.mutable_printing_plan().find_or_add_object_instance(owner.print.object(0), 0);
    gap.source.region_island = gap.target.region_island;
    gap.source.printing_catalog_id = gap.target.printing_catalog_id;
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {rectangle(0., 0., 5., 10.), rectangle(15., 0., 5., 10.)}, 0, SIZE_MAX, {}});
    const work::ExPolygon obstacle = rectangle(9., -1., 2., 12.);
    if (forbidden) contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    owner.chronology.travels = {gap};
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    CHECK(planner.bridges() == 1);
    CHECK(planner.fallbacks() == 0);
    CHECK(planner.domain_routes() == 1);
    if (forbidden) check_clear(route, {obstacle}, owner.storage);
}

// Accept a mixed connector when the selected ring exit and entry collapse to one point.
TEST_CASE("Zone planner accepts a mixed connector with coincident endpoints", "[plugins][zone-planner]")
{
    Owner owner;
    travel::PendingTravel gap = pending(0., 5., 2., 5.);
    const travel::PendingTravel provenance = owner.in_part(gap);
    gap.target.region_island = provenance.target.region_island;
    gap.target.printing_catalog_id = provenance.target.printing_catalog_id;
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {rectangle(0., 0., 10., 10.)}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache,
        owner.plan, 0, Slic3r::scale_i(0.6));
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() >= 2);
    CHECK(route.front().x == gap.source.position.x);
    CHECK(route.back().x == gap.target.position.x);
    CHECK(planner.domain_routes() == 1);
    CHECK(planner.fallbacks() == 0);
}

// Slice real support, then test platter group selection, translated slices and endpoint obstacle routing.
TEST_CASE("Support platter routing retains other components and printed instances", "[plugins][zone-planner][travel-zones]")
{
    Owner owner;
    owner.model.clear_objects();
    Slic3r::TriangleMesh mesh = Slic3r::make_cube(4., 8., 2.);
    Slic3r::TriangleMesh second = Slic3r::make_cube(4., 8., 2.);
    second.translate(12., 0., 0.); mesh.merge(second);
    Slic3r::TriangleMesh roof = Slic3r::make_cube(12., 8., 0.6);
    roof.translate(0., 0., 2.); mesh.merge(roof);
    Slic3r::ModelObject *object = owner.model.add_object();
    object->add_volume(mesh, Slic3r::ModelVolumeType::MODEL_PART, false);
    object->add_instance();
    object->add_instance()->set_offset(Slic3r::Vec3d(35., 0., 0.));
    object->ensure_on_bed();
    owner.config.set_deserialize_strict({{"support_material", "1"}, {"support_material_style", "grid"},
        {"support_material_extruder", "1"}, {"support_material_interface_extruder", "1"},
        {"support_material_contact_distance", "0.2"}, {"dont_support_bridges", "0"},
        {"nozzle_diameter", "0.4"}});
    owner.print.apply(owner.model, owner.config);
    owner.slice();
    owner.plan = slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&owner.print.mutable_printing_plan()));
    Slic3r::Orchestrator &host = Slic3r::Orchestrator::instance();
    owner.print.pipeline_progress().reset();
    Slic3r::Steps::StepExtrusionOrdering::clear(owner.print);
    Slic3r::Steps::StepExtrusionOrdering::prepare(owner.print);
    Slic3r::Steps::StepExtrusionOrdering::run_step(host, owner.print);
    owner.plan = slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&owner.print.mutable_printing_plan()));
    owner.print.pipeline_progress().reset();
    Slic3r::Test::TransitionPipeline::run_layer_plugins(owner.print, {
        "layer_extrusion_edit.transition_scope.default", "layer_extrusion_edit.entry_state.default"});
    travel::ScopeTravelConnector connector(reinterpret_cast<orchestrator_handle *>(&host));
    uint32_t layer_count = 0;
    Slic3r::Printing::PrintingPlan &plan = owner.print.mutable_printing_plan();
    for (const Slic3r::Printing::PrintingGroup &group : plan.groups)
        layer_count += uint32_t(group.layers.size());
    slic3r_api::PluginProgress progress;
    progress.reset(nullptr, layer_count, nullptr);
    connector.setup(owner.plan, layer_count);
    for (uint32_t g = 0; g < plan.groups.size(); ++g)
        for (uint32_t l = 0; l < plan.groups[g].layers.size(); ++l) {
            slic3r_api::PrintingLayerGroup layer_view(reinterpret_cast<printing_layer_group_handle *>(
                &plan.groups[g].layers[l]));
            connector.setup_run(layer_view, g, l, progress);
        }
    size_t real_support_gaps = 0, real_support_detours = 0, real_support_fallbacks = 0;
    // The pieces have already been united. Separate pieces can still meet at a ring boundary.
    const auto touches = [](const work::ExPolygon &a, const work::ExPolygon &b) {
        const auto meets_boundary = [](const work::ExPolygon &from, const work::ExPolygon &to) {
            for (size_t i = 0; i <= from.holes.size(); ++i) {
                const work::Polygon &ring = i == 0 ? from.contour : from.holes[i - 1];
                for (const work::Point &point : ring.points)
                    for (size_t j = 0; j <= to.holes.size(); ++j) {
                        const work::Polygon &boundary = j == 0 ? to.contour : to.holes[j - 1];
                        if (work::contains(boundary, point, true) && !work::contains(boundary, point, false))
                            return true;
                    }
            }
            return false;
        };
        return meets_boundary(a, b) || meets_boundary(b, a);
    };
    ac::CompatibilityPlanner real_compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    for (uint32_t g = 0; g < plan.groups.size(); ++g) {
        for (uint32_t l = 0; l < plan.groups[g].layers.size(); ++l) {
            slic3r_api::PrintingLayerGroup layer_view(reinterpret_cast<printing_layer_group_handle *>(
                &plan.groups[g].layers[l]));
            const travel::LayerTravelChronology chronology = connector.prepare(layer_view, g, l);
            const work::ExPolygons blockers = ac::platter_blockers(owner.context.plugin_storage, owner.plan, g, layer_view.print_z());
            zones::TravelZoneView real_view({}, owner.input());
            zones::ZonePlanner real_planner(real_compatibility, real_view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, g, layer_view.print_z());
            for (size_t t = 0; t < chronology.travels.size(); ++t) {
                const travel::PendingTravel &gap = chronology.travels[t];
                if (!gap.source_extrusion.valid() ||
                    !(RAW_EXTRUSION_ROLE_IS_SUPPORT(gap.source_extrusion.role()) ||
                      RAW_EXTRUSION_ROLE_IS_SUPPORT(gap.target_extrusion.role())))
                    continue;
                REQUIRE(real_planner.eligible(gap.source, gap.target));
                const size_t before = real_planner.fallbacks();
                const std::vector<c_point> route = real_planner.path(t, gap);
                CHECK(route.size() >= 2);
                if (real_planner.fallbacks() == before) {
                    std::vector<bool> grouped(blockers.size(), false);
                    std::vector<size_t> queue;
                    for (size_t i = 0; i < blockers.size(); ++i)
                        if (blockers[i].contains({gap.source.position.x, gap.source.position.y}, true) ||
                            blockers[i].contains({gap.target.position.x, gap.target.position.y}, true)) {
                            grouped[i] = true;
                            queue.push_back(i);
                        }
                    for (size_t seed = 0; seed < queue.size(); ++seed)
                        for (size_t i = 0; i < blockers.size(); ++i)
                            if (!grouped[i] && touches(blockers[queue[seed]], blockers[i])) {
                                grouped[i] = true;
                                queue.push_back(i);
                            }
                    work::ExPolygons remaining;
                    for (size_t i = 0; i < blockers.size(); ++i)
                        if (!grouped[i]) remaining.push_back(blockers[i]);
                    CAPTURE(g, l, t, route.size(), is_detour(route));
                    check_clear(route, remaining, owner.storage);
                } else {
                    ++real_support_fallbacks;
                    CHECK(route.size() == 2);
                }
                if (is_detour(route)) ++real_support_detours;
                ++real_support_gaps;
            }
        }
    }
    REQUIRE(real_support_gaps > 0);
    REQUIRE(real_support_detours > 0);
    if (real_support_fallbacks != 0)
        WARN("Straight support fallbacks: " << real_support_fallbacks);
    slic3r_api::PrintingExtrusion support(nullptr);
    for (Slic3r::Printing::PrintingGroup &group : owner.print.mutable_printing_plan().groups)
        for (Slic3r::Printing::PrintingLayerGroup &layer : group.layers)
            for (Slic3r::Printing::PrintingToolGroup &tool : layer.tool_groups)
                for (Slic3r::Printing::PrintingExtrusion &pe : tool.extrusions)
                    if (pe.sregion_island_role.is_support())
                        support = slic3r_api::PrintingExtrusion(reinterpret_cast<printing_extrusion_handle *>(&pe));
    REQUIRE(support.valid());
    const Slic3r::Layer &layer = owner.print.object(0).layers()[2];
    REQUIRE(layer.lslices().size() == 2);
    // A group 0 query excludes group 1, which is still in the future.
    REQUIRE(plan.object_instances.size() == 2);
    plan.groups.resize(2);
    for (uint32_t instance = 0; instance < 2; ++instance)
        plan.groups[instance].printing_catalog_ids = {plan.find_or_add_object_instance(owner.print.object(0), instance)};
    const auto blockers_at = [&](uint32_t group, coord_t z) {
        return ac::platter_blockers(owner.context.plugin_storage, owner.plan, group, z);
    };
    const work::ExPolygons first_group = blockers_at(0, layer.scaled_print_z());
    REQUIRE(first_group.size() == 2);
    const work::ExPolygons exact = blockers_at(1, layer.scaled_print_z());
    REQUIRE(exact.size() == 4);
    const work::ExPolygons between = blockers_at(1, layer.scaled_print_z() + Slic3r::scale_i(0.05));
    // Compare filled areas at Clipper's tolerance, not exact vertex coordinates or order.
    REQUIRE(between.size() == exact.size());
    CHECK(slic3r_api::PluginCommon::clipper::diff_ex(owner.context.plugin_storage, between, exact).empty());
    CHECK(slic3r_api::PluginCommon::clipper::diff_ex(owner.context.plugin_storage, exact, between).empty());
    CHECK(blockers_at(1, 0).empty());
    work::ExPolygons expected_all;
    for (uint32_t instance = 0; instance < 2; ++instance) {
        const c_point shift = owner.print_view.object(0).instance_shift(instance);
        Slic3r::ExPolygons translated = layer.lslices();
        for (Slic3r::ExPolygon &slice : translated)
            slice.translate(Slic3r::Point(shift.x, shift.y));
        const work::ExPolygons expected = slic3r_api::PluginCommon::clipper::from_expolygon_collection_view(
            slic3r_api::ExPolygonCollection(reinterpret_cast<const expolygon_collection_handle *>(&translated)));
        CHECK(slic3r_api::PluginCommon::clipper::diff_ex(owner.context.plugin_storage, expected, first_group).empty() == (instance == 0));
        CHECK(slic3r_api::PluginCommon::clipper::diff_ex(owner.context.plugin_storage, expected, exact).empty());
        if (instance == 0)
            CHECK(slic3r_api::PluginCommon::clipper::diff_ex(owner.context.plugin_storage, first_group, expected).empty());
        expected_all.insert(expected_all.end(), expected.begin(), expected.end());
    }
    CHECK(slic3r_api::PluginCommon::clipper::diff_ex(owner.context.plugin_storage, exact, expected_all).empty());
    // The roof replaces two pillar components at the first layer above the pillars.
    const coord_t roof_z = owner.print.object(0).layers()[10].scaled_print_z();
    const work::ExPolygons roof_exact = blockers_at(1, roof_z);
    const work::ExPolygons below_roof = blockers_at(1, roof_z - Slic3r::scale_i(0.1));
    REQUIRE(roof_exact.size() == 2);
    REQUIRE(below_roof.size() == 4);
    const Slic3r::Point minimum = Slic3r::get_extents(layer.lslices()).min;
    const c_point shift = owner.print_view.object(0).instance_shift(0);
    const bool target_object = GENERATE(false, true);
    travel::PendingTravel gap = pending(-5., 4., target_object ? 49. : 56., 4.);
    gap.source.position.x += minimum.x() + shift.x;
    gap.source.position.y += minimum.y() + shift.y;
    gap.target.position.x += minimum.x() + shift.x;
    gap.target.position.y += minimum.y() + shift.y;
    gap.source_extrusion = gap.target_extrusion = support;
    gap.source.region_island = gap.target.region_island = support.region_island();
    gap.source.printing_catalog_id = gap.target.printing_catalog_id = support.printing_catalog_id();
    if (target_object) {
        const slic3r_api::Layer view(reinterpret_cast<const layer_handle *>(&layer));
        gap.target.printing_catalog_id = owner.print.mutable_printing_plan().find_or_add_object_instance(owner.print.object(0), 1);
        for (size_t i = 0; i < layer.lslices().size(); ++i)
            if (layer.lslices()[i].contains(minimum + Slic3r::Point(Slic3r::scale_i(14.), Slic3r::scale_i(4.))))
                gap.target.region_island = view.island(i).get_or_create_full_region_island(0);
        REQUIRE(gap.target.region_island.valid());
        gap.target_extrusion = slic3r_api::PrintingExtrusion(nullptr);
    }
    FixedZones domain_contributor;
    if (target_object)
        for (const work::ExPolygon &piece : exact)
            if (piece.contains({gap.target.position.x, gap.target.position.y}))
                domain_contributor.areas.push_back({zones::ZoneRole::Attractor,
                    slic3r_api::PluginCommon::clipper::offset_ex(owner.context.plugin_storage, {piece}, -double(Slic3r::scale_i(0.4))),
                    0, SIZE_MAX, {}});
    for (zones::TravelZone &zone : domain_contributor.areas) {
        zone.source_layer = gap.target.region_island.region(0).layer().handle();
        zone.scope.printing_catalog_id = gap.target.printing_catalog_id;
    }
    owner.chronology.travels = {gap};
    const bool reverse = GENERATE(false, true);
    if (reverse) {
        std::swap(gap.source, gap.target);
        std::swap(gap.source_extrusion, gap.target_extrusion);
    }
    zones::TravelZoneView view({std::cref(domain_contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache, owner.plan, 1, layer.scaled_print_z());
    REQUIRE(planner.eligible(gap.source, gap.target));
    const std::vector<c_point> result = planner.path(0, gap);
    REQUIRE(result.size() > 2);
    CHECK(planner.fallbacks() == 0);
    // The platter exit wins because the score counts travel through the endpoint piece three times.
    CHECK(planner.domain_routes() == 0);
    work::ExPolygons obstacles;
    for (const work::ExPolygon &piece : exact)
        if (!piece.contains({gap.target.position.x, gap.target.position.y}, true) &&
            !piece.contains({gap.source.position.x, gap.source.position.y}, true))
            obstacles.push_back(piece);
    REQUIRE(obstacles.size() == (target_object ? 3 : 4));
    check_clear(result, obstacles, owner.storage);
    // The warm layer and zone caches leave the third check for the fresh platter collection.
    zones::ZonePlanner fresh(compatibility, view, owner.settings(), true, &owner.context, owner.print_view,
        owner.cache, owner.plan, 1, layer.scaled_print_z());
    owner.checks = 0;
    owner.cancel_at = 3;
    std::vector<c_point> resumed;
    CHECK_THROWS_AS(resumed = fresh.path(1, gap), slic3r_api::PluginCancelled);
    CHECK(resumed.empty());
    owner.cancel_at = 0;
    resumed = fresh.path(1, gap);
    REQUIRE(resumed.size() == result.size());
    for (size_t i = 0; i < result.size(); ++i)
        CHECK(work::Point(resumed[i]) == work::Point(result[i]));
    CHECK(fresh.fallbacks() == 0);
}
} // namespace
