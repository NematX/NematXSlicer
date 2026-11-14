#include <catch2/catch.hpp>
#include <algorithm>

#include "layer_extrusion_edit_transition_test_helpers.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/AdvancedTravel/ZonePlanner.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/AdvancedTravel/DefaultZoneContributors.hpp"
#include "libslic3r/Plugins/Common/ClipperHelpers.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Api/host/ApiHostUtils.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Steps/StepExtrusionOrdering.hpp"
#include "test_data.hpp"

/*
Zone planner tests supply small dated platter-space areas through a contributor.
Local print and storage owners keep public views alive. Auxiliary travels isolate
obstacle routing; a sliced cube supplies real regional provenance and shifted
instances for domain routing. Segment checks inspect filled material,
not just route vertices. The fixture selects travel.advanced to keep its settings
active while slicing, but invokes ZonePlanner directly without production wiring.
Ranked candidate checks use real layer weights with neighbouring perimeter endpoints,
nearest ring attachments, budget rejection followed by the option without a domain,
and repeat queries to check deterministic ordering.
An endpoint in the expanded nozzle margin checks that an unusable obstacle-only
reference does not reject a safe domain route around the original material.
Bridge cases retain the selected exit and entry points, contour a forbidden top,
and check the component approaches and the reference outside-length bound.
An obstacle splitting one attractor places bridge endpoints exactly on its expanded
contour; an offset pair of components places a bridge endpoint on a contour vertex.
A thin obstacle across an attachment, but clear of the chord and its margin,
checks rejection of that mixed hypothesis followed by the chord without a domain.
Mixed cases retain the nearest ring point between a component and a far free endpoint.
A bent hole's box overlaps a second hole containing an island; an unavailable
box-selected bridge must leave the mixed hypotheses available.
Support obstacle checks slice an overhanging roof and two posts, retaining real support
printing extrusions. Controlled endpoint chords test each shifted component;
the support roles are read from the sliced plan, not synthesized.

    Owner: own print, geometry storage, cancellation and contributor inputs
    FixedZones::contribute: copy synthetic dated areas
    rectangle / pending: construct millimeter geometry in platter coordinates
    check_clear: compare each route segment with original filled areas
    tests: exercise eligibility, cached margins, domain routing, jumps and limits;
        real default contributors retain disjoint tops when an endpoint occupies another top;
        activation checks inspect collected zones and routing without inherited legacy top avoidance;
        a wide external perimeter preserves a real legacy top-detour witness after crossing-test inset
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

struct FixedZones : zones::ZoneContributor {
    std::vector<zones::TravelZone> areas;
    // Copy synthetic owned zones without retaining contributor input views.
    void contribute(const zones::ZoneContributorInput &, std::vector<zones::TravelZone> &out) const override
    { out.insert(out.end(), areas.begin(), areas.end()); }
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
    zones::ChronologySummary summary;
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
            {"avoid_crossing_perimeters_max_detour", "0"}, {"nozzle_diameter", "0.4,1.2"}});
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
    // Return contributor inputs borrowing this owner until its destruction.
    zones::ZoneContributorInput input()
    { return {print_view, plan, context, Slic3r::scale_i(0.6), cache, chronology.travels, summary}; }
    // Borrow applied print settings; auxiliary avoidance is passed separately to the planner.
    slic3r_api::Config settings() const
    { return print_view.config(); }
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
    SECTION("Tangent vertex of expanded obstacle") { gap = pending(0., -8.4, 10., 1.6); expect_detour = false; }
    SECTION("Segment along expanded edge") { gap = pending(0., -2.2, 10., -2.2); expect_detour = false; }
    for (const work::ExPolygon &area : areas)
        contributor.areas.push_back({zones::ZoneRole::Forbidden, {area}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache);
    const std::vector<c_point> result = planner.path(0, gap);
    REQUIRE(result.size() >= 2);
    CHECK(result.front().x == gap.source.position.x);
    CHECK(result.front().y == gap.source.position.y);
    CHECK(result.back().x == gap.target.position.x);
    CHECK(result.back().y == gap.target.position.y);
    CHECK(planner.fallbacks() == 0);
    CHECK(planner.detours() == size_t(expect_detour));
    work::ExPolygons remaining;
    for (const work::ExPolygon &area : areas)
        if (!area.contains(work::Point(result.front())) && !area.contains(work::Point(result.back()))) remaining.push_back(area);
    check_clear(result, remaining, owner.storage);
}

TEST_CASE("Zone planner caches tool margins and endpoint ignored sets independently", "[plugins][zone-planner]")
{
    Owner owner;
    FixedZones contributor;
    const work::ExPolygon area = rectangle(4., -2., 2., 4.);
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {area}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache);
    CHECK(planner.path(0, pending(5., 0., 10., 0.)).size() == 2);
    const std::vector<c_point> small = planner.path(1, pending(0., 0., 10., 0., 0));
    const std::vector<c_point> large = planner.path(2, pending(0., 0., 10., 0., 1));
    REQUIRE(small.size() > 2);
    REQUIRE(large.size() > 2);
    CHECK(std::abs(large[2].y) > std::abs(small[2].y));
    CHECK(view.query(2, pending(0., 0., 10., 0.)).id == view.query(2, pending(0., 0., 10., 0., 1)).id);
    CHECK(planner.detours() == 2);
    CHECK(planner.fallbacks() == 0);
    check_clear(small, {area}, owner.storage);
    check_clear(large, {area}, owner.storage);
}

TEST_CASE("Zone planner counts fallback when either endpoint lies in an obstacle hole", "[plugins][zone-planner]")
{
    Owner owner;
    work::ExPolygon ring = rectangle(2., -4., 6., 8.);
    work::Polygon hole = rectangle(3., -3., 4., 6.).contour;
    std::reverse(hole.points.begin(), hole.points.end());
    ring.holes.push_back(hole);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {ring}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache);
    const bool target_inside = GENERATE(false, true);
    const std::vector<c_point> result = planner.path(0,
        target_inside ? pending(0., 0., 5., 0.) : pending(5., 0., 10., 0.));
    CHECK(result.size() == 2);
    CHECK(planner.fallbacks() == 1);
    CHECK(planner.detours() == 0);
}

TEST_CASE("Zone planner counts fallback for an endpoint inside only the nozzle margin", "[plugins][zone-planner]")
{
    Owner owner;
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {rectangle(4., -2., 2., 4.)}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache);
    const bool target_inside = GENERATE(false, true);
    const std::vector<c_point> result = planner.path(0,
        target_inside ? pending(0., 0., 6.1, 0.) : pending(3.9, 0., 10., 0.));
    CHECK(result.size() == 2);
    CHECK(planner.fallbacks() == 1);
    CHECK(planner.detours() == 0);
}

TEST_CASE("Zone planner accepts real global adhesion provenance without an object instance", "[plugins][zone-planner]")
{
    const bool brim = GENERATE(false, true);
    Owner owner;
    owner.config.set_deserialize_strict({{"skirts", "2"}, {"brim_width", brim ? "2" : "0"}});
    owner.print.apply(owner.model, owner.config);
    owner.print.process();
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
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache);
    REQUIRE(compatibility.endpoint_enables_avoidance(object));
    CHECK_FALSE(compatibility.endpoint_enables_avoidance(auxiliary));
    CHECK(planner.eligible(auxiliary, object));
    CHECK(planner.eligible(object, auxiliary));
    CHECK(planner.eligible(auxiliary, auxiliary));
    travel::TravelEndpoint invalid = auxiliary;
    invalid.printing_catalog_id = uint32_t(owner.plan.object_instance_count());
    CHECK(planner.eligible(invalid, object));
    zones::ZonePlanner disabled_defaults(compatibility, view,
        owner.settings(), false, &owner.context, owner.print_view, owner.cache);
    CHECK_FALSE(disabled_defaults.eligible(auxiliary, auxiliary));
    CHECK(disabled_defaults.eligible(auxiliary, object));
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
    zones::ZonePlanner planner(compatibility, view, owner.settings(), auxiliary_avoidance, &owner.context, owner.print_view, owner.cache);
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
    owner.print.process();
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
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache);
    const std::vector<c_point> result = planner.path(0, gap);
    REQUIRE(planner.detours() == size_t(use_tops));
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
    owner.print.process();
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
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache);
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
    owner.print.process();
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
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache);
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
        CHECK(planner.detours() == 1);
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
    zones::ZonePlanner excluding(compatibility, view, first_excluded, true, &owner.context, owner.print_view, owner.cache);
    CHECK_FALSE(excluding.eligible(auxiliary, first_target));
    CHECK(excluding.eligible(first_target, auxiliary));
}
// Exercise domain containment and endpoint attachment without introducing object provenance.
TEST_CASE("Zone planner retains interior points and attaches nearby free endpoints", "[plugins][zone-planner]")
{
    Owner owner;
    FixedZones contributor;
    work::ExPolygon domain = rectangle(0., 0., 10., 10.);
    travel::PendingTravel gap = pending(2., 5., 8., 5.);
    bool direct = true, inside = true, near = false, far = false;
    SECTION("Interior segment") {}
    SECTION("Interior near the edge") { gap = pending(0.01, 0.01, 9.99, 0.01); }
    SECTION("Z-only with a nonempty domain") { gap = pending(2., 5., 2., 5.); gap.target.position.z = Slic3r::scale_i(0.2); }
    SECTION("Nearby free endpoint") { gap = pending(-0.1, 5., 8., 8.); inside = false; direct = false; near = true; }
    SECTION("Both endpoints far use obstacle routing") {
        domain = rectangle(0., 100., 10., 10.);
        contributor.areas.push_back({zones::ZoneRole::Forbidden, {rectangle(4., 4., 2., 2.)}, 0, SIZE_MAX, {}});
        inside = false; direct = false; far = true;
    }
    SECTION("Forbidden hole is bypassed") {
        contributor.areas.push_back({zones::ZoneRole::Forbidden, {rectangle(4., 4., 2., 2.)}, 0, SIZE_MAX, {}});
        direct = false;
    }
    contributor.areas.push_back({zones::ZoneRole::Attractor, {domain}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache);
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

// Accept a safe domain route when an endpoint inside the expanded margin prevents an obstacle-only reference route.
TEST_CASE("Zone planner accepts a safe domain route without a usable reference", "[plugins][zone-planner]")
{
    Owner owner;
    const work::ExPolygon obstacle = rectangle(4., -2., 2., 4.);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {rectangle(-2., -5., 14., 10.)}, 0, SIZE_MAX, {}});
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache);
    const travel::PendingTravel gap = pending(3.9, 0., 10., 0.);
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

// Rank nearby perimeter attachments against the chord without a domain, and try the next hypothesis after budget rejection.
TEST_CASE("Zone planner ranks perimeter neighbours by weight and retries after the budget", "[plugins][zone-planner]")
{
    Owner owner;
    bool domain_expected = true;
    bool target_on_perimeter = false;
    std::string weight = "0";
    SECTION("Low weight uses nearest ring attachments") {}
    SECTION("Endpoint on perimeter needs no outside attachment") { target_on_perimeter = true; }
    SECTION("High weight uses the chord without a domain") {
        domain_expected = false;
        weight = "2";
    }
    SECTION("Budget rejection tries the chord without a domain") {
        domain_expected = false;
        owner.config.set_deserialize_strict("avoid_crossing_perimeters_max_detour", "0.1");
    }
    owner.config.set_deserialize_strict("avoid_travel_island", "1");
    owner.config.set_deserialize_strict("avoid_travel_island_weight", weight);
    owner.print.apply(owner.model, owner.config);
    owner.print.process();
    const slic3r_api::Layer layer(reinterpret_cast<const layer_handle *>(&owner.print.object(0).layers().back()));
    travel::PendingTravel gap = pending(-0.1, 2., target_on_perimeter ? 0. : -0.1, 8.);
    gap.source.region_island = layer.island(0).get_or_create_full_region_island(0);
    gap.target.region_island = gap.source.region_island;
    gap.source.printing_catalog_id = owner.print.mutable_printing_plan().find_or_add_object_instance(owner.print.object(0), 0);
    gap.target.printing_catalog_id = gap.source.printing_catalog_id;
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {rectangle(0., 0., 10., 10.)}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache);
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() >= 2);
    CHECK(route.front().x == gap.source.position.x);
    CHECK(route.front().y == gap.source.position.y);
    CHECK(route.back().x == gap.target.position.x);
    CHECK(route.back().y == gap.target.position.y);
    CHECK(planner.domain_routes() == size_t(domain_expected));
    CHECK(planner.fallbacks() == 0);
    work::Polyline candidate;
    for (const c_point &point : route) candidate.points.emplace_back(point);
    const work::Polylines outside = slic3r_api::PluginCommon::clipper::diff_pl(
        owner.context.plugin_storage, {candidate}, {rectangle(0., 0., 10., 10.)});
    double outside_length = 0.;
    for (const work::Polyline &part : outside) outside_length += part.length();
    // With no obstacles and a chord outside the rectangle, the reference's outside length is the chord length.
    CHECK(outside_length <= work::Point(route.front()).distance_to(work::Point(route.back())) + Slic3r::SCALED_EPSILON);
    if (domain_expected) {
        REQUIRE(route.size() >= size_t(target_on_perimeter ? 3 : 4));
        CHECK(route[1].x == 0);
        CHECK(route[1].y == gap.source.position.y);
        if (!target_on_perimeter) {
            CHECK(route[route.size() - 2].x == 0);
            CHECK(route[route.size() - 2].y == gap.target.position.y);
        }
    } else CHECK(route.size() == 2);
    const std::vector<c_point> repeated = planner.path(1, gap);
    REQUIRE(repeated.size() == route.size());
    for (size_t i = 0; i < route.size(); ++i) {
        CHECK(repeated[i].x == route[i].x);
        CHECK(repeated[i].y == route[i].y);
    }
}

// Keep the inexpensive domain hypothesis by contouring its blocked bridge even when the original chord is clear.
TEST_CASE("Zone planner contours a blocked bridge above a clear chord", "[plugins][zone-planner]")
{
    Owner owner;
    const work::ExPolygons domains{rectangle(-1., 0.5, 2., 2.), rectangle(9., 0.5, 2., 2.)};
    const work::ExPolygon obstacle = rectangle(4., 0.25, 2., 0.5);
    const travel::PendingTravel gap = pending(0., 0., 10., 0.);
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
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache);
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() > 2);
    CHECK(route.front().x == gap.source.position.x);
    CHECK(route.front().y == gap.source.position.y);
    CHECK(route.back().x == gap.target.position.x);
    CHECK(route.back().y == gap.target.position.y);
    CHECK(planner.fallbacks() == 0);
    CHECK(planner.domain_routes() == 1);
    CHECK(planner.detours() == 1);
    check_clear(route, {obstacle}, owner.storage);
}

// Reuse active zones while alternating endpoint membership, retaining the obstacle only in the matching cached domain.
TEST_CASE("Zone planner caches domains separately for ignored forbidden sets", "[plugins][zone-planner]")
{
    Owner owner;
    FixedZones contributor;
    const work::ExPolygon obstacle = rectangle(4., -2., 2., 4.);
    contributor.areas.push_back({zones::ZoneRole::Attractor, {rectangle(-2., -5., 14., 10.)}, 0, SIZE_MAX, {}});
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache);
    CHECK(planner.path(0, pending(5., 0., 10., 0.)).size() == 2);
    const std::vector<c_point> route = planner.path(1, pending(0., 0., 10., 0.));
    REQUIRE(route.size() > 2);
    check_clear(route, {obstacle}, owner.storage);
    CHECK(planner.path(2, pending(5., 0., 10., 0.)).size() == 2);
    const std::vector<c_point> larger = planner.path(3, pending(0., 0., 10., 0., 1));
    REQUIRE(larger.size() > 2);
    check_clear(larger, {obstacle}, owner.storage);
    coord_t small_extent = 0, large_extent = 0;
    for (const c_point &point : route) small_extent = std::max(small_extent, std::abs(point.y));
    for (const c_point &point : larger) large_extent = std::max(large_extent, std::abs(point.y));
    CHECK(large_extent > small_extent);
    CHECK(planner.domain_routes() == 4);
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
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {rectangle(0., 0., 10., 10.)}, 0, SIZE_MAX, {}});
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {rectangle(4., 4., 2., 2.)}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), enabled, &owner.context, owner.print_view, owner.cache);
    if (cancelled) {
        owner.checks = 0;
        owner.cancel_at = 3;
        CHECK_THROWS_AS(planner.path(0, pending(2., 5., 8., 5.)), slic3r_api::PluginCancelled);
        return;
    }
    const std::vector<c_point> route = planner.path(0, pending(2., 5., 8., 5.));
    CHECK((route.size() > 2) == (enabled && !reject));
    CHECK(planner.fallbacks() == size_t(reject));
    CHECK(planner.domain_routes() == size_t(enabled && !reject));
}

// Reject an attachment crossing a thin forbidden area while accepting the original chord, whose expanded nozzle margin is clear.
TEST_CASE("Zone planner rejects a blocked attachment and accepts the clear chord", "[plugins][zone-planner]")
{
    Owner owner;
    const work::ExPolygon obstacle = rectangle(-0.1, 0.25, 0.2, 0.01);
    const travel::PendingTravel gap = pending(0., 0., 10., 0.);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {rectangle(-1., 0.5, 2., 2.)}, 0, SIZE_MAX, {}});
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache);
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() == 2);
    CHECK(route.front().x == gap.source.position.x);
    CHECK(route.front().y == gap.source.position.y);
    CHECK(route.back().x == gap.target.position.x);
    CHECK(route.back().y == gap.target.position.y);
    CHECK(planner.fallbacks() == 0);
    CHECK(planner.domain_routes() == 0);
    CHECK(planner.detours() == 0);
    check_clear(route, {obstacle}, owner.storage);
}

// Retain contour-edge identities when subtraction puts both bridge endpoints exactly on the expanded forbidden border.
TEST_CASE("Zone planner contours bridges starting and ending on expanded forbidden material", "[plugins][zone-planner]")
{
    Owner owner;
    work::ExPolygons attractors{rectangle(0., 0., 20., 10.)};
    work::ExPolygon obstacle = rectangle(9., -1., 2., 12.);
    travel::PendingTravel gap = pending(2., 5., 18., 5.);
    bool vertex = false;
    SECTION("Bridge endpoints on contour edges") {}
    SECTION("Bridge starts at a contour vertex") {
        attractors = {rectangle(0., -5., 8.8, 4.8), rectangle(11.2, 0.5, 8.8, 4.5)};
        obstacle = rectangle(9., 0., 2., 2.);
        gap = pending(2., -1., 18., 1.);
        vertex = true;
    }
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, attractors, 0, SIZE_MAX, {}});
    contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache);
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(planner.bridges() == 1);
    CHECK(planner.domain_routes() == 1);
    CHECK(planner.fallbacks() == 0);
    check_clear(route, {obstacle}, owner.storage);
    const work::ExPolygons expanded = slic3r_api::PluginCommon::clipper::offset_ex(owner.context.plugin_storage, {obstacle}, double(Slic3r::scale_i(0.2)));
    const work::ExPolygons domains = slic3r_api::PluginCommon::clipper::diff_ex(owner.context.plugin_storage, attractors, expanded);
    if (vertex) {
        const work::Point corner(Slic3r::scale_i(8.8), Slic3r::scale_i(-0.2));
        REQUIRE(expanded.size() == 1);
        const work::Points &contour = expanded.front().contour.points;
        CHECK(std::find(contour.begin(), contour.end(), corner) != contour.end());
        CHECK(std::find_if(route.begin(), route.end(), [&corner](const c_point &point) {
            return point.x == corner.x() && point.y == corner.y();
        }) != route.end());
    }
    FixedZones reference_contributor;
    reference_contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    zones::TravelZoneView reference_view({std::cref(reference_contributor)}, owner.input());
    zones::ZonePlanner reference_planner(compatibility, reference_view, owner.settings(), true, &owner.context, owner.print_view, owner.cache);
    work::Polyline path, reference;
    for (const c_point &point : route) path.points.emplace_back(point);
    for (const c_point &point : reference_planner.path(0, gap)) reference.points.emplace_back(point);
    REQUIRE(reference_planner.fallbacks() == 0);
    double outside = 0., reference_outside = 0.;
    for (const work::Polyline &part : slic3r_api::PluginCommon::clipper::diff_pl(owner.context.plugin_storage, {path}, domains)) outside += part.length();
    for (const work::Polyline &part : slic3r_api::PluginCommon::clipper::diff_pl(owner.context.plugin_storage, {reference}, domains)) reference_outside += part.length();
    CHECK(outside <= reference_outside + Slic3r::SCALED_EPSILON);
}

// Contour a forbidden top between two components while preserving the same bridge points used to estimate the route.
TEST_CASE("Zone planner contours selected bridges between component ring routes", "[plugins][zone-planner]")
{
    Owner owner;
    const double weight = GENERATE(0., 0.2);
    CAPTURE(weight);
    owner.config.set_deserialize_strict("avoid_travel_island", "1");
    owner.config.set_deserialize_strict("avoid_travel_island_weight", std::to_string(weight));
    owner.print.apply(owner.model, owner.config);
    owner.print.process();
    const slic3r_api::Layer layer(reinterpret_cast<const layer_handle *>(&owner.print.object(0).layers().back()));
    travel::PendingTravel gap = pending(0.2, 1., 15.2, 9.);
    gap.source.region_island = gap.target.region_island = layer.island(0).get_or_create_full_region_island(0);
    gap.source.printing_catalog_id = gap.target.printing_catalog_id =
        owner.print.mutable_printing_plan().find_or_add_object_instance(owner.print.object(0), 0);
    const work::ExPolygons domains{rectangle(0., 0., 5., 10.), rectangle(15., 0., 5., 10.)};
    const work::ExPolygon obstacle = rectangle(9., 3., 2., 4.);
    ac::Boundary boundary;
    ac::build_boundary(owner.context.plugin_storage, work::ExPolygons(domains), Slic3r::scale_i(0.8), boundary);
    work::Point start(gap.source.position.x, gap.source.position.y), end(gap.target.position.x, gap.target.position.y);
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
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache);
    const std::vector<c_point> route = planner.path(0, gap);
    CHECK(planner.fallbacks() == 0);
    CHECK(planner.domain_routes() == 1);
    REQUIRE(planner.bridges() == 1);
    check_clear(route, {obstacle}, owner.storage);
    work::Polyline path;
    for (const c_point &point : route) path.points.emplace_back(point);
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
    FixedZones only_obstacle;
    only_obstacle.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    zones::TravelZoneView reference_view({std::cref(only_obstacle)}, owner.input());
    zones::ZonePlanner reference_planner(compatibility, reference_view, owner.settings(), true, &owner.context, owner.print_view, owner.cache);
    work::Polyline reference;
    for (const c_point &point : reference_planner.path(0, gap)) reference.points.emplace_back(point);
    REQUIRE(reference_planner.fallbacks() == 0);
    double outside = 0., reference_outside = 0.;
    for (const work::Polyline &part : slic3r_api::PluginCommon::clipper::diff_pl(owner.context.plugin_storage, {path}, domains)) outside += part.length();
    for (const work::Polyline &part : slic3r_api::PluginCommon::clipper::diff_pl(owner.context.plugin_storage, {reference}, domains)) reference_outside += part.length();
    CHECK(outside <= reference_outside + Slic3r::SCALED_EPSILON);
}

// Retain the nearest ring exit/entry when the other endpoint has no nearby component, in both directions.
TEST_CASE("Zone planner connects a far free endpoint through the nearest component ring point", "[plugins][zone-planner]")
{
    Owner owner;
    const bool reverse = GENERATE(false, true);
    const travel::PendingTravel gap = reverse ? pending(20., 9., 2., 9.) : pending(2., 9., 20., 9.);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {rectangle(0., 0., 5., 10.)}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache);
    const std::vector<c_point> route = planner.path(0, gap);
    REQUIRE(route.size() >= 3);
    CHECK(route.front().x == gap.source.position.x);
    CHECK(route.back().x == gap.target.position.x);
    CHECK(std::find_if(route.begin(), route.end(), [](const c_point &point) {
        return point.x == Slic3r::scale_i(5.) && point.y == Slic3r::scale_i(9.);
    }) != route.end());
    for (const c_point &point : route) CHECK(point.y == Slic3r::scale_i(9.));
    CHECK(planner.domain_routes() == 1);
    CHECK(planner.bridges() == 0);
    CHECK(planner.fallbacks() == 0);
}

// Skip a bridge whose box-based hole choice cannot seed its ring, then leave the source component at the first nearest ring point.
TEST_CASE("Zone planner skips an unavailable bridge and tries a mixed hypothesis", "[plugins][zone-planner]")
{
    Owner owner;
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
    const travel::PendingTravel gap = pending(15., 12., 12., 12.);
    ac::Boundary boundary;
    ac::build_boundary(owner.context.plugin_storage, work::ExPolygons(domains), Slic3r::scale_i(0.8), boundary);
    size_t source_component = SIZE_MAX, target_component = SIZE_MAX, ring = 0;
    for (const work::ExPolygon &component : domains) {
        if (component.contains({gap.source.position.x, gap.source.position.y})) source_component = ring;
        if (component.contains({gap.target.position.x, gap.target.position.y})) target_component = ring;
        ring += 1 + component.holes.size();
    }
    REQUIRE(source_component != SIZE_MAX);
    REQUIRE(target_component != SIZE_MAX);
    work::Point start(gap.source.position.x, gap.source.position.y), end(gap.target.position.x, gap.target.position.y);
    ac::RouterCounts counts;
    CHECK_FALSE(ac::select_island_bridge(boundary, source_component, target_component, start, end, 0., counts, []() {}).has_value());
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, domains, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache);
    const std::vector<c_point> route = planner.path(0, gap);
    CHECK(planner.bridges() == 0);
    CHECK(planner.domain_routes() == 1);
    CHECK(planner.fallbacks() == 0);
    std::string coordinates;
    for (const c_point &point : route) coordinates += std::to_string(point.x) + "," + std::to_string(point.y) + ";";
    INFO(coordinates);
    CHECK(std::find_if(route.begin(), route.end(), [](const c_point &point) {
        return point.x == Slic3r::scale_i(10.) && point.y == Slic3r::scale_i(12.);
    }) != route.end());
}

// Construct explicit advanced bridges with and without forbidden material between the domains.
TEST_CASE("Zone planner constructs bridges and validates forbidden material", "[plugins][zone-planner]")
{
    Owner owner;
    const bool forbidden = GENERATE(false, true);
    owner.config.set_deserialize_strict("avoid_travel_island", "1");
    // Zero interior weight keeps the two-domain bridge competitive with the chord without a domain.
    owner.config.set_deserialize_strict("avoid_travel_island_weight", "0");
    owner.print.apply(owner.model, owner.config);
    owner.print.process();
    const slic3r_api::Layer layer(reinterpret_cast<const layer_handle *>(&owner.print.object(0).layers().back()));
    travel::PendingTravel gap = pending(2., 2., 18., 8.);
    gap.target.region_island = layer.island(0).get_or_create_full_region_island(0);
    gap.target.printing_catalog_id = owner.print.mutable_printing_plan().find_or_add_object_instance(owner.print.object(0), 0);
    FixedZones contributor;
    contributor.areas.push_back({zones::ZoneRole::Attractor, {rectangle(0., 0., 5., 10.), rectangle(15., 0., 5., 10.)}, 0, SIZE_MAX, {}});
    const work::ExPolygon obstacle = rectangle(9., -1., 2., 12.);
    if (forbidden) contributor.areas.push_back({zones::ZoneRole::Forbidden, {obstacle}, 0, SIZE_MAX, {}});
    zones::TravelZoneView view({std::cref(contributor)}, owner.input());
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache);
    const std::vector<c_point> route = planner.path(0, gap);
    CHECK(planner.bridges() == 1);
    CHECK(planner.fallbacks() == 0);
    CHECK(planner.domain_routes() == 1);
    if (forbidden) check_clear(route, {obstacle}, owner.storage);
}

// Slice real support, then test component isolation, instance shifts, Z selection and non-support travels remaining unaffected.
TEST_CASE("Support obstacles retain other components and catalog instances", "[plugins][zone-planner][travel-zones]")
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
    owner.print.process();
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
    size_t real_support_gaps = 0, real_support_detours = 0;
    const zones::SupportObstacleContributor real_contributor;
    ac::CompatibilityPlanner real_compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    for (uint32_t g = 0; g < plan.groups.size(); ++g) {
        for (uint32_t l = 0; l < plan.groups[g].layers.size(); ++l) {
            slic3r_api::PrintingLayerGroup layer_view(reinterpret_cast<printing_layer_group_handle *>(
                &plan.groups[g].layers[l]));
            const travel::LayerTravelChronology chronology = connector.prepare(layer_view, g, l);
            zones::ZoneContributorInput real_input = owner.input();
            real_input.print_z = plan.groups[g].layers[l].print_z;
            zones::TravelZoneView real_view({std::cref(real_contributor)}, real_input);
            zones::ZonePlanner real_planner(real_compatibility, real_view, owner.settings(), true, &owner.context, owner.print_view, owner.cache);
            for (size_t t = 0; t < chronology.travels.size(); ++t) {
                const travel::PendingTravel &gap = chronology.travels[t];
                if (!gap.source_extrusion.valid() ||
                    !(RAW_EXTRUSION_ROLE_IS_SUPPORT(gap.source_extrusion.role()) ||
                      RAW_EXTRUSION_ROLE_IS_SUPPORT(gap.target_extrusion.role())))
                    continue;
                REQUIRE(real_planner.eligible(gap.source, gap.target));
                REQUIRE_FALSE(real_view.query(t, gap).forbidden_indexes.empty());
                const size_t before = real_planner.detours();
                const std::vector<c_point> route = real_planner.path(t, gap);
                CHECK(route.size() >= 2);
                if (real_planner.detours() != before) {
                    work::ExPolygons remaining;
                    for (size_t index : real_view.query(t, gap).forbidden_indexes)
                        for (const work::ExPolygon &area : real_view.zone_area(index))
                            if (!area.contains({gap.source.position.x, gap.source.position.y}) &&
                                !area.contains({gap.target.position.x, gap.target.position.y}))
                                remaining.push_back(area);
                    check_clear(route, remaining, owner.storage);
                    ++real_support_detours;
                }
                ++real_support_gaps;
            }
        }
    }
    REQUIRE(real_support_gaps > 0);
    REQUIRE(real_support_detours > 0);
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
    zones::SupportObstacleContributor contributor;
    zones::ZoneContributorInput input = owner.input();
    input.print_z = layer.scaled_print_z();
    std::vector<zones::TravelZone> exact, between, absent;
    contributor.contribute(input, exact);
    REQUIRE(exact.size() == 4);
    input.print_z += Slic3r::scale_i(0.05);
    contributor.contribute(input, between);
    REQUIRE(between.size() == exact.size());
    input.print_z = 0;
    contributor.contribute(input, absent);
    CHECK(absent.empty());
    // The roof replaces two pillar components at the first layer above the pillars.
    std::vector<zones::TravelZone> roof_exact, below_roof;
    input.print_z = owner.print.object(0).layers()[10].scaled_print_z();
    contributor.contribute(input, roof_exact);
    input.print_z -= Slic3r::scale_i(0.1);
    contributor.contribute(input, below_roof);
    REQUIRE(roof_exact.size() == 2);
    REQUIRE(below_roof.size() == 4);
    for (size_t z = 0; z < exact.size(); ++z) {
        CHECK(exact[z].begin == 0);
        CHECK(exact[z].end == SIZE_MAX);
        CHECK(exact[z].scope.extruder == EXTRUDER_ID_UNKNOWN);
        CHECK(exact[z].scope.printing_catalog_id == SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE);
        CHECK(exact[z].scope.support_travels_only);
        REQUIRE(exact[z].area.front().contour.points == between[z].area.front().contour.points);
        const Slic3r::Polygon &expected = layer.lslices()[z % 2].contour;
        const c_point instance_shift = owner.print_view.object(0).instance_shift(z / 2);
        REQUIRE(exact[z].area.front().contour.points.size() == expected.size());
        for (size_t p = 0; p < expected.size(); ++p) {
            CHECK(exact[z].area.front().contour.points[p].x() == expected.points[p].x() + instance_shift.x);
            CHECK(exact[z].area.front().contour.points[p].y() == expected.points[p].y() + instance_shift.y);
        }
    }
    input.print_z = layer.scaled_print_z();
    const Slic3r::Point minimum = Slic3r::get_extents(layer.lslices()).min;
    const c_point shift = owner.print_view.object(0).instance_shift(0);
    const bool target_object = GENERATE(false, true);
    travel::PendingTravel gap = pending(-5., 4., target_object ? 49. : 56., 4.);
    gap.source.position.x += minimum.x() + shift.x;
    gap.source.position.y += minimum.y() + shift.y;
    gap.target.position.x += minimum.x() + shift.x;
    gap.target.position.y += minimum.y() + shift.y;
    gap.source_extrusion = gap.target_extrusion = support;
    if (target_object) {
        const slic3r_api::Layer view(reinterpret_cast<const layer_handle *>(&layer));
        gap.target.printing_catalog_id = owner.print.mutable_printing_plan().find_or_add_object_instance(owner.print.object(0), 1);
        for (size_t i = 0; i < layer.lslices().size(); ++i)
            if (layer.lslices()[i].contains(minimum + Slic3r::Point(Slic3r::scale_i(14.), Slic3r::scale_i(4.))))
                gap.target.region_island = view.island(i).get_or_create_full_region_island(0);
        REQUIRE(gap.target.region_island.valid());
        gap.target_extrusion = slic3r_api::PrintingExtrusion(nullptr);
    }
    zones::TravelZoneView view({std::cref(contributor)}, input);
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context, 0, Slic3r::scale_i(0.6));
    zones::ZonePlanner planner(compatibility, view, owner.settings(), true, &owner.context, owner.print_view, owner.cache);
    REQUIRE(planner.eligible(gap.source, gap.target));
    const std::vector<c_point> result = planner.path(0, gap);
    REQUIRE(result.size() > 2);
    CHECK(planner.fallbacks() == 0);
    work::ExPolygons obstacles;
    for (const zones::TravelZone &zone : exact)
        if (!zone.area.front().contains({gap.target.position.x, gap.target.position.y}))
            obstacles.push_back(zone.area.front());
    REQUIRE(obstacles.size() == (target_object ? 3 : 4));
    check_clear(result, obstacles, owner.storage);
    gap.source_extrusion = gap.target_extrusion = slic3r_api::PrintingExtrusion(nullptr);
    CHECK(view.query(1, gap).forbidden_indexes.empty());
    CHECK(planner.path(1, gap).size() == 2);
    // Removing a catalog entry must not create zones for its still-existing object instance.
    REQUIRE(plan.object_instances.size() == 2);
    plan.object_instances.pop_back();
    std::vector<zones::TravelZone> catalog_only;
    contributor.contribute(input, catalog_only);
    CHECK(catalog_only.size() == 2);
    owner.checks = 0;
    owner.cancel_at = 2;
    CHECK_THROWS_AS(contributor.contribute(input, catalog_only), slic3r_api::PluginCancelled);
}
} // namespace
