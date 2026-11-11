#include <catch2/catch.hpp>

#include "layer_extrusion_edit_transition_test_helpers.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/AdvancedTravel/ZonePlanner.hpp"
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
instances for compatibility candidates. Segment checks inspect filled material,
not just route vertices. The fixture selects travel.advanced to keep its settings
active while slicing, but invokes ZonePlanner directly without production wiring.

    Owner: own print, geometry storage, cancellation and contributor inputs
    FixedZones::contribute: copy synthetic dated areas
    rectangle / pending: construct millimeter geometry in platter coordinates
    check_clear: compare each route segment with original filled areas
    tests: exercise eligibility, cached margins, routing, limits and delegation
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
    // Borrow the complete applied configuration, including regional defaults omitted by Print::config().
    slic3r_api::Config settings() const
    { return slic3r_api::Config(Slic3r::ApiHost::to_config_handle(&print.full_print_config())); }
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
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context);
    zones::ZonePlanner planner(compatibility, view, owner.settings(), &owner.context);
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
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context);
    zones::ZonePlanner planner(compatibility, view, owner.settings(), &owner.context);
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
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context);
    zones::ZonePlanner planner(compatibility, view, owner.settings(), &owner.context);
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
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context);
    zones::ZonePlanner planner(compatibility, view, owner.settings(), &owner.context);
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
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context);
    zones::ZonePlanner planner(compatibility, view, owner.settings(), &owner.context);
    REQUIRE(compatibility.endpoint_enables_avoidance(object));
    CHECK_FALSE(compatibility.endpoint_enables_avoidance(auxiliary));
    CHECK(planner.eligible(auxiliary, object));
    CHECK(planner.eligible(object, auxiliary));
    CHECK(planner.eligible(auxiliary, auxiliary));
    travel::TravelEndpoint invalid = auxiliary;
    invalid.printing_catalog_id = uint32_t(owner.plan.object_instance_count());
    CHECK(planner.eligible(invalid, object));
    Slic3r::DynamicPrintConfig disabled = owner.config;
    disabled.set_deserialize_strict({{"avoid_crossing_perimeters", "0"}});
    zones::ZonePlanner disabled_defaults(compatibility, view,
        slic3r_api::Config(Slic3r::ApiHost::to_config_handle(&disabled)), &owner.context);
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
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context);
    zones::ZonePlanner planner(compatibility, view, owner.settings(), &owner.context);
    const travel::PendingTravel gap = pending(0., 0., 10., 0.);
    CHECK(planner.eligible(gap.source, gap.target) == enabled);
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

TEST_CASE("Zone planner delegates shifted attractors but forbidden material wins", "[plugins][zone-planner]")
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
    ac::CompatibilityPlanner compatibility(owner.print_view, owner.plan, &owner.context);
    zones::ZonePlanner planner(compatibility, view, owner.settings(), &owner.context);
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
    REQUIRE(result.size() >= 2);
    CHECK(result.front().x == gap.source.position.x);
    CHECK(result.back().x == gap.target.position.x);
    if (!forbidden) {
        REQUIRE(result.size() == expected.size());
        for (size_t i = 0; i < result.size(); ++i) { CHECK(result[i].x == expected[i].x); CHECK(result[i].y == expected[i].y); }
        CHECK(planner.same_attractor_delegations() == 1);
    } else {
        CHECK(planner.same_attractor_delegations() == 0);
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
    zones::ZonePlanner excluding(compatibility, view, first_excluded, &owner.context);
    CHECK_FALSE(excluding.eligible(auxiliary, first_target));
    CHECK(excluding.eligible(first_target, auxiliary));
}
} // namespace
