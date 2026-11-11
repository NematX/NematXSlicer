#include <catch2/catch.hpp>

#include "layer_extrusion_edit_transition_test_helpers.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/AdvancedTravel/TravelZones.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/AvoidCrossing/LayerInputCache.hpp"

/*
Travel-zone tests give contributors small owned rectangles and query their
dated, scoped unions without a production consumer. A direct connector check
uses the shared transition helpers to publish real scopes and entry state, then
checks pair owners before apply. All views borrow locally owned prints or PEs.

    rectangle: build a counter-clockwise work area
    ZoneTestInput: keep borrowed contributor input and polygon storage alive
    FixedContributor::contribute: append synthetic areas and count calls
    tests: collect eagerly, query lazy unions, and summarize real ordered pairs
*/

namespace {
namespace zones = slic3r_api::LayerExtrusionEdit::AdvancedTravel;
namespace travel = slic3r_api::LayerExtrusionEdit::TravelConnection;
namespace helpers = Slic3r::Test::TransitionPipeline;
namespace work = slic3r_api::PluginCommon::polygon_work;

// Convert millimeter dimensions into one owned counter-clockwise rectangle; no API storage is borrowed.
work::ExPolygons rectangle(coord_t x, coord_t width);

struct ZoneTestInput
{
    Slic3r::Print print;
    Slic3r::PluginStorage storage;
    plugin_run_context context{};
    slic3r_api::Print print_view{reinterpret_cast<const print_handle *>(&print)};
    slic3r_api::PrintingPlan plan_view;
    slic3r_api::LayerExtrusionEdit::AvoidCrossing::LayerInputCache cache;
    travel::LayerTravelChronology chronology;
    zones::ChronologySummary summary;
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
                chronology.travels, summary};
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
    CHECK(attracted.forbidden.empty());
    CHECK(attracted.id != empty.id);
    CHECK(&view.query(1, pending) == &attracted);
    CHECK(view.union_builds() == 1);
    const zones::ActiveZones &overlap = view.query(2, pending);
    CHECK(overlap.id != attracted.id);
    CHECK(overlap.attractors.front().area() == Approx(15000. * Slic3r::scale_i(1.) * Slic3r::scale_i(1.)));
    CHECK(overlap.forbidden.front().area() == Approx(2500. * Slic3r::scale_i(1.) * Slic3r::scale_i(1.)));
    const zones::ActiveZones &ended = view.query(3, pending);
    CHECK(ended.attractors.empty());
    CHECK_FALSE(ended.forbidden.empty());
    CHECK(&view.query(4, pending) == &empty);
    CHECK(view.union_builds() == 4);
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
    SECTION("Between role unions without publishing a partial cache") {
        zones::TravelZoneView view({std::cref(contributor)}, owner.input());
        owner.cancel_at = owner.checks + 5;
        travel::PendingTravel pending;
        CHECK_THROWS_AS(view.query(0, pending), slic3r_api::PluginCancelled);
        CHECK(view.union_builds() == 1);
        owner.cancel_at = 0;
        const zones::ActiveZones &active = view.query(0, pending);
        CHECK_FALSE(active.attractors.empty());
        CHECK_FALSE(active.forbidden.empty());
        CHECK(view.union_builds() == 3);
        owner.cancel_at = owner.checks + 1;
        CHECK_THROWS_AS(view.query(0, pending), slic3r_api::PluginCancelled);
    }
}

TEST_CASE("Travel summary includes continuous and materialized printing owners and tools",
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
    const zones::ChronologySummary summary = zones::summarize(chronology,
        [](const travel::PendingTravel &pending) { return pending.extruder_id == 1; });
    CHECK(summary.travel_count == 3);
    REQUIRE(summary.extrusions.size() == 5);
    const std::vector<zones::TravelInterval> expected{{0, 0}, {0, 1}, {1, 1}, {1, 1}, {1, 3}};
    for (size_t index = 0; index < expected.size(); ++index) {
        CHECK(summary.extrusions[index].extrusion.handle() == expected_owners[index]);
        CHECK(summary.extrusions[index].interval.begin == expected[index].begin);
        CHECK(summary.extrusions[index].interval.end == expected[index].end);
    }
    REQUIRE(summary.tools.size() == 2);
    CHECK(summary.tools[0].extruder == 0);
    CHECK(summary.tools[0].interval.begin == 0);
    CHECK(summary.tools[0].interval.end == 1);
    CHECK(summary.tools[1].extruder == 1);
    CHECK(summary.tools[1].interval.begin == 1);
    CHECK(summary.tools[1].interval.end == 3);
    CHECK(summary.avoidance_active == std::vector<bool>{false, true, true});
    CHECK(summary.any_avoidance_active);
    CHECK_FALSE(zones::summarize(chronology, [](const travel::PendingTravel &) { return false; }).any_avoidance_active);
    const travel::LayerTravelChronology empty;
    CHECK(zones::summarize(empty, [](const travel::PendingTravel &) { return true; }).extrusions.empty());
}

} // namespace
