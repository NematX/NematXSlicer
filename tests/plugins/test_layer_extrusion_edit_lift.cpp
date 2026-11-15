///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
CreateRetractionLift tests
==========================

These cases run the real compact transition pipeline. They inspect Z offsets
on the reserved travel and wipe phases, which verifies both the lift profile
and the rule that a layer worker never has to rebuild printable scope content.
*/

#include "layer_extrusion_edit_transition_test_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Config/PrintConfig.hpp"

namespace {

using namespace Slic3r;
using namespace Slic3r::Printing;
using namespace Slic3r::Test::TransitionPipeline;
using slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope;

constexpr const char *WIPE_PLUGIN = "layer_extrusion_edit.wipe.default";
constexpr const char *ENTRY_STATE_PLUGIN =
    "layer_extrusion_edit.entry_state.default";
constexpr const char *TRAVEL_PLUGIN = "layer_extrusion_edit.travel.default";
constexpr const char *LIFT_PLUGIN = "layer_extrusion_edit.lift.default";

/* One geometric leaf with the process role effective at that leaf. */
struct LiftLeaf
{
    slic3r_api::ExtrusionEntity entity;
    raw_extrusion_role role = RAW_EXTRUSION_ROLE_NONE;
};

/* Build canonical print paths with lift settings, wipe retraction share and outgoing return/inside-end options applied before Print::apply. */
void prepare_lift_print(Print &print,
                        Model &model,
                        const char *lift,
                        const char *ramping,
                        const char *slope,
                        const char *minimum_travel = "0",
                         bool wipe_lift = false,
                         const char *wipe_length = "100%",
                         const char *before_wipe = "20%",
                         bool returning = false,
                          bool inside_end = false,
                          const char *minimum_z = "0",
                          const char *ceiling = "200");

/* Collect geometric leaves while resolving inherited process attributes. */
void collect_lift_leaves(
    const slic3r_api::ExtrusionEntity &entity,
    std::optional<slic3r_api::EPropertyAttributes> inherited_attributes,
    std::vector<LiftLeaf> &leaves);

void prepare_lift_print(
    Print &print,
    Model &model,
    const char *lift,
    const char *ramping,
    const char *slope,
    const char *minimum_travel,
    const bool wipe_lift,
    const char *wipe_length,
    const char *before_wipe,
    const bool returning,
    const bool inside_end,
    const char *minimum_z,
    const char *ceiling)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"lift_min", minimum_z}, {"max_print_height", ceiling},
        {"retract_before_travel", "0"},
        {"retract_layer_change", "0"},
        {"retract_length", "2"},
        {"retract_length_toolchange", "2"},
        {"retract_restart_extra", "0.1"},
        {"retract_restart_extra_toolchange", "0.2"},
        {"retract_lift", lift},
        {"retract_lift_above", "0"},
        {"retract_lift_below", "0"},
        {"retract_lift_first_layer", "0"},
        {"retract_lift_top", "All surfaces"},
        {"retract_lift_before_travel", minimum_travel},
        {"travel_ramping_lift", ramping},
        {"travel_slope", slope},
        {"wipe", wipe_lift ? "1" : "0"},
        {"wipe_speed", "20"},
        {"wipe_min", "8"},
        {"wipe_return", returning ? "1" : "0"},
        {"wipe_only_crossing", "0"},
        {"retract_before_wipe", before_wipe},
        {"retract_speed", "40"},
        {"travel_speed", "120"},
        {"wipe_inside_start", "0"},
        {"wipe_inside_end", inside_end ? "1" : "0"},
        {"wipe_extra_perimeter", "0"},
        {"wipe_lift", wipe_lift ? "1" : "0"},
        {"wipe_lift_length", wipe_lift ? wipe_length : "0"}
    });
    Slic3r::Test::init_print(
        {Slic3r::Test::TestMesh::cube_20x20x20}, print, model, config);

    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.clear();
    plan.groups.emplace_back();
    plan.groups.back().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.back().layers.back();
    layer.print_z = scale_i(0.2);
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool = layer.tool_groups.back();
    tool.extruder_id = 0;
    append_printing_extrusion(tool, make_print_path(0.0, 10.0));
    append_printing_extrusion(tool, make_print_path(20.0, 30.0));
    if (inside_end) {
        std::unique_ptr<ExtrusionPath> path = make_print_path(0.0, 10.0);
        path->polyline().append(Point(scale_i(10.), scale_i(10.)));
        path->polyline().append(Point(0, scale_i(10.)));
        path->polyline().append(Point(0, 0));
        ExtrusionEntity::Children children;
        children.push_back(std::move(path));
        std::unique_ptr<ExtrusionEntity> loop = std::make_unique<ExtrusionEntity>(
            std::move(children), false, false, false);
        loop->get_or_add_property<ExtrusionPropertyLoopRole>().set_perimeter_role(elrDefault);
        tool.extrusions.front().root = CanonicalExtrusion::acquire(*loop);
    }
}

void collect_lift_leaves(
    const slic3r_api::ExtrusionEntity &entity,
    std::optional<slic3r_api::EPropertyAttributes> inherited_attributes,
    std::vector<LiftLeaf> &leaves)
{
    if (const slic3r_api::EPropertyAttributes *direct =
            entity.get(slic3r_api::EPropertyAttributes::key))
        inherited_attributes = *direct;
    if (entity.segment_count() != 0 && inherited_attributes)
        leaves.push_back(LiftLeaf{entity, inherited_attributes->extrusion_role()});
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        collect_lift_leaves(entity.child(child_idx), inherited_attributes, leaves);
}

TEST_CASE("CreateRetractionLift registers after travel routing",
          "[plugins][layer-extrusion-edit][lift]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Plugin *plugin = Orchestrator::instance().get_plugin(LIFT_PLUGIN);
    REQUIRE(plugin != nullptr);
    CHECK(plugin->get_step() == STEP_LAYER_EXTRUSION_EDIT);
    CHECK(plugin->get_priority() == -40);
    CHECK(plugin->get_exclusive_group() == "layer_extrusion_edit.lift");
    REQUIRE(plugin->get_dependencies().size() == 1);
    CHECK(plugin->get_dependencies().front() == TRANSITION_SCOPE_PLUGIN);

    const std::vector<Plugin::UsedConfigKey> keys = plugin->get_used_config_keys();
    CHECK(std::any_of(keys.begin(), keys.end(), [](const Plugin::UsedConfigKey &key) {
        return key.key == "retract_lift" && key.type == RAW_CO_VECTOR_FLOAT;
    }));
    CHECK(std::any_of(keys.begin(), keys.end(), [](const Plugin::UsedConfigKey &key) {
        return key.key == "wipe_lift" &&
            key.type == RAW_CO_VECTOR_FLOAT_OR_PERCENT;
    }));

    // The plugin publishes print_retract_lift and reads it as a region override.
    std::vector<std::string> defined_keys;
    for (const Plugin::DefinedConfigKey &key : plugin->get_defined_config_keys())
        defined_keys.push_back(key.key);
    CHECK(defined_keys.size() == 1);
    CHECK(defined_keys.front() == "print_retract_lift");

    const ConfigOptionDef *def = PrintConfigDef::instance().get("print_retract_lift");
    REQUIRE(def != nullptr);
    CHECK(def->type == coFloat);
    CHECK(def->container_type == ConfigOptionContainerType::Region);
    CHECK(def->invalidates_step == STEP_LAYER_EXTRUSION_EDIT);
    CHECK(def->can_be_disabled);
    CHECK(def->default_value->serialize() == "!0");
    CHECK(PrintConfig::defaults().option("print_retract_lift") == nullptr);
    CHECK(PrintRegionConfig::defaults().option("print_retract_lift") != nullptr);
}

TEST_CASE("Stationary deposits keep separate scopes through the transition chain",
          "[plugins][deposit][lift][retraction][transition-scope][travel][layer-entry]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool ordinary_point = GENERATE(false, true);
    Print print;
    Model model;
    prepare_lift_print(print, model, "1", "1", "0", "0", true);
    PrintingToolGroup &tool = print.mutable_printing_plan().groups.front()
        .layers.front().tool_groups.front();
    std::unique_ptr<ExtrusionPath> deposit = make_print_path(15.0, 16.0);
    deposit->polyline() = ArcPolyline();
    deposit->polyline().append(Point(scale_i(15.0), 0));
    deposit->get_or_add_property<ExtrusionAttributes>().set_role(ExtrusionRole::SupportMaterial);
    deposit->add_property(ExtrusionPropertySpecialCommand(
        ExtrusionPropertySpecialCommand::Code::DEPOSIT, 0.5));
    PrintingExtrusion middle;
    middle.root = CanonicalExtrusion::acquire(*deposit);
    middle.sregion_island_role = ExtrusionRole::SupportMaterial;
    tool.extrusions.insert(tool.extrusions.begin() + 1, std::move(middle));
    ExtrusionEntity *ignored = nullptr;
    if (ordinary_point) {
        std::unique_ptr<ExtrusionPath> point = make_print_path(12.0, 13.0);
        point->polyline() = ArcPolyline();
        point->polyline().append(Point(scale_i(100.0), 0));
        PrintingExtrusion event;
        event.root = CanonicalExtrusion::acquire(*point);
        ignored = event.root.get();
        tool.extrusions.insert(tool.extrusions.begin() + 1, std::move(event));
    }

    run_layer_plugins(print, {
        TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, WIPE_PLUGIN,
        ENTRY_STATE_PLUGIN, TRAVEL_PLUGIN, LIFT_PLUGIN
    });

    REQUIRE(tool.extrusions.size() == (ordinary_point ? 4 : 3));
    if (ignored != nullptr) {
        CHECK_FALSE(find_scope(entity_view(*ignored)).valid());
        CHECK(ignored->child_count() == 0);
        CHECK(entity_view(*ignored).point_count() == 1);
    }
    const OrderedExtrusionScope scope(find_scope(entity_view(*tool.extrusions[ordinary_point ? 2 : 1].root)),
                                     scope_property_key());
    REQUIRE(scope.content().is_deposit());
    CHECK(scope.content().point_count() == 1);
    CHECK(scope.content().get(slic3r_api::EPropertySpecialCommand::key)->extra_data == 0.5);
    std::vector<std::string> before;
    std::vector<std::string> after;
    collect_transition_events(scope.before().readonly(), before);
    collect_transition_events(scope.after().readonly(), after);
    CHECK(before == std::vector<std::string>{"unretract"});
    CHECK(after == std::vector<std::string>{"retract"});
    std::vector<LiftLeaf> incoming;
    collect_lift_leaves(scope.travel().readonly(), std::nullopt, incoming);
    REQUIRE_FALSE(incoming.empty());
    const slic3r_api::ExtrusionEntity last = incoming.back().entity;
    CHECK(last.point(last.point_count() - 1).x == scale_i(15.0));
    CHECK(last.z_offset(last.point_count() - 1) == 0);
    const OrderedExtrusionScope target(find_scope(entity_view(*tool.extrusions[ordinary_point ? 3 : 2].root)),
                                      scope_property_key());
    std::vector<LiftLeaf> outgoing;
    collect_lift_leaves(target.travel().readonly(), std::nullopt, outgoing);
    REQUIRE(outgoing.size() == 2);
    CHECK(outgoing.front().entity.point(0).x == scale_i(15.0));
    CHECK(outgoing.front().entity.z_offset(outgoing.front().entity.point_count() - 1) == scale_i(1.0));
    CHECK(outgoing.back().entity.point(outgoing.back().entity.point_count() - 1).x == scale_i(20.0));
    CHECK(outgoing.back().entity.z_offset(outgoing.back().entity.point_count() - 1) == 0);
    // Neither the stationary source nor its pre-content phase acquires a wipe path.
    std::vector<LiftLeaf> deposit_after;
    collect_lift_leaves(scope.after().readonly(), std::nullopt, deposit_after);
    CHECK(deposit_after.empty());
    std::vector<LiftLeaf> deposit_before;
    collect_lift_leaves(scope.before().readonly(), std::nullopt, deposit_before);
    CHECK(deposit_before.empty());
    for (const LiftLeaf &leaf : deposit_before)
        CHECK_FALSE(RAW_EXTRUSION_ROLE_IS_WIPE(leaf.role));
    for (const LiftLeaf &leaf : deposit_after)
        CHECK_FALSE(RAW_EXTRUSION_ROLE_IS_WIPE(leaf.role));
}

TEST_CASE("Travel snapping moves a stationary deposit without creating a segment",
          "[plugins][deposit][travel][transition-scope]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_lift_print(print, model, "0", "0", "0");
    PrintingToolGroup &tool = print.mutable_printing_plan().groups.front()
        .layers.front().tool_groups.front();
    ExtrusionEntity &deposit = *tool.extrusions[1].root;
    REQUIRE(deposit.has_polyline());
    deposit.polyline_ref() = ArcPolyline();
    deposit.polyline_ref().append(Point(scale_i(10.0) + SCALED_EPSILON / 3, 0));
    deposit.polyline_ref().set_z_offset(0, SCALED_EPSILON / 3);
    deposit.add_property(ExtrusionPropertySpecialCommand(
        ExtrusionPropertySpecialCommand::Code::DEPOSIT, 0.5));
    deposit.get_or_add_property<ExtrusionPropertyModifier>().enforce_retraction = true;
    run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, ENTRY_STATE_PLUGIN, TRAVEL_PLUGIN});
    const OrderedExtrusionScope scope(find_scope(entity_view(*tool.extrusions[1].root)),
                                     scope_property_key());
    REQUIRE(scope.content().is_deposit());
    CHECK(scope.content().point_count() == 1);
    CHECK(scope.content().segment_count() == 0);
    CHECK(scope.content().point(0).x == scale_i(10.0));
    CHECK(scope.content().point(0).y == 0);
    CHECK(scope.content().z_offset(0) == 0);
    CHECK(scope.travel().point_count() == 0);
    CHECK(scope.content().get(slic3r_api::EPropertySpecialCommand::key)->extra_data == 0.5);
}

TEST_CASE("CreateRetractionLift creates a symmetric ramped travel",
          "[plugins][layer-extrusion-edit][lift][travel]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_lift_print(print, model, "1", "1", "0");

    run_layer_plugins(print, {
        TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, ENTRY_STATE_PLUGIN,
        TRAVEL_PLUGIN, LIFT_PLUGIN
    });

    PrintingToolGroup &tool = print.mutable_printing_plan().groups.front()
        .layers.front().tool_groups.front();
    slic3r_api::MutableExtrusionEntity target = find_scope(
        entity_view(*tool.extrusions[1].root));
    REQUIRE(target.valid());
    OrderedExtrusionScope scope(target, scope_property_key());
    const slic3r_api::MutableExtrusionEntity travel = scope.travel();
    REQUIRE(travel.child_count() == 2);
    const slic3r_api::ExtrusionEntity ascending = travel.child(0);
    const slic3r_api::ExtrusionEntity descending = travel.child(1);
    REQUIRE(ascending.point_count() >= 2);
    REQUIRE(descending.point_count() >= 2);
    CHECK(ascending.z_offset(0) == 0);
    CHECK(ascending.z_offset(ascending.point_count() - 1) == scale_i(1.0));
    CHECK(descending.z_offset(0) == scale_i(1.0));
    CHECK(descending.z_offset(descending.point_count() - 1) == 0);
}

// Absolute bounds apply to both endpoints of a descending path, including the climb.
TEST_CASE("Descending lift stays below the printer ceiling",
          "[plugins][lift][travel][lift-min-ceiling]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_lift_print(print, model, "0.5", "0", "0", "0", false, "100%", "20%", false, false, "5", "10");
    PrintingGroup &group = print.mutable_printing_plan().groups.front();
    PrintingExtrusion target = std::move(group.layers.front().tool_groups.front().extrusions.back());
    group.layers.front().tool_groups.front().extrusions.pop_back();
    group.layers.front().print_z = scale_i(9.0);
    group.layers.emplace_back();
    PrintingLayerGroup &layer = group.layers.back();
    layer.print_z = scale_i(0.2);
    layer.tool_groups.emplace_back();
    layer.tool_groups.back().extruder_id = 0;
    layer.tool_groups.back().extrusions.push_back(std::move(target));
    run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, ENTRY_STATE_PLUGIN, TRAVEL_PLUGIN, LIFT_PLUGIN});
    OrderedExtrusionScope scope(find_scope(entity_view(*layer.tool_groups.front().extrusions.front().root)), scope_property_key());
    std::vector<LiftLeaf> leaves;
    collect_lift_leaves(scope.travel().readonly(), std::nullopt, leaves);
    REQUIRE_FALSE(leaves.empty());
    bool horizontal = false;
    for (const LiftLeaf &leaf : leaves)
        for (const c_extrusion_segment &segment : leaf.entity.segments()) {
            const double a = unscaled(layer.print_z + segment.z_offset_a);
            const double b = unscaled(layer.print_z + segment.z_offset_b);
            CHECK(a <= 10.0); CHECK(b <= 10.0);
            if (segment.point_a.x != segment.point_b.x || segment.point_a.y != segment.point_b.y) {
                horizontal = true;
                CHECK(a >= 5.0); CHECK(b >= 5.0);
            }
        }
    CHECK(horizontal);
}

// Source-free auxiliary paths use distance, not a shared missing island identity.
TEST_CASE("Auxiliary lift floor starts at ten nozzle diameters",
          "[plugins][lift][travel][lift-min-auxiliary]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const double distance = GENERATE(1.0, 4.0, 10.0);
    Print print;
    Model model;
    prepare_lift_print(print, model, "0.5", "0", "0", "0", false, "100%", "20%", false, false, "5", "10");
    PrintingLayerGroup &layer = print.mutable_printing_plan().groups.front().layers.front();
    PrintingToolGroup &tool = layer.tool_groups.front();
    tool.extrusions.back().root = CanonicalExtrusion::acquire(*make_print_path(10.0 + distance, 20.0 + distance));
    run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, ENTRY_STATE_PLUGIN, TRAVEL_PLUGIN, LIFT_PLUGIN});
    OrderedExtrusionScope scope(find_scope(entity_view(*tool.extrusions.back().root)), scope_property_key());
    std::vector<LiftLeaf> leaves;
    collect_lift_leaves(scope.travel().readonly(), std::nullopt, leaves);
    size_t horizontal = 0;
    for (const LiftLeaf &leaf : leaves)
        for (const c_extrusion_segment &segment : leaf.entity.segments())
            if (segment.point_a.x != segment.point_b.x || segment.point_a.y != segment.point_b.y) {
                ++horizontal;
                CHECK(unscaled(layer.print_z + segment.z_offset_a) == Approx(distance >= 4.0 ? 5.0 : 0.7));
                CHECK(unscaled(layer.print_z + segment.z_offset_b) == Approx(distance >= 4.0 ? 5.0 : 0.7));
            }
    CHECK(horizontal > 0);
}

TEST_CASE("CreateRetractionLift carries source lift across an empty layer",
          "[plugins][layer-extrusion-edit][lift][parallel]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_lift_print(print, model, "0.6", "1", "0");
    REQUIRE(print.config().retract_lift.get_at(0) == Approx(0.6));

    PrintingGroup &group = print.mutable_printing_plan().groups.front();
    PrintingExtrusion target = std::move(
        group.layers.front().tool_groups.front().extrusions.back());
    group.layers.front().tool_groups.front().extrusions.pop_back();
    group.layers.emplace_back();
    group.layers.back().print_z = scale_i(0.3);
    group.layers.emplace_back();
    PrintingLayerGroup &target_layer = group.layers.back();
    target_layer.print_z = scale_i(0.4);
    target_layer.tool_groups.emplace_back();
    target_layer.tool_groups.back().extruder_id = 0;
    target_layer.tool_groups.back().extrusions.push_back(std::move(target));

    run_layer_plugins(print, {
        TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, ENTRY_STATE_PLUGIN,
        TRAVEL_PLUGIN, LIFT_PLUGIN
    });

    PrintingToolGroup &target_tool = target_layer.tool_groups.front();
    OrderedExtrusionScope scope(
        find_scope(entity_view(*target_tool.extrusions.front().root)),
        scope_property_key());
    const slic3r_api::MutableExtrusionEntity travel = scope.travel();
    REQUIRE(travel.child_count() == 2);
    // The original travel climbs from Z 0.2 to Z 0.4. Lift is added over that
    // existing profile, so the midpoint baseline is -0.1 relative to layer Z.
    CHECK(travel.child(0).z_offset(0) == -scale_i(0.2));
    CHECK(travel.child(0).z_offset(
              travel.child(0).point_count() - 1) == scale_i(0.5));
    CHECK(travel.child(1).z_offset(
              travel.child(1).point_count() - 1) == 0);
}

TEST_CASE("CreateRetractionLift materializes vertical lift and respects its threshold",
          "[plugins][layer-extrusion-edit][lift][travel]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("vertical profile") {
        Print print;
        Model model;
        prepare_lift_print(print, model, "0.8", "0", "0");
        run_layer_plugins(print, {
            TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, ENTRY_STATE_PLUGIN,
            TRAVEL_PLUGIN, LIFT_PLUGIN
        });

        PrintingToolGroup &tool = print.mutable_printing_plan().groups.front()
            .layers.front().tool_groups.front();
        OrderedExtrusionScope scope(
            find_scope(entity_view(*tool.extrusions[1].root)),
            scope_property_key());
        const slic3r_api::MutableExtrusionEntity travel = scope.travel();
        REQUIRE(travel.segment_count() == 3);
        CHECK(travel.segment(0).point_a.x == travel.segment(0).point_b.x);
        CHECK(travel.segment(0).point_a.y == travel.segment(0).point_b.y);
        CHECK(travel.segment(0).z_offset_a == 0);
        CHECK(travel.segment(0).z_offset_b == scale_i(0.8));
        CHECK(travel.segment(2).point_a.x == travel.segment(2).point_b.x);
        CHECK(travel.segment(2).point_a.y == travel.segment(2).point_b.y);
        CHECK(travel.segment(2).z_offset_a == scale_i(0.8));
        CHECK(travel.segment(2).z_offset_b == 0);
    }

    SECTION("travel at threshold") {
        Print print;
        Model model;
        prepare_lift_print(print, model, "0.8", "0", "0", "10");
        run_layer_plugins(print, {
            TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, ENTRY_STATE_PLUGIN,
            TRAVEL_PLUGIN, LIFT_PLUGIN
        });

        PrintingToolGroup &tool = print.mutable_printing_plan().groups.front()
            .layers.front().tool_groups.front();
        OrderedExtrusionScope scope(
            find_scope(entity_view(*tool.extrusions[1].root)),
            scope_property_key());
        const slic3r_api::MutableExtrusionEntity travel = scope.travel();
        REQUIRE(travel.segment_count() == 1);
        CHECK(travel.segment(0).z_offset_a == 0);
        CHECK(travel.segment(0).z_offset_b == 0);
    }
}

// Split the generated 10 mm travel as the deceleration provider does, then check one lift profile across all its ordered leaves.
TEST_CASE("CreateRetractionLift lifts a travel split into ordered leaves",
          "[plugins][layer-extrusion-edit][lift][travel]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    enum class Profile { Vertical, Triangle, Plateau };
    const Profile profile = GENERATE(Profile::Vertical, Profile::Triangle, Profile::Plateau);
    CAPTURE(static_cast<int>(profile));
    Print print;
    Model model;
    // A 45 degree slope with a 1 mm lift ramps over 1 mm at each end.
    prepare_lift_print(print, model, "1", profile == Profile::Vertical ? "0" : "1",
                       profile == Profile::Plateau ? "45" : "0");
    run_layer_plugins(print, {
        TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, ENTRY_STATE_PLUGIN, TRAVEL_PLUGIN
    });
    PrintingToolGroup &tool = print.mutable_printing_plan().groups.front()
        .layers.front().tool_groups.front();
    OrderedExtrusionScope scope(find_scope(entity_view(*tool.extrusions[1].root)),
                                scope_property_key());
    REQUIRE(scope.travel().segment_count() == 1);
    REQUIRE(scope.travel().local_length() == Approx(scale_d(10.)));
    const slic3r_api::MutableExtrusionEntity final_part = split_travel_leaf(scope.travel(), 7.);
    run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, LIFT_PLUGIN});

    std::vector<LiftLeaf> leaves;
    collect_lift_leaves(scope.travel().readonly(), std::nullopt, leaves);
    // The ramp ends become leaf boundaries inside the existing leaves.
    REQUIRE(leaves.size() == (profile == Profile::Vertical ? 2 : profile == Profile::Triangle ? 3 : 4));
    for (size_t idx = 0; idx < leaves.size(); ++idx) {
        CHECK(RAW_EXTRUSION_ROLE_IS_TRAVEL(leaves[idx].role));
        if (idx == 0)
            continue;
        const slic3r_api::ExtrusionEntity &previous = leaves[idx - 1].entity;
        const slic3r_api::ExtrusionEntity &next = leaves[idx].entity;
        CHECK(previous.point(previous.point_count() - 1).x == next.point(0).x);
        CHECK(previous.point(previous.point_count() - 1).y == next.point(0).y);
        CHECK(previous.z_offset(previous.point_count() - 1) == next.z_offset(0));
    }

    // The final part keeps its acceleration and still holds every point beyond its original start.
    const slic3r_api::EPropertySpeed *speed = final_part.get(slic3r_api::EPropertySpeed::key);
    REQUIRE(speed != nullptr);
    CHECK(speed->accel_mm_per_s2 == 1234.f);
    std::vector<LiftLeaf> final_leaves;
    const slic3r_api::EPropertyAttributes *travel_attributes =
        scope.travel().get(slic3r_api::EPropertyAttributes::key);
    REQUIRE(travel_attributes != nullptr);
    collect_lift_leaves(final_part.readonly(), *travel_attributes, final_leaves);
    REQUIRE_FALSE(final_leaves.empty());
    CHECK(final_leaves.front().entity.point(0).x == scale_i(17.));
    CHECK(final_leaves.back().entity.same_handle(leaves.back().entity));

    const slic3r_api::ExtrusionEntity &first = leaves.front().entity;
    const slic3r_api::ExtrusionEntity &last = leaves.back().entity;
    CHECK(first.point(0).x == scale_i(10.));
    CHECK(first.z_offset(0) == 0);
    CHECK(last.point(last.point_count() - 1).x == scale_i(20.));
    CHECK(last.z_offset(last.point_count() - 1) == 0);
    if (profile == Profile::Vertical) {
        // Only the explicit vertical segments at both travel ends leave the lifted height.
        CHECK(first.point(1).x == scale_i(10.));
        CHECK(last.point(last.point_count() - 2).x == scale_i(20.));
        for (const LiftLeaf &leaf : leaves)
            for (uint32_t idx = 0; idx < leaf.entity.point_count(); ++idx) {
                const bool travel_start = leaf.entity.same_handle(first) && idx == 0;
                const bool travel_end = leaf.entity.same_handle(last) && idx + 1 == leaf.entity.point_count();
                if (!travel_start && !travel_end)
                    CHECK(leaf.entity.z_offset(idx) == scale_i(1.));
            }
        return;
    }

    // Every point follows the triangle or trapezoid along its planar distance from the start.
    std::vector<double> breakpoints;
    for (const LiftLeaf &leaf : leaves)
        for (uint32_t idx = 0; idx < leaf.entity.point_count(); ++idx) {
            const double distance = unscaled(leaf.entity.point(idx).x) - 10.;
            const double expected = profile == Profile::Triangle ?
                std::min(distance, 10. - distance) / 5. :
                std::min({1., distance, 10. - distance});
            CAPTURE(distance);
            CHECK(unscaled(leaf.entity.z_offset(idx)) == Approx(expected).margin(1e-5));
            if (leaf.entity.z_offset(idx) == scale_i(1.))
                breakpoints.push_back(distance);
        }
    REQUIRE_FALSE(breakpoints.empty());
    CHECK(*std::min_element(breakpoints.begin(), breakpoints.end()) ==
          Approx(profile == Profile::Triangle ? 5. : 1.).margin(1e-5));
    CHECK(*std::max_element(breakpoints.begin(), breakpoints.end()) ==
          Approx(profile == Profile::Triangle ? 5. : 9.).margin(1e-5));
}

// Check shared ramp values and joins across outgoing wipe leaves, then verify the next travel inherits the final Z.
TEST_CASE("CreateRetractionLift raises the whole outgoing wipe continuously",
          "[plugins][layer-extrusion-edit][lift][wipe]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    const char *length = GENERATE("4", "8", "16", "50%", "100%", "200%");
    const char *before = GENERATE("20%", "100%");
    const int outgoing = GENERATE(0, 1, 2);
    const bool returning = outgoing == 1;
    const bool inside_end = outgoing == 2;
    CAPTURE(length, before, returning, inside_end);
    prepare_lift_print(print, model, "0", "0", "0", "0", true, length, before, returning, inside_end);

    run_layer_plugins(print, {
        TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, WIPE_PLUGIN,
        ENTRY_STATE_PLUGIN, TRAVEL_PLUGIN, LIFT_PLUGIN
    });

    PrintingToolGroup &tool = print.mutable_printing_plan().groups.front()
        .layers.front().tool_groups.front();
    OrderedExtrusionScope source(
        find_scope(entity_view(*tool.extrusions[0].root)),
        scope_property_key());
    std::vector<LiftLeaf> leaves;
    collect_lift_leaves(source.after().readonly(), std::nullopt, leaves);

    std::vector<LiftLeaf> wipes;
    double total = 0.;
    for (const LiftLeaf &leaf : leaves)
        if (RAW_EXTRUSION_ROLE_IS_WIPE(leaf.role)) {
            wipes.push_back(leaf);
            total += leaf.entity.local_length();
        }
    REQUIRE_FALSE(wipes.empty());
    if (inside_end && !returning) {
        REQUIRE(wipes.back().entity.segment_count() > 0);
        const c_extrusion_segment segment = wipes.back().entity.segment(wipes.back().entity.segment_count() - 1);
        CHECK(segment.point_a.x == segment.point_b.x);
        CHECK(segment.point_a.y != segment.point_b.y);
        CHECK(std::abs(segment.point_b.y - segment.point_a.y) == scale_i(0.2));
    }
    if (std::string(before) == "20%" && !returning && !inside_end) REQUIRE(wipes.size() == 2);
    const std::string setting(length);
    const double ramp = std::min(total, setting.back() == '%' ?
        total * std::stod(setting) / 100. : scale_d(std::stod(setting)));
    double traversed = 0.;
    coord_t previous = 0;
    for (const LiftLeaf &leaf : wipes) {
        CHECK(leaf.entity.z_offset(0) == previous);
        for (uint32_t idx = 0; idx < leaf.entity.point_count(); ++idx) {
            if (idx > 0) {
                const c_extrusion_segment segment = leaf.entity.segment(idx - 1);
                traversed += std::hypot(double(segment.point_b.x - segment.point_a.x),
                                        double(segment.point_b.y - segment.point_a.y));
            }
            const coord_t actual = leaf.entity.z_offset(idx);
            const double factor = std::clamp((traversed - (total - ramp)) / ramp, 0., 1.);
            CHECK(actual == Approx(double(scale_i(1.0)) * factor).margin(1.));
            CHECK(actual >= previous);
            previous = actual;
        }
    }
    CHECK(previous == scale_i(1.0));
    OrderedExtrusionScope target(
        find_scope(entity_view(*tool.extrusions[1].root)), scope_property_key());
    CHECK(target.travel().segment(0).z_offset_a == previous);
}

// Check the running maximum of original path Z plus ramp on a reversed, descending arc without moving retraction.
TEST_CASE("CreateRetractionLift never lowers a non-planar wipe", "[plugins][layer-extrusion-edit][lift][wipe]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const double height = GENERATE(0.4, 2.);
    const char *before = GENERATE("100%", "20%");
    CAPTURE(height, before);
    Print print;
    Model model;
    prepare_lift_print(print, model, "0", "0", "0", "0", true, "50%", before);
    PrintingToolGroup &tool = print.mutable_printing_plan().groups.front().layers.front().tool_groups.front();
    slic3r_api::MutableExtrusionEntity path(reinterpret_cast<extrusion_entity_handle *>(tool.extrusions[0].root.get()));
    c_extrusion_segment segment = path.segment(0);
    segment.radius = float(scale_i(10.));
    segment.orientation = RAW_EXTRUSION_ARC_ORIENTATION_CCW;
    segment.z_offset_b = scale_i(height);
    REQUIRE(path.set_segments({segment}));
    const double source_length = path.local_length();
    run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, WIPE_PLUGIN,
        ENTRY_STATE_PLUGIN, TRAVEL_PLUGIN, LIFT_PLUGIN});
    OrderedExtrusionScope source(find_scope(entity_view(*tool.extrusions[0].root)), scope_property_key());
    std::vector<LiftLeaf> leaves;
    collect_lift_leaves(source.after().readonly(), std::nullopt, leaves);
    double total = 0.;
    for (const LiftLeaf &leaf : leaves)
        if (RAW_EXTRUSION_ROLE_IS_WIPE(leaf.role)) total += leaf.entity.local_length();
    REQUIRE(total > 0.);
    double distance = 0.;
    double expected = scale_i(height);
    coord_t previous = scale_i(height);
    size_t wipe_count = 0;
    for (const LiftLeaf &leaf : leaves) {
        if (!RAW_EXTRUSION_ROLE_IS_WIPE(leaf.role)) continue;
        ++wipe_count;
        if (std::string(before) == "100%") CHECK_FALSE(RAW_EXTRUSION_ROLE_IS_RETRACT(leaf.role));
        for (uint32_t idx = 0; idx < leaf.entity.point_count(); ++idx) {
            if (idx > 0) {
                const c_extrusion_segment s = leaf.entity.segment(idx - 1);
                REQUIRE(s.radius != 0.f);
                const double chord = std::hypot(double(s.point_b.x - s.point_a.x), double(s.point_b.y - s.point_a.y));
                distance += 2. * std::abs(double(s.radius)) * std::asin(chord / (2. * std::abs(double(s.radius))));
            }
            const double ramp = scale_i(1.) * std::clamp((distance - total * 0.5) / (total * 0.5), 0., 1.);
            expected = std::max(expected, scale_i(height) * (1. - distance / source_length) + ramp);
            CHECK(leaf.entity.z_offset(idx) == Approx(expected).margin(3.));
            CHECK(leaf.entity.z_offset(idx) >= previous);
            previous = leaf.entity.z_offset(idx);
        }
    }
    CHECK(wipe_count == (std::string(before) == "100%" ? 1 : 2));
    CHECK(previous == Approx(expected).margin(3.));
    OrderedExtrusionScope target(find_scope(entity_view(*tool.extrusions[1].root)), scope_property_key());
    CHECK(target.travel().segment(0).z_offset_a == previous);
}

// Verify an explicit zero-lift vertex inside either wipe leaf, including true arcs, without changing the retraction target.
TEST_CASE("CreateRetractionLift inserts the exact ramp start", "[plugins][layer-extrusion-edit][lift][wipe]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool arc = GENERATE(false, true);
    const char *length = GENERATE("4", "7.8");
    CAPTURE(arc, length);
    Print print;
    Model model;
    prepare_lift_print(print, model, "0", "0", "0", "0", true, length);
    const double ramp_length = scale_d(std::stod(length));
    PrintingToolGroup &tool = print.mutable_printing_plan().groups.front().layers.front().tool_groups.front();
    if (arc) {
        slic3r_api::MutableExtrusionEntity path(reinterpret_cast<extrusion_entity_handle *>(tool.extrusions[0].root.get()));
        c_extrusion_segment segment = path.segment(0);
        segment.radius = float(scale_i(10.));
        segment.orientation = RAW_EXTRUSION_ARC_ORIENTATION_CCW;
        REQUIRE(path.set_segments({segment}));
    }
    run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, WIPE_PLUGIN,
        ENTRY_STATE_PLUGIN, TRAVEL_PLUGIN, LIFT_PLUGIN});
    OrderedExtrusionScope source(find_scope(entity_view(*tool.extrusions[0].root)), scope_property_key());
    std::vector<LiftLeaf> leaves;
    collect_lift_leaves(source.after().readonly(), std::nullopt, leaves);
    double total = 0.;
    for (const LiftLeaf &leaf : leaves)
        if (RAW_EXTRUSION_ROLE_IS_WIPE(leaf.role)) total += leaf.entity.local_length();
    double distance = 0.;
    bool found = false;
    size_t wipe_count = 0;
    for (const LiftLeaf &leaf : leaves) {
        if (!RAW_EXTRUSION_ROLE_IS_WIPE(leaf.role)) continue;
        ++wipe_count;
        if (RAW_EXTRUSION_ROLE_IS_RETRACT(leaf.role)) {
            const slic3r_api::EPropertyExtrusionAxis *axis = leaf.entity.get(slic3r_api::EPropertyExtrusionAxis::key);
            REQUIRE(axis != nullptr);
            CHECK(axis->operation == C_EXTRUSION_AXIS_OPERATION_RETRACT_TO);
            CHECK(axis->value == Approx(2.));
        }
        for (uint32_t idx = 0; idx < leaf.entity.point_count(); ++idx) {
            if (idx > 0) {
                const c_extrusion_segment segment = leaf.entity.segment(idx - 1);
                const double chord = std::hypot(double(segment.point_b.x - segment.point_a.x),
                                                double(segment.point_b.y - segment.point_a.y));
                distance += segment.radius == 0.f ? chord :
                    2. * std::abs(double(segment.radius)) * std::asin(chord / (2. * std::abs(double(segment.radius))));
                if (arc) CHECK(segment.radius != 0.f);
            }
            if (std::abs(distance - (total - ramp_length)) < 3.) {
                CHECK(leaf.entity.z_offset(idx) == 0);
                found = true;
            }
            const double factor = std::clamp((distance - (total - ramp_length)) / ramp_length, 0., 1.);
            CHECK(leaf.entity.z_offset(idx) == Approx(scale_i(1.) * factor).margin(2.));
        }
    }
    CHECK(found);
    CHECK(wipe_count == 2);
}

} // namespace
