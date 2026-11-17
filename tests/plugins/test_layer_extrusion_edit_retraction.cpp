///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
CreateRetraction tests
======================

The cases verify the compact-scope contract first: a source owns Retract in
after, while the target owns Unretract in before. Physical tool selection is
tested separately through CreateToolChange because it belongs to the target
PrintingToolGroup rather than to a compact scope. Plan-terminal retraction is
checked separately because it belongs to STEP_EXTRUSION_EDIT.
Fixture replacements acquire canonical nodes when they enter the plan, before
the production stage validates them; plugin execution never repairs these trees.
*/

#include "layer_extrusion_edit_transition_test_helpers.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepPostSlicing.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"

namespace {

using namespace Slic3r;
using namespace Slic3r::Printing;
using namespace Slic3r::Test::TransitionPipeline;
using slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope;

constexpr const char *ENTRY_STATE_PLUGIN =
    "layer_extrusion_edit.entry_state.default";
constexpr const char *TRAVEL_PLUGIN = "layer_extrusion_edit.travel.default";

TEST_CASE("CreateRetraction declares the settings used by crossing decisions",
          "[plugins][layer-extrusion-edit][retraction][crossing-policy]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const Plugin *plugin = Orchestrator::instance().get_plugin(RETRACTION_PLUGIN);
    REQUIRE(plugin != nullptr);
    const std::vector<Plugin::UsedConfigKey> keys = plugin->get_used_config_keys();
    for (const char *name : {"only_retract_when_crossing_perimeters", "enforce_retract_first_layer",
                            "fill_density", "avoid_crossing_perimeters", "nozzle_diameter"}) {
        CAPTURE(name);
        CHECK(std::any_of(keys.begin(), keys.end(), [name](const Plugin::UsedConfigKey &key) {
            return key.key == name;
        }));
    }
}

TEST_CASE("CreateRetraction honors interior crossing suppression and its first-layer exemption",
          "[plugins][layer-extrusion-edit][retraction][crossing-policy]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool crossing_only = GENERATE(false, true);
    const bool first_layer = GENERATE(false, true);
    const bool enforce_first_layer = GENERATE(false, true);
    CAPTURE(crossing_only, first_layer, enforce_first_layer);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"only_retract_when_crossing_perimeters", crossing_only ? "1" : "0"},
        {"enforce_retract_first_layer", enforce_first_layer ? "1" : "0"},
        {"retract_before_travel", "0.5"}, {"retract_layer_change", "0"},
        {"retract_length", "2"}, {"fill_density", "20%"}
    });
    Print print;
    Model model;
    Slic3r::Test::init_print(
        {Slic3r::Test::TestMesh::cube_20x20x20}, print, model, config);
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepLayerHeightGeneration::run_step(orchestrator, print);
    Steps::StepSlicing::run_step(orchestrator, print);
    Steps::StepPostSlicing::run_step(orchestrator, print);
    PrintObject &object = print.object(0);
    REQUIRE(object.layer_count() > 1);
    Layer &source_layer = object.layer(first_layer ? 0 : 1);
    LayerSliceIsland &island = source_layer.island(0);
    LayerRegionIsland &region = island.get_or_add_region_island(island.regions(), 0);
    const Point center = get_extents(source_layer.lslices()).center() + object.instances().front().shift;

    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.clear();
    const PrintingObjectInstanceId catalog_id = plan.find_or_add_object_instance(object, 0);
    plan.groups.emplace_back();
    plan.groups.back().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.back().layers.back();
    layer.print_z = source_layer.scaled_print_z();
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool = layer.tool_groups.back();
    tool.extruder_id = 0;
    for (const double x : {-3., 1.}) {
        std::unique_ptr<ExtrusionPath> path = make_print_path(0., 1.);
        path->set_polyline(ArcPolyline(Points{
            center + Point(scale_i(x), 0), center + Point(scale_i(x + 1.), 0)}));
        append_printing_extrusion(tool, path->clone_canonical());
        tool.extrusions.back().region_island = &region;
        tool.extrusions.back().printing_catalog_id = catalog_id;
    }

    run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, ENTRY_STATE_PLUGIN, TRAVEL_PLUGIN});
    const OrderedExtrusionScope source(
        find_scope(entity_view(*tool.extrusions.front().root)), scope_property_key());
    const OrderedExtrusionScope target(
        find_scope(entity_view(*tool.extrusions.back().root)), scope_property_key());
    std::vector<std::string> outgoing;
    std::vector<std::string> incoming;
    collect_transition_events(source.after().readonly(), outgoing);
    collect_transition_events(target.before().readonly(), incoming);
    const bool expected = !crossing_only || (first_layer && enforce_first_layer);
    CHECK(outgoing == (expected ? std::vector<std::string>{"retract"} : std::vector<std::string>{}));
    CHECK(incoming == (expected ? std::vector<std::string>{"unretract"} : std::vector<std::string>{}));
}

TEST_CASE("CreateRetraction classifies a routed hole crossing without bypassing thresholds or modifiers",
          "[plugins][layer-extrusion-edit][retraction][crossing-policy]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool avoiding = GENERATE(false, true);
    const int mode = GENERATE(0, 1, 2, 3);
    CAPTURE(avoiding, mode);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"only_retract_when_crossing_perimeters", "1"}, {"enforce_retract_first_layer", "0"},
        {"avoid_crossing_perimeters", avoiding ? "1" : "0"}, {"avoid_crossing_not_first_layer", "0"},
        {"avoid_crossing_perimeters_max_detour", "0"}, {"external_perimeter_extrusion_width", "1.2"},
        {"retract_before_travel", mode == 1 ? "100" : "0.5"}, {"retract_layer_change", "0"},
        {"retract_length", "2"}, {"fill_density", "20%"}
    });
    // A sliced normal layer with a central hole: auxiliary layers are off object and are not routed.
    Print print;
    Model model;
    Slic3r::Test::init_print({Slic3r::Test::TestMesh::cube_with_hole}, print, model, config);
    print.set_status_silent();
    print.process();
    print.pipeline_progress().reset();
    PrintObject &object = print.object(0);
    REQUIRE(object.layer_count() > 1);
    Layer &source_layer = object.layer(1);
    REQUIRE(source_layer.lslices().size() == 1);
    REQUIRE(source_layer.lslices().front().holes.size() == 1);
    LayerSliceIsland &island = source_layer.island(0);
    LayerRegionIsland &region = island.get_or_add_region_island(island.regions(), 0);
    // The hole spans 5 mm on each side of the center and the part 10 mm.
    const Point center = get_extents(source_layer.lslices()).center() + object.instances().front().shift;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.clear();
    const PrintingObjectInstanceId catalog = plan.find_or_add_object_instance(object, 0);
    plan.groups.emplace_back();
    // Like a built plan, the group lists the instance it prints.
    plan.groups.back().printing_catalog_ids.push_back(catalog);
    plan.groups.back().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.back().layers.back();
    layer.print_z = source_layer.scaled_print_z();
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool = layer.tool_groups.back();
    tool.extruder_id = 0;
    for (const double x : {-8., 7.}) {
        std::unique_ptr<ExtrusionPath> path = make_print_path(0., 1.);
        path->set_polyline(ArcPolyline(Points{center + Point(scale_i(x), 0), center + Point(scale_i(x + 1.), 0)}));
        if (mode == 2) path->get_or_add_property<ExtrusionPropertyModifier>().enforce_retraction = 1;
        if (mode == 3) path->get_or_add_property<ExtrusionPropertyModifier>().disable_retraction = 1;
        append_printing_extrusion(tool, path->clone_canonical());
        tool.extrusions.back().region_island = &region;
        tool.extrusions.back().printing_catalog_id = catalog;
    }
    const char *travel = avoiding ? "layer_extrusion_edit.travel.avoid_crossing_perimeters" : TRAVEL_PLUGIN;
    run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, ENTRY_STATE_PLUGIN, travel});
    const OrderedExtrusionScope source(find_scope(entity_view(*tool.extrusions.front().root)), scope_property_key());
    const OrderedExtrusionScope target(find_scope(entity_view(*tool.extrusions.back().root)), scope_property_key());
    std::vector<std::string> outgoing;
    std::vector<std::string> incoming;
    collect_transition_events(source.after().readonly(), outgoing);
    collect_transition_events(target.before().readonly(), incoming);
    const bool expected = mode == 2 || (mode == 0 && !avoiding);
    CHECK(outgoing == (expected ? std::vector<std::string>{"retract"} : std::vector<std::string>{}));
    CHECK(incoming == (expected ? std::vector<std::string>{"unretract"} : std::vector<std::string>{}));
    if (avoiding)
        CHECK(target.travel().segment_count() > 1);
    else
        CHECK(target.travel().segment_count() == 1);
}

/* Return the second printable extrusion used as the target in these fixtures. */
PrintingExtrusion &target_extrusion(Print &print)
{
    return print.mutable_printing_plan().groups.front().layers.front()
        .tool_groups.front().extrusions.back();
}

/* Acquire a canonical target tree while keeping its plan ownership unchanged. */
void replace_target_root(Print &print, std::unique_ptr<ExtrusionEntity> root)
{
    target_extrusion(print).root = CanonicalExtrusion::acquire(*root);
}

/* Build an outer inherited-role parent and an inner fixed scope candidate. */
std::unique_ptr<ExtrusionEntity> make_nested_target_root(
    std::unique_ptr<ExtrusionEntity> path,
    const bool override_child_role)
{
    if (override_child_role) {
        ExtrusionAttributes *attributes =
            path->get_property<ExtrusionAttributes>();
        if (attributes == nullptr)
            throw std::runtime_error("The role fixture path has no attributes.");
        attributes->set_role(ExtrusionRole::ExternalPerimeter);
    } else {
        path->remove_property<ExtrusionAttributes>();
    }

    std::unique_ptr<ExtrusionEntityCollection> inner =
        std::make_unique<ExtrusionEntityCollection>(false, false);
    inner->append(std::move(path));

    std::unique_ptr<ExtrusionEntityCollection> outer =
        std::make_unique<ExtrusionEntityCollection>(true, true);
    outer->get_or_add_property<ExtrusionAttributes>().set_role(
        ExtrusionRole::InternalInfill);
    outer->append(std::move(inner));
    return std::unique_ptr<ExtrusionEntity>(std::move(outer));
}

/* Read the semantic role copied onto the target's Unretract event. */
raw_extrusion_role target_unretract_role(Print &print)
{
    const OrderedExtrusionScope target(
        find_scope(entity_view(*target_extrusion(print).root)),
        scope_property_key());
    const slic3r_api::EPropertyAttributes *attributes =
        target.before().get(slic3r_api::EPropertyAttributes::key);
    return attributes == nullptr ? RAW_EXTRUSION_ROLE_NONE :
        attributes->extrusion_role();
}

TEST_CASE("CreateRetraction registers both transition providers",
          "[plugins][layer-extrusion-edit][retraction][registration]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    Plugin *retraction = orchestrator.get_plugin(RETRACTION_PLUGIN);
    Plugin *terminal = orchestrator.get_plugin(TERMINAL_RETRACTION_PLUGIN);
    REQUIRE(retraction != nullptr);
    REQUIRE(terminal != nullptr);
    CHECK(retraction->get_priority() == -90);
    CHECK(retraction->get_exclusive_group() ==
          "layer_extrusion_edit.retraction");
    REQUIRE_FALSE(retraction->get_dependencies().empty());
    CHECK(retraction->get_dependencies().front() == TRANSITION_SCOPE_PLUGIN);
    CHECK(terminal->get_priority() == -100);
    CHECK(terminal->get_exclusive_group() ==
          "extrusion_edit.terminal_retraction");

    // Only the primary provider defines print_retract_length; the terminal
    // provider reads it without publishing a definition of its own.
    std::vector<std::string> defined_keys;
    for (const Plugin::DefinedConfigKey &key : retraction->get_defined_config_keys())
        defined_keys.push_back(key.key);
    CHECK(defined_keys.size() == 1);
    CHECK(defined_keys.front() == "print_retract_length");
    CHECK(terminal->get_defined_config_keys().empty());

    const ConfigOptionDef *def = PrintConfigDef::instance().get("print_retract_length");
    REQUIRE(def != nullptr);
    CHECK(def->type == coFloat);
    CHECK(def->container_type == ConfigOptionContainerType::Region);
    CHECK(def->invalidates_step == STEP_LAYER_EXTRUSION_EDIT);
    CHECK(def->can_be_disabled);
    CHECK(def->default_value->serialize() == "!0");
    CHECK(PrintConfig::defaults().option("print_retract_length") == nullptr);
    CHECK(PrintRegionConfig::defaults().option("print_retract_length") != nullptr);
}

TEST_CASE("CreateRetraction places semantic E events in compact phases",
          "[plugins][layer-extrusion-edit][retraction][gap]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_print(print, model);
    run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN});

    PrintingToolGroup &tool = print.mutable_printing_plan().groups.front()
        .layers.front().tool_groups.front();
    slic3r_api::MutableExtrusionEntity source_root = find_scope(
        entity_view(*tool.extrusions.front().root));
    slic3r_api::MutableExtrusionEntity target_root = find_scope(
        entity_view(*tool.extrusions.back().root));
    REQUIRE(source_root.valid());
    REQUIRE(target_root.valid());
    OrderedExtrusionScope source(source_root, scope_property_key());
    OrderedExtrusionScope target(target_root, scope_property_key());

    std::vector<std::string> source_events;
    std::vector<std::string> target_events;
    collect_transition_events(source.after().readonly(), source_events);
    collect_transition_events(target.before().readonly(), target_events);
    CHECK(source_events == std::vector<std::string>{"retract"});
    CHECK(target_events == std::vector<std::string>{"unretract"});

    const slic3r_api::EPropertyExtrusionAxis *retract_axis =
        source.after().get(slic3r_api::EPropertyExtrusionAxis::key);
    const slic3r_api::EPropertyExtrusionAxis *unretract_axis =
        target.before().get(slic3r_api::EPropertyExtrusionAxis::key);
    REQUIRE(retract_axis != nullptr);
    REQUIRE(unretract_axis != nullptr);
    CHECK(retract_axis->operation == C_EXTRUSION_AXIS_OPERATION_RETRACT_TO);
    CHECK(retract_axis->value == Approx(2.0));
    CHECK(unretract_axis->operation == C_EXTRUSION_AXIS_OPERATION_UNRETRACT);
    CHECK(unretract_axis->restart_extra == Approx(0.1));
    CHECK(target_unretract_role(print) ==
          (RAW_EXTRUSION_ROLE_PERIMETER | RAW_EXTRUSION_ROLE_UNRETRACT));
}

TEST_CASE("CreateRetraction inherits an effective role from a scope parent",
          "[plugins][layer-extrusion-edit][retraction][role]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_print(print, model);
    std::unique_ptr<ExtrusionEntity> path =
        std::move(target_extrusion(print).root);
    replace_target_root(print, make_nested_target_root(std::move(path), false));

    run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN});

    CHECK(target_unretract_role(print) ==
          (RAW_EXTRUSION_ROLE_INFILL | RAW_EXTRUSION_ROLE_UNRETRACT));
}

TEST_CASE("CreateRetraction lets a direct content role override its parent",
          "[plugins][layer-extrusion-edit][retraction][role]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_print(print, model);
    std::unique_ptr<ExtrusionEntity> path =
        std::move(target_extrusion(print).root);
    replace_target_root(print, make_nested_target_root(std::move(path), true));

    run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN});

    CHECK(target_unretract_role(print) ==
          (RAW_EXTRUSION_ROLE_PERIMETER | RAW_EXTRUSION_ROLE_EXTERNAL |
           RAW_EXTRUSION_ROLE_UNRETRACT));
}

TEST_CASE("CreateRetraction stops role lookup at the first geometric leaf",
          "[plugins][layer-extrusion-edit][retraction][role]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_print(print, model);

    std::unique_ptr<ExtrusionPath> first = make_print_path(20.0, 25.0);
    first->remove_property<ExtrusionAttributes>();
    std::unique_ptr<ExtrusionPath> later = make_print_path(25.0, 30.0);
    later->set_role(ExtrusionRole::InternalInfill);
    std::unique_ptr<ExtrusionEntityCollection> root =
        std::make_unique<ExtrusionEntityCollection>(false, false);
    root->append(std::move(first));
    root->append(std::move(later));
    replace_target_root(print, std::unique_ptr<ExtrusionEntity>(std::move(root)));

    run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN});

    CHECK(target_unretract_role(print) == RAW_EXTRUSION_ROLE_UNRETRACT);
}

TEST_CASE("CreateRetraction ignores technical phase roles for Unretract",
          "[plugins][layer-extrusion-edit][retraction][role]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_print(print, model);
    std::unique_ptr<ExtrusionEntity> path =
        std::move(target_extrusion(print).root);
    path->remove_property<ExtrusionAttributes>();
    replace_target_root(print, std::move(path));

    run_layer_plugins(print, {
        TRANSITION_SCOPE_PLUGIN, ENTRY_STATE_PLUGIN, TRAVEL_PLUGIN,
        RETRACTION_PLUGIN});

    const OrderedExtrusionScope target(
        find_scope(entity_view(*target_extrusion(print).root)),
        scope_property_key());
    const slic3r_api::EPropertyAttributes *travel_attributes =
        target.travel().get(slic3r_api::EPropertyAttributes::key);
    REQUIRE(travel_attributes != nullptr);
    CHECK(travel_attributes->extrusion_role() == RAW_EXTRUSION_ROLE_TRAVEL);
    CHECK(target_unretract_role(print) == RAW_EXTRUSION_ROLE_UNRETRACT);
}

TEST_CASE("CreateRetraction falls back to Unretract without a content role",
          "[plugins][layer-extrusion-edit][retraction][role]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_print(print, model);
    target_extrusion(print).root->remove_property<ExtrusionAttributes>();

    run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN});

    CHECK(target_unretract_role(print) == RAW_EXTRUSION_ROLE_UNRETRACT);
}

TEST_CASE("Tool changes remain semantic when their retraction target is zero",
          "[plugins][layer-extrusion-edit][retraction][toolchange]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_print(print, model, "0", "0");
    PrintingLayerGroup &layer = print.mutable_printing_plan().groups.front()
        .layers.front();
    PrintingExtrusion moved = std::move(layer.tool_groups.front().extrusions.back());
    layer.tool_groups.front().extrusions.pop_back();
    layer.tool_groups.emplace_back();
    layer.tool_groups.back().extruder_id = 1;
    layer.tool_groups.back().extrusions.push_back(std::move(moved));

    run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN});

    slic3r_api::MutableExtrusionEntity source_root = find_scope(entity_view(
        *layer.tool_groups.front().extrusions.front().root));
    slic3r_api::MutableExtrusionEntity target_root = find_scope(entity_view(
        *layer.tool_groups.back().extrusions.front().root));
    OrderedExtrusionScope source(source_root, scope_property_key());
    OrderedExtrusionScope target(target_root, scope_property_key());
    CHECK(source.has_outgoing_toolchange());
    CHECK(target.has_incoming_toolchange());

    std::vector<std::string> source_events;
    std::vector<std::string> target_events;
    collect_transition_events(source.after().readonly(), source_events);
    collect_transition_events(target.before().readonly(), target_events);
    CHECK(source_events.empty());
    CHECK(target_events.empty());
}

TEST_CASE("Tool-change restart extra follows the selected extruder",
          "[plugins][layer-extrusion-edit][retraction][toolchange]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_print(print, model, "2", "3");
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.retract_restart_extra_toolchange.set(
        std::vector<double>{0.2, -0.3});

    PrintingLayerGroup &layer = print.mutable_printing_plan().groups.front()
        .layers.front();
    PrintingExtrusion moved = std::move(
        layer.tool_groups.front().extrusions.back());
    layer.tool_groups.front().extrusions.pop_back();
    layer.tool_groups.emplace_back();
    layer.tool_groups.back().extruder_id = 1;
    layer.tool_groups.back().extrusions.push_back(std::move(moved));

    run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN});

    OrderedExtrusionScope source(
        find_scope(entity_view(
            *layer.tool_groups.front().extrusions.front().root)),
        scope_property_key());
    OrderedExtrusionScope target(
        find_scope(entity_view(
            *layer.tool_groups.back().extrusions.front().root)),
        scope_property_key());
    const slic3r_api::EPropertyExtrusionAxis *retract_axis =
        source.after().get(slic3r_api::EPropertyExtrusionAxis::key);
    REQUIRE(retract_axis != nullptr);
    CHECK(retract_axis->value == Approx(3.0));
    CHECK(retract_axis->toolchange == 1);

    const slic3r_api::EPropertyExtrusionAxis *unretract_axis =
        target.before().get(slic3r_api::EPropertyExtrusionAxis::key);
    REQUIRE(unretract_axis != nullptr);
    CHECK(unretract_axis->restart_extra == Approx(-0.3));
    CHECK(unretract_axis->toolchange == 1);
}

TEST_CASE("Empty tool visits remain visible between printable scopes",
          "[plugins][layer-extrusion-edit][retraction][toolchange]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_print(print, model);

    PrintingLayerGroup &layer = print.mutable_printing_plan().groups.front()
        .layers.front();
    PrintingExtrusion moved = std::move(
        layer.tool_groups.front().extrusions.back());
    layer.tool_groups.front().extrusions.pop_back();
    layer.tool_groups.emplace_back();
    layer.tool_groups.back().extruder_id = 1;
    layer.tool_groups.emplace_back();
    layer.tool_groups.back().extruder_id = 0;
    layer.tool_groups.back().extrusions.push_back(std::move(moved));

    run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN});

    OrderedExtrusionScope source(
        find_scope(entity_view(
            *layer.tool_groups.front().extrusions.front().root)),
        scope_property_key());
    OrderedExtrusionScope target(
        find_scope(entity_view(
            *layer.tool_groups.back().extrusions.front().root)),
        scope_property_key());
    CHECK(source.has_outgoing_toolchange());
    CHECK(target.has_incoming_toolchange());

    std::vector<std::string> target_events;
    collect_transition_events(target.before().readonly(), target_events);
    CHECK(target_events == std::vector<std::string>{"unretract"});
}

TEST_CASE("CreateRetraction shares one decision across non-empty layers",
          "[plugins][layer-extrusion-edit][retraction][parallel]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_print(print, model);
    PrintingGroup &group = print.mutable_printing_plan().groups.front();
    PrintingExtrusion moved = std::move(
        group.layers.front().tool_groups.front().extrusions.back());
    group.layers.front().tool_groups.front().extrusions.pop_back();
    group.layers.emplace_back();
    group.layers.back().print_z = scale_i(0.4);
    group.layers.back().tool_groups.emplace_back();
    group.layers.back().tool_groups.back().extruder_id = 0;
    group.layers.back().tool_groups.back().extrusions.push_back(std::move(moved));

    run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN});

    slic3r_api::MutableExtrusionEntity source_root = find_scope(entity_view(
        *group.layers.front().tool_groups.front().extrusions.front().root));
    slic3r_api::MutableExtrusionEntity target_root = find_scope(entity_view(
        *group.layers.back().tool_groups.front().extrusions.front().root));
    OrderedExtrusionScope source(source_root, scope_property_key());
    OrderedExtrusionScope target(target_root, scope_property_key());
    std::vector<std::string> source_events;
    std::vector<std::string> target_events;
    collect_transition_events(source.after().readonly(), source_events);
    collect_transition_events(target.before().readonly(), target_events);
    CHECK(source_events == std::vector<std::string>{"retract"});
    CHECK(target_events == std::vector<std::string>{"unretract"});
}

TEST_CASE("Disable retraction overrides an enforced boundary",
          "[plugins][layer-extrusion-edit][retraction][modifier]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_print(print, model);
    PrintingToolGroup &tool = print.mutable_printing_plan().groups.front()
        .layers.front().tool_groups.front();
    tool.extrusions.front().root
        ->get_or_add_property<ExtrusionPropertyModifier>()
        .set_enforce_retraction();
    tool.extrusions.back().root
        ->get_or_add_property<ExtrusionPropertyModifier>()
        .set_disable_retraction();

    run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN});

    OrderedExtrusionScope source(
        find_scope(entity_view(*tool.extrusions.front().root)),
        scope_property_key());
    OrderedExtrusionScope target(
        find_scope(entity_view(*tool.extrusions.back().root)),
        scope_property_key());
    std::vector<std::string> source_events;
    std::vector<std::string> target_events;
    collect_transition_events(source.after().readonly(), source_events);
    collect_transition_events(target.before().readonly(), target_events);
    CHECK(source_events.empty());
    CHECK(target_events.empty());
}

TEST_CASE("Direct gap threshold controls ordinary retraction",
          "[plugins][layer-extrusion-edit][retraction][threshold]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_print(print, model);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.retract_before_travel.set(std::vector<double>{20.0});

    run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN});

    PrintingToolGroup &tool = print.mutable_printing_plan().groups.front()
        .layers.front().tool_groups.front();
    OrderedExtrusionScope source(
        find_scope(entity_view(*tool.extrusions.front().root)),
        scope_property_key());
    OrderedExtrusionScope target(
        find_scope(entity_view(*tool.extrusions.back().root)),
        scope_property_key());
    std::vector<std::string> source_events;
    std::vector<std::string> target_events;
    collect_transition_events(source.after().readonly(), source_events);
    collect_transition_events(target.before().readonly(), target_events);
    CHECK(source_events.empty());
    CHECK(target_events.empty());
}

TEST_CASE("Layer-change retraction reserves phases for contiguous geometry",
          "[plugins][layer-extrusion-edit][retraction][layer-change]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_print(print, model);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.retract_layer_change.set(std::vector<unsigned char>{1});

    PrintingGroup &group = print.mutable_printing_plan().groups.front();
    PrintingExtrusion moved = std::move(
        group.layers.front().tool_groups.front().extrusions.back());
    group.layers.front().tool_groups.front().extrusions.pop_back();
    moved.root = make_print_path(10.0, 20.0)->clone_canonical();
    group.layers.emplace_back();
    group.layers.back().print_z = scale_i(0.2);
    group.layers.back().tool_groups.emplace_back();
    group.layers.back().tool_groups.back().extruder_id = 0;
    group.layers.back().tool_groups.back().extrusions.push_back(std::move(moved));

    run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN});

    OrderedExtrusionScope source(
        find_scope(entity_view(
            *group.layers.front().tool_groups.front().extrusions.front().root)),
        scope_property_key());
    OrderedExtrusionScope target(
        find_scope(entity_view(
            *group.layers.back().tool_groups.front().extrusions.front().root)),
        scope_property_key());
    REQUIRE(source.has_outgoing_transition());
    REQUIRE(target.has_incoming_transition());
    std::vector<std::string> source_events;
    std::vector<std::string> target_events;
    collect_transition_events(source.after().readonly(), source_events);
    collect_transition_events(target.before().readonly(), target_events);
    CHECK(source_events == std::vector<std::string>{"retract"});
    CHECK(target_events == std::vector<std::string>{"unretract"});
}

TEST_CASE("Terminal retraction belongs to PrintingPlan after events",
          "[plugins][layer-extrusion-edit][retraction][terminal]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_print(print, model);
    run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN});
    run_plan_plugins(print, {TERMINAL_RETRACTION_PLUGIN});

    const slic3r_api::PrintingPlan plan(
        reinterpret_cast<printing_plan_handle *>(
            &print.mutable_printing_plan()));
    REQUIRE(plan.events().has_after());
    std::vector<std::string> events;
    collect_transition_events(plan.events().after(), events);
    CHECK(events == std::vector<std::string>{"retract"});
}

TEST_CASE("Configured tool and filament scripts keep separate ordered owners",
          "[plugins][layer-extrusion-edit][retraction][settings-scripts]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_print(print, model);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.nozzle_diameter.set(std::vector<double>{0.4, 0.4});
    config.option<ConfigOptionString>("toolchange_gcode")->value = "; toolchange";
    config.option<ConfigOptionStrings>("start_filament_gcode")->set(
        std::vector<std::string>{"", "; start filament"});

    PrintingLayerGroup &layer = print.mutable_printing_plan().groups.front()
        .layers.front();
    PrintingExtrusion moved = std::move(layer.tool_groups.front().extrusions.back());
    layer.tool_groups.front().extrusions.pop_back();
    layer.tool_groups.emplace_back();
    layer.tool_groups.back().extruder_id = 1;
    layer.tool_groups.back().extrusions.push_back(std::move(moved));

    run_layer_plugins(print, {
        TOOLCHANGE_PLUGIN, TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN});
    run_plan_plugins(print, {SETTINGS_SCRIPTS_PLUGIN});

    slic3r_api::MutableExtrusionEntity target_root = find_scope(entity_view(
        *layer.tool_groups.back().extrusions.front().root));
    OrderedExtrusionScope target(target_root, scope_property_key());
    std::vector<std::string> tool_events;
    const slic3r_api::PrintingToolGroup target_tool(
        reinterpret_cast<printing_tool_group_handle *>(
            &layer.tool_groups.back()));
    collect_transition_events(target_tool.events().before(), tool_events);
    CHECK(tool_events == std::vector<std::string>{"toolchange_script"});

    std::vector<std::string> events;
    collect_transition_events(target.before().readonly(), events);
    CHECK(events == std::vector<std::string>{
        "start_filament_script", "unretract"});
}

} // namespace
