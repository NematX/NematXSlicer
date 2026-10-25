#include <catch2/catch.hpp>

#include <algorithm>
#include <string>
#include <vector>

#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"

/*
Plugin-defined option lists
===========================

These tests cover plugin-ID enum pruning, deferred option-definition patches,
exclusive provider selectors, and STRING_EQUALS GUI conditions. Infill pattern
lists are the reference user of the plugin-defined enum machinery.

Test option definitions are registered by the shared runtime before it applies
deferred patches, because PrintConfigDef can only be extended before finalization.
*/

using namespace Slic3r;

namespace {

const ConfigOptionDef *option_def(const char *key)
{
    return PrintConfigDef::instance().get(key);
}

std::vector<std::string> option_values(const char *key)
{
    const ConfigOptionDef *def = option_def(key);
    if (def == nullptr || def->enum_def.get() == nullptr)
        return {};
    return def->enum_def->values();
}

bool has_value(const std::vector<std::string> &values, const std::string &value)
{
    return std::find(values.begin(), values.end(), value) != values.end();
}

} // namespace

TEST_CASE("Infill pattern lists are plugin-defined enums", "[plugins][infill][options]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    const Plugin *support_base = orchestrator.get_plugin("supportbase");
    const Plugin *support_cubic = orchestrator.get_plugin("supportcubic");
    const Plugin *lightning = orchestrator.get_plugin("lightning");
    REQUIRE(support_base != nullptr);
    REQUIRE(support_cubic != nullptr);
    REQUIRE(lightning != nullptr);
    CHECK(lightning->get_step() == INFILL_PATTERN);
    CHECK(lightning->get_priority() == 23);
    CHECK(support_base->get_step() == INFILL_PATTERN);
    CHECK(support_cubic->get_step() == INFILL_PATTERN);

    for (const char *key : { "fill_pattern", "top_fill_pattern", "bottom_fill_pattern",
                             "solid_fill_pattern", "bridge_fill_pattern" }) {
        const ConfigOptionDef *def = option_def(key);
        INFO(key);
        REQUIRE(def != nullptr);
        CHECK(def->contains_plugin_id);
        CHECK(def->type == coEnum);
    }

    const std::vector<std::string> solid = option_values("solid_fill_pattern");
    CHECK(has_value(solid, "ensuring"));
    CHECK(has_value(solid, "smooth"));
    CHECK_FALSE(has_value(solid, "monotonicgapfill"));

    const std::vector<std::string> sparse = option_values("fill_pattern");
    CHECK(has_value(sparse, "stars"));
    CHECK(has_value(sparse, "gyroid"));
    CHECK(has_value(sparse, "supportcubic"));
    CHECK(std::count(sparse.begin(), sparse.end(), "lightning") == 1);
    for (const char *key : {"top_fill_pattern", "bottom_fill_pattern", "solid_fill_pattern", "bridge_fill_pattern"})
        CHECK_FALSE(has_value(option_values(key), "lightning"));
    CHECK_FALSE(has_value(sparse, "ensuring"));

    // Patterns provided by standalone plugins still join the lists they belong
    // to. The generator no longer declares them in its base lists.
    CHECK(has_value(sparse, "honeycomb"));
    CHECK(has_value(sparse, "3dhoneycomb"));
    CHECK(has_value(sparse, "hilbertcurve"));
    CHECK(has_value(sparse, "archimedeanchords"));
    CHECK(has_value(sparse, "octagramspiral"));
    CHECK(has_value(option_values("top_fill_pattern"), "hilbertcurve"));
    CHECK(has_value(option_values("bottom_fill_pattern"), "archimedeanchords"));
    CHECK(has_value(option_values("solid_fill_pattern"), "octagramspiral"));

    // Each contribution is applied once even when several plugins contribute
    // to the same list.
    CHECK(std::count(sparse.begin(), sparse.end(), "honeycomb") == 1);
    CHECK(std::count(sparse.begin(), sparse.end(), "gyroid") == 1);

    // Each slot keeps its own subset instead of sharing one merged list.
    CHECK(has_value(option_values("top_fill_pattern"), "sawtooth"));
    CHECK_FALSE(has_value(option_values("bottom_fill_pattern"), "sawtooth"));
    CHECK_FALSE(has_value(option_values("solid_fill_pattern"), "grid"));
    CHECK_FALSE(has_value(sparse, "supportbase"));
    CHECK_FALSE(has_value(option_values("top_fill_pattern"), "supportbase"));
    CHECK_FALSE(has_value(option_values("bottom_fill_pattern"), "supportbase"));
    CHECK_FALSE(has_value(option_values("solid_fill_pattern"), "supportbase"));
    CHECK_FALSE(has_value(option_values("bridge_fill_pattern"), "supportbase"));
    CHECK_FALSE(has_value(option_values("top_fill_pattern"), "supportcubic"));
    CHECK_FALSE(has_value(option_values("bottom_fill_pattern"), "supportcubic"));
    CHECK_FALSE(has_value(option_values("solid_fill_pattern"), "supportcubic"));
    CHECK_FALSE(has_value(option_values("bridge_fill_pattern"), "supportcubic"));
    CHECK(std::count(sparse.begin(), sparse.end(), "supportcubic") == 1);

    const std::vector<std::string> bridge = option_values("bridge_fill_pattern");
    REQUIRE(bridge.size() == 2);
    CHECK(has_value(bridge, "rectilinear"));
    CHECK(has_value(bridge, "monotonic"));

    // Defaults are owned by the plugin definition, not reset to a single value.
    REQUIRE(option_def("solid_fill_pattern")->default_value);
    CHECK(option_def("solid_fill_pattern")->default_value->serialize() == "ensuring");
    REQUIRE(option_def("fill_pattern")->default_value);
    CHECK(option_def("fill_pattern")->default_value->serialize() == "stars");
    REQUIRE(option_def("top_fill_pattern")->default_value);
    CHECK(option_def("top_fill_pattern")->default_value->serialize() == "monotonic");
}

TEST_CASE("Plugin-id option lists prune inactive plugins", "[plugins][infill][options]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    // The fixture starts with one active plugin id and one unknown id.
    CHECK(option_values("test.infill.prune_list") == std::vector<std::string>{ "rectilinear" });

    // A list left empty by pruning is reported as a non-fatal error.
    CHECK(option_values("test.infill.empty_list").empty());
    CHECK(Slic3r::Test::Plugins::plugin_test_empty_option_list_error_reported());
}

/*
Read the finalized selector and evaluate its real GUI registry with the valid provider ID. The local
configuration owns its values and rule applications are returned by value; evaluation refreshes the
registry's explanation cache but does not alter provider activation, while failed Catch2 checks report
missing options, a wrong menu/default, or a disabled setting. Identifier-versus-label matching is
covered by test_fff_print_gui_rules.cpp:804-829, where perimeter-generator labels differ from IDs.
*/
TEST_CASE("Exclusive selectors prune failed providers and match GUI rules by ID",
          "[plugins][config][gui-rules][options]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    const char *const failed_id = "slic3r.test.exclusive_pruning.failed";
    const char *const valid_id = "slic3r.test.exclusive_pruning.valid";
    const char *const valid_setting = "test.s2.valid_provider_setting";
    const std::string group_id = "test.exclusive.selector_pruning";

    const Plugin *failed = orchestrator.get_plugin(failed_id);
    const Plugin *valid = orchestrator.get_plugin(valid_id);
    REQUIRE(failed != nullptr);
    REQUIRE(valid != nullptr);
    CHECK(failed->get_priority() < valid->get_priority());
    CHECK_FALSE(orchestrator.is_plugin_active(failed));
    CHECK_FALSE(orchestrator.is_plugin_active(valid));

    const std::string selector_key = Steps::plugin_exclusive_group_option_key(
        orchestrator, STEP_POST_SLICING, group_id);
    const ConfigOptionDef *selector = option_def(selector_key.c_str());
    REQUIRE(selector != nullptr);
    CHECK(selector->contains_plugin_id);
    REQUIRE(selector->enum_def.get() != nullptr);
    CHECK(selector->enum_def->values() == std::vector<std::string>{ valid_id });
    REQUIRE(selector->default_value);
    CHECK(selector->default_value->serialize() == valid_id);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    REQUIRE(config.option(selector_key) != nullptr);
    REQUIRE(config.option(valid_setting) != nullptr);
    config.set_deserialize_strict(selector_key, valid_id);
    const std::vector<Orchestrator::GuiRuleApplication> applications =
        orchestrator.gui_rules().evaluate(
            config, RAW_PRESET_TYPE_FFF_PRINT, -1, valid_id);
    const std::vector<Orchestrator::GuiRuleApplication>::const_iterator valid_rule =
        std::find_if(applications.begin(), applications.end(),
                     [valid_setting](const Orchestrator::GuiRuleApplication &application) {
                         return application.target_key == valid_setting;
                     });
    REQUIRE(valid_rule != applications.end());
    CHECK(valid_rule->enabled);
}

TEST_CASE("REPLICATE_ENUM copies a resolved list and breaks cycles softly", "[plugins][infill][options]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const ConfigOptionDef *target = option_def("test.replicate.target");
    REQUIRE(target != nullptr);
    REQUIRE(target->enum_def.get() != nullptr);
    CHECK(target->enum_def->values() == std::vector<std::string>{ "rectilinear" });

    // A contribution added before the target list exists is kept, and gives the
    // same result as the same contribution added after the list exists.
    const std::vector<std::string> expected = { "rectilinear", "gyroid" };
    const ConfigOptionDef *contrib_target = option_def("test.replicate.contrib_target");
    const ConfigOptionDef *order_target = option_def("test.replicate.order_target");
    REQUIRE(contrib_target != nullptr);
    REQUIRE(order_target != nullptr);
    REQUIRE(contrib_target->enum_def.get() != nullptr);
    REQUIRE(order_target->enum_def.get() != nullptr);
    CHECK(contrib_target->enum_def->values() == expected);
    CHECK(order_target->enum_def->values() == expected);

    // A->B and B->A: the cyclic edge is ignored, so each option still resolves
    // once and deterministically.
    const ConfigOptionDef *cycle_a = option_def("test.cycle.a");
    const ConfigOptionDef *cycle_b = option_def("test.cycle.b");
    REQUIRE(cycle_a != nullptr);
    REQUIRE(cycle_b != nullptr);
    REQUIRE(cycle_a->enum_def.get() != nullptr);
    REQUIRE(cycle_b->enum_def.get() != nullptr);
    CHECK_FALSE(cycle_a->enum_def->values().empty());
    CHECK(cycle_a->enum_def->values() == cycle_b->enum_def->values());
}

TEST_CASE("STRING_EQUALS compares serialized option values", "[plugins][gui-rules]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    const char *const owner = "test.string_equals";

    raw_gui_rule_clause clause = raw_gui_rule_clause_init();
    clause.condition = RAW_GUI_RULE_CONDITION_STRING_EQUALS;
    clause.condition_key = "solid_fill_pattern";
    clause.condition_string_value = "smooth";

    raw_gui_rule rule = raw_gui_rule_init();
    rule.target_key = "solid_fill_pattern";
    rule.scope = RAW_PRESET_TYPE_FFF_PRINT;
    rule.clauses = &clause;
    rule.clause_count = 1;
    CHECK(orchestrator.gui_rules().add(&rule, owner) == Orchestrator::GuiRuleRegistration::Added);

    const auto target_enabled = [&orchestrator, owner](const DynamicPrintConfig &config) {
        for (const Orchestrator::GuiRuleApplication &application :
             orchestrator.gui_rules().evaluate(config, RAW_PRESET_TYPE_FFF_PRINT, -1, owner))
            if (application.target_key == "solid_fill_pattern")
                return application.enabled;
        return false;
    };

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("solid_fill_pattern", "smooth");
    CHECK(target_enabled(config));

    config.set_deserialize_strict("solid_fill_pattern", "rectilinear");
    CHECK_FALSE(target_enabled(config));
}

TEST_CASE("STRING_EQUALS resolves the vector item selected by condition_index", "[plugins][gui-rules]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    const char *const indexed_owner = "test.string_equals_vector.indexed";
    const char *const any_owner = "test.string_equals_vector.any";
    const char *const current_owner = "test.string_equals_vector.current";
    const char *const out_of_range_owner = "test.string_equals_vector.out_of_range";

    raw_gui_rule_clause clause = raw_gui_rule_clause_init();
    clause.condition = RAW_GUI_RULE_CONDITION_STRING_EQUALS;
    clause.condition_key = "filament_type";
    clause.condition_string_value = "PETG";

    raw_gui_rule rule = raw_gui_rule_init();
    rule.scope = RAW_PRESET_TYPE_FFF_PRINT;
    rule.clauses = &clause;
    rule.clause_count = 1;

    // An explicit index reads exactly that vector item.
    clause.condition_index = 1;
    rule.target_key = "fill_aligned_z";
    CHECK(orchestrator.gui_rules().add(&rule, indexed_owner) == Orchestrator::GuiRuleRegistration::Added);

    // An explicit index outside the vector never matches.
    clause.condition_index = 7;
    rule.target_key = "fill_angle";
    CHECK(orchestrator.gui_rules().add(&rule, out_of_range_owner) == Orchestrator::GuiRuleRegistration::Added);

    // INDEX_ALL without a current item is true when any item matches.
    clause.condition_index = RAW_GUI_RULE_INDEX_ALL;
    rule.target_key = "infill_every_layers";
    CHECK(orchestrator.gui_rules().add(&rule, any_owner) == Orchestrator::GuiRuleRegistration::Added);

    // INDEX_CURRENT follows the extruder item currently processed.
    clause.condition_index = RAW_GUI_RULE_INDEX_CURRENT;
    rule.target_key = "filament_type";
    rule.target_index = RAW_GUI_RULE_INDEX_CURRENT;
    CHECK(orchestrator.gui_rules().add(&rule, current_owner) == Orchestrator::GuiRuleRegistration::Added);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    REQUIRE(config.option("filament_type")->is_vector());
    REQUIRE(static_cast<const ConfigOptionVectorBase *>(config.option("filament_type"))->is_extruder_size());

    const auto configured_types = [&config]() {
        return config.option<ConfigOptionStrings>("filament_type")->get_values();
    };
    const auto set_types = [&config](const char *serialized) {
        config.set_deserialize_strict("filament_type", serialized);
    };
    const auto enabled_for = [&orchestrator, &config](const char *owner, const char *target, int current_index) {
        for (const Orchestrator::GuiRuleApplication &application :
             orchestrator.gui_rules().evaluate(config, RAW_PRESET_TYPE_FFF_PRINT, current_index, owner))
            if (application.target_key == target)
                return application.enabled;
        return false;
    };

    // Two different items: the element read depends on the resolved index.
    set_types("PLA;PETG");
    REQUIRE(configured_types() == std::vector<std::string>{ "PLA", "PETG" });
    CHECK(enabled_for(indexed_owner, "fill_aligned_z", -1));
    CHECK_FALSE(enabled_for(out_of_range_owner, "fill_angle", -1));

    set_types("PETG;PLA");
    CHECK_FALSE(enabled_for(indexed_owner, "fill_aligned_z", -1));

    set_types("PLA;PLA");
    CHECK_FALSE(enabled_for(indexed_owner, "fill_aligned_z", -1));

    // "Any item" read, in both orders.
    set_types("PLA;PETG");
    CHECK(enabled_for(any_owner, "infill_every_layers", -1));
    set_types("PETG;PLA");
    CHECK(enabled_for(any_owner, "infill_every_layers", -1));
    set_types("PLA;PLA");
    CHECK_FALSE(enabled_for(any_owner, "infill_every_layers", -1));

    // Current-item read: index 0 is PLA, index 1 is PETG.
    set_types("PLA;PETG");
    CHECK_FALSE(enabled_for(current_owner, "filament_type", 0));
    CHECK(enabled_for(current_owner, "filament_type", 1));
}
