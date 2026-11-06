/*
 * Headless tests for entity-setting scopes, atomic overrides, and layer ranges.
 *
 * These tests drive the scene-independent module directly: option definitions
 * come from the initialized engine, while each write targets a private model
 * configuration. The HTTP suite separately verifies session publication.
 */

#include <catch2/catch.hpp>

#include "entity_settings.hpp"
#include "rest_models_test_helpers.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

using namespace RestServerTest;
using namespace Slic3r;
using namespace Slic3r::Rest;

namespace {

DynamicPrintConfig default_session_config()
{
    DynamicPrintConfig configuration;
    configuration.apply(FullPrintConfig::defaults());
    return configuration;
}

} // namespace

TEST_CASE("Entity setting scopes follow the GUI lists and plugin containers", "[entity_settings]")
{
    ensure_engine_initialized();
    const std::set<std::string> &objects = scope_keys(SettingScope::object);
    const std::set<std::string> &parts = scope_keys(SettingScope::part);
    const std::set<std::string> &ranges = scope_keys(SettingScope::layer_range);
    CHECK(objects.find("layer_height") != objects.end());
    CHECK(ranges.find("layer_height") != ranges.end());
    CHECK(parts.find("layer_height") == parts.end());
    CHECK(objects.find("brim_width") != objects.end());
    CHECK(ranges.find("brim_width") == ranges.end());
    CHECK(setting_scopes("extruder") == std::vector<std::string>{"object", "part", "layer_range"});

    const PrintConfigDef &definitions = PrintConfigDef::instance();
    std::string support_setting;
    for (const std::pair<const t_config_option_key, ConfigOptionDef> &entry : definitions.options) {
        if (entry.second.category == OptionCategory::support) {
            support_setting = entry.first;
            break;
        }
    }
    REQUIRE_FALSE(support_setting.empty());
    CHECK_FALSE(setting_in_scope(SettingScope::part, support_setting));
    CHECK_FALSE(setting_in_scope(SettingScope::layer_range, support_setting));

    std::string plugin_region_setting;
    std::string plugin_region_candidates;
    for (const std::pair<const t_config_option_key, ConfigOptionDef> &entry : definitions.options) {
        const ConfigOptionDef &definition = entry.second;
        const bool region_plugin = definition.container_type == ConfigOptionContainerType::Layer ||
            definition.container_type == ConfigOptionContainerType::Region;
        if (region_plugin) {
            if (!plugin_region_candidates.empty())
                plugin_region_candidates += ", ";
            plugin_region_candidates += entry.first + "(" + std::to_string(definition.option_preset_type) + ")";
            if (setting_in_scope(SettingScope::part, entry.first) &&
                setting_in_scope(SettingScope::layer_range, entry.first)) {
                plugin_region_setting = entry.first;
                break;
            }
        }
    }
    INFO("Region plugin candidates: " << plugin_region_candidates);
    REQUIRE_FALSE(plugin_region_setting.empty());
    CHECK(setting_in_scope(SettingScope::part, plugin_region_setting));
    CHECK(setting_in_scope(SettingScope::layer_range, plugin_region_setting));
}

TEST_CASE("Entity settings write atomically and remove overrides", "[entity_settings]")
{
    ensure_engine_initialized();
    Model model;
    ModelObject *object = model.add_object("object", "object.stl", make_cube(10, 10, 10));
    object->add_instance();
    SettingsTarget target{SettingScope::object, &object->config, object, std::nullopt};
    const DynamicPrintConfig session = default_session_config();

    const EntitySettingsChange first = write_entity_settings(target, {{"layer_height", "0.3"}}, session);
    CHECK(first.changed);
    CHECK(read_entity_setting(target, "layer_height", session).at("value") == Approx(0.3));
    const DynamicPrintConfig unchanged = object->config.get();
    CHECK_FALSE(write_entity_settings(target, {{"layer_height", "0.3"}}, session).changed);
    CHECK(object->config.get() == unchanged);

    const EntitySettingsChange removed = write_entity_settings(target, {{"layer_height", std::nullopt}}, session);
    CHECK(removed.changed);
    CHECK_FALSE(object->config.has("layer_height"));
    CHECK_FALSE(write_entity_settings(target, {{"layer_height", std::nullopt}}, session).changed);

    const std::vector<std::pair<std::string, std::optional<std::string>>> invalid_updates{
        {"layer_height", std::string("0.25")}, {"not_a_setting", std::string("1")}
    };
    CHECK_THROWS_AS(write_entity_settings(target, invalid_updates, session), SceneEditError);
    CHECK_FALSE(object->config.has("layer_height"));

    CHECK(write_entity_settings(target, {{"extruder", "1"}}, session).changed);
    CHECK_THROWS_AS(write_entity_settings(target, {{"extruder", "2"}}, session), SceneEditError);
    CHECK(object->config.opt_int("extruder") == 1);

    ModelConfig range_config;
    range_config.set_key_value("layer_height", new ConfigOptionFloat(0.2));
    range_config.set_key_value("extruder", new ConfigOptionInt(0));
    const t_layer_height_range bounds{1., 3.};
    object->layer_config_ranges.emplace(bounds, std::move(range_config));
    SettingsTarget range_target{SettingScope::layer_range, &object->layer_config_ranges.at(bounds), object, bounds};
    CHECK_THROWS_AS(write_entity_settings(range_target, {{"layer_height", std::nullopt}}, session), SceneEditError);
    CHECK(object->layer_config_ranges.at(bounds).has("layer_height"));
}

TEST_CASE("Layer ranges validate bounds overlaps and nozzle height limits", "[entity_settings]")
{
    ensure_engine_initialized();
    const DynamicPrintConfig session = default_session_config();
    Model model;
    ModelObject *object = model.add_object("object", "object.stl", make_cube(10, 10, 10));
    object->add_instance();

    const EntitySettingsChange first = add_layer_range(*object, 0., 2., 0.3, 0, session);
    REQUIRE(first.bindings.size() == 1);
    CHECK(first.bindings.front().kind == EntityKind::layer_range);
    CHECK(first.bindings.front().engine_id == static_cast<std::uint64_t>(object->id().id));
    CHECK(first.bindings.front().range == t_layer_height_range{0., 2.});
    CHECK(add_layer_range(*object, 2., 4., 0.2, 0, session).changed);

    CHECK_THROWS_AS(add_layer_range(*object, 1.5, 3., 0.2, 0, session), SceneEditError);
    CHECK_THROWS_AS(add_layer_range(*object, 5., 4., 0.2, 0, session), SceneEditError);
    CHECK_THROWS_AS(add_layer_range(*object, 5., std::numeric_limits<double>::infinity(), 0.2, 0, session), SceneEditError);
    CHECK_THROWS_AS(add_layer_range(*object, 5., 8., 0.35, 0, session), SceneEditError);

    const t_layer_config_ranges ranges = object->layer_config_ranges;
    CHECK_THROWS_AS(validate_layer_range_bounds(ranges, {1., 3.}, std::nullopt), SceneEditError);
    CHECK_THROWS_AS(validate_layer_range_bounds(ranges, {3., 3.}, std::nullopt), SceneEditError);
    CHECK_THROWS_AS(validate_layer_range_bounds(ranges,
        {0., std::numeric_limits<double>::quiet_NaN()}, std::nullopt), SceneEditError);
    CHECK_NOTHROW(validate_layer_range_bounds(ranges, {4., 5.}, std::nullopt));
}
