///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Tests for plugin-owned temporary configurations.

The suite exercises the C ABI as a plugin sees it, then uses the C++ wrappers
to verify buffer ownership. The core SCFG codec is covered separately
by libslic3r tests; this suite verifies the ABI buffer and merge semantics.
Invalidation checks compare each option's earliest consuming step with the
checkpoint used to replay its affected pipeline suffix.
Setting-support checks compare definition defaults with config values and read
only provider metadata; unregistered copies model providers with no used keys.
*/

#include <catch2/catch.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

// PrintConfig.hpp hash macros call boost::hash_combine without its definition;
// Release builds of this file need it to instantiate the template.
#include <boost/container_hash/hash.hpp>

#include "plugin_test_helpers.hpp"
#include "gcode_test_helpers.hpp"
#include "test_data.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintRegion.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"

#include "libslic3r/Api/host/ApiHostUtils.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/slic3r_config.h"
#include "libslic3r/Api/plugin/cpp/ConfigDefinitionViews.hpp"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/OrchestratorViews.hpp"
#include "libslic3r/Config/ConfigDef.hpp"
#include "libslic3r/Config/ConfigOption.hpp"
#include "libslic3r/Config/PrintConfig.hpp"

namespace {

// This unregistered metadata copy models a provider that declares no setting
// support; the borrowed ABI callbacks are never executed by these checks.
class PluginWithoutSettingSupport final : public Slic3r::Plugin
{
public:
    // Copy a borrowed provider's metadata and clear its used-key declarations;
    // retained ABI pointers remain provider-owned, with allocation failures propagated.
    explicit PluginWithoutSettingSupport(const Slic3r::Plugin &provider) : Plugin(provider)
    {
        m_used_config_keys.clear();
    }
};

// This small static store exercises the API rejection path without depending
// on the global print-config caches initialized by the larger slicing fixture.
class StaticTestConfig final : public Slic3r::StaticConfig
{
public:
    const Slic3r::ConfigDef *def() const override { return nullptr; }

    const Slic3r::ConfigOption *optptr(const Slic3r::t_config_option_key &key) const override
    {
        return key == "value" ? &m_value : nullptr;
    }

    Slic3r::ConfigOption *optptr(const Slic3r::t_config_option_key &key, bool /* create */) override
    {
        return key == "value" ? &m_value : nullptr;
    }

    Slic3r::t_config_option_keys keys() const override { return {"value"}; }

private:
    Slic3r::ConfigOptionInt m_value{17};
};

static std::string serialize_all(const config_handle *config);
template<class OptionType>
static void check_vector_item_round_trip(OptionType &option);
template<class OptionType>
static void check_vector_item_rejections(
    OptionType &option, std::initializer_list<const char *> invalid_values);

// Use the public two-call protocol so tests also validate byte counts and null
// termination rather than reaching into the host implementation.
static std::string serialize_all(const config_handle *config)
{
    const uint32_t size = config_serialize_all(config, nullptr, 0);
    REQUIRE(size > 0);
    std::string serialized(size + 1, '\0');
    REQUIRE(config_serialize_all(config, &serialized[0], size + 1) == size);
    REQUIRE(serialized[size] == '\0');
    serialized.resize(size);
    return serialized;
}

// Exercise the public element codec in both directions and verify that value,
// per-item state and aggregate state remain coherent after replacement.
template<class OptionType>
static void check_vector_item_round_trip(OptionType &option)
{
    config_option_handle *handle = reinterpret_cast<config_option_handle *>(&option);
    config_option_vector_handle *vector = config_option_vector_cast_mutable(handle);
    REQUIRE(vector != nullptr);
    REQUIRE(option.size() >= 2);

    const std::string expected = option.serialize_at(0);
    std::vector<char> serialized(expected.size() + 1, '\0');
    CHECK(config_option_vector_serialize_at(
        vector, 0, serialized.data(), uint32_t(serialized.size())) == expected.size());
    CHECK(std::string(serialized.data()) == expected);
    REQUIRE(config_option_vector_deserialize_at(vector, 1, serialized.data()) == 1);
    CHECK(option.serialize_at(1) == expected);
    CHECK(option.is_enabled(1) == option.is_enabled(0));
    CHECK(option.is_enabled(-1) == option.has_enabled());
}

// Invalid element text must not leak any partial work from the temporary
// parser into the destination vector.
template<class OptionType>
static void check_vector_item_rejections(
    OptionType &option, std::initializer_list<const char *> invalid_values)
{
    config_option_handle *handle = reinterpret_cast<config_option_handle *>(&option);
    config_option_vector_handle *vector = config_option_vector_cast_mutable(handle);
    REQUIRE(vector != nullptr);
    REQUIRE(option.size() >= 2);

    const std::string before = option.serialize();
    const size_t before_size = option.size();
    std::vector<bool> before_enabled(before_size, false);
    for (size_t idx = 0; idx < before_size; ++idx)
        before_enabled[idx] = option.is_enabled(int32_t(idx));
    const bool before_global_enabled = option.is_enabled(-1);

    for (const char *invalid_value : invalid_values) {
        CAPTURE(invalid_value);
        CHECK(config_option_vector_deserialize_at(vector, 1, invalid_value) == 0);
        CHECK(option.serialize() == before);
        REQUIRE(option.size() == before_size);
        for (size_t idx = 0; idx < before_size; ++idx)
            CHECK(option.is_enabled(int32_t(idx)) == before_enabled[idx]);
        CHECK(option.is_enabled(-1) == before_global_enabled);
    }
}

} // namespace

TEST_CASE("Plugin storage owns temporary configs", "[plugins][config]")
{
    Slic3r::PluginStorage storage;
    storage_handle *storage_api = reinterpret_cast<storage_handle *>(&storage);
    REQUIRE(storage_size(storage_api) == 0);

    config_handle *first = storage_new_config(storage_api);
    REQUIRE(first != nullptr);
    CHECK(is_local_storage(storage_api, first) == 1);
    CHECK(storage_size(storage_api) == 1);

    // The registry keeps the first ConfigBase subobject at the same address
    // while later allocations grow it.
    std::vector<config_handle *> configs;
    for (size_t idx = 0; idx < 32; ++idx)
        configs.push_back(storage_new_config(storage_api));
    CHECK(config_get_or_add_mutable(first, "stable", SLIC3R_CONFIG_OPTION_INT) != nullptr);
    CHECK(storage_size(storage_api) == 33);

    CHECK(storage_free(storage_api, first) == 1);
    CHECK(storage_size(storage_api) == 32);
    storage_clear(storage_api);
    CHECK(storage_size(storage_api) == 0);
}

TEST_CASE("Temporary configs create options and clear only dynamic configs", "[plugins][config]")
{
    Slic3r::PluginStorage storage;
    storage_handle *storage_api = reinterpret_cast<storage_handle *>(&storage);
    config_handle *config = storage_new_config(storage_api);
    REQUIRE(config != nullptr);

    config_option_handle *integer =
        config_get_or_add_mutable(config, "value", SLIC3R_CONFIG_OPTION_INT);
    REQUIRE(integer != nullptr);
    CHECK(config_get_or_add_mutable(config, "value", SLIC3R_CONFIG_OPTION_INT) == integer);
    CHECK(config_get_or_add_mutable(config, "value", SLIC3R_CONFIG_OPTION_STRING) == nullptr);
    CHECK(config_get_or_add_mutable(config, "", SLIC3R_CONFIG_OPTION_INT) == nullptr);
    CHECK(config_get_or_add_mutable(config, "none", SLIC3R_CONFIG_OPTION_NONE) == nullptr);
    CHECK(config_get_or_add_mutable(config, "unknown", static_cast<config_option_type>(999)) == nullptr);

    config_option_set_int(integer, 91, 0);
    CHECK(config_option_get_int(config_get(config, "value"), 0) == 91);
    CHECK(config_clear(config) == 1);
    CHECK(config_keys(config).size == 0);
    CHECK(config_clear(config) == 1);

    StaticTestConfig static_config;
    config_handle *static_handle = Slic3r::ApiHost::to_config_handle(&static_config);
    const size_t static_key_count = static_config.keys().size();
    CHECK(config_clear(static_handle) == 0);
    CHECK(config_get_or_add_mutable(static_handle, "temporary", SLIC3R_CONFIG_OPTION_INT) == nullptr);
    CHECK(static_config.keys().size() == static_key_count);
    CHECK(static_config.option<Slic3r::ConfigOptionInt>("value")->value == 17);
}

TEST_CASE("Graph config values expose copied unscaled points and their interpolation type",
          "[plugins][config][graph]")
{
    Slic3r::PluginStorage storage;
    storage_handle *storage_api = reinterpret_cast<storage_handle *>(&storage);
    config_handle *config = storage_new_config(storage_api);
    REQUIRE(config != nullptr);

    config_option_handle *option =
        config_get_or_add_mutable(config, "graph", SLIC3R_CONFIG_OPTION_GRAPH);
    REQUIRE(option != nullptr);
    REQUIRE(config_option_deserialize(option, "1:3:2:-5.5x1.25:2.5x3.75:8x9.5:12x14", 0) == 1);

    const graph_data_handle *graph = config_option_get_graph(option, 0);
    REQUIRE(graph != nullptr);
    CHECK(graph_data_size(graph) == 2);
    CHECK(graph_data_type(graph) == RAW_GRAPH_TYPE_SPLINE);

    c_vec2d point{-1.0, -1.0};
    REQUIRE(graph_data_get_point(graph, 0, &point) == 1);
    CHECK(point.x == Approx(2.5));
    CHECK(point.y == Approx(3.75));
    REQUIRE(graph_data_get_point(graph, 1, &point) == 1);
    CHECK(point.x == Approx(8.0));
    CHECK(point.y == Approx(9.5));

    CHECK(graph_data_get_point(graph, 2, &point) == 0);
    CHECK(point.x == 0.0);
    CHECK(point.y == 0.0);
    CHECK(graph_data_get_point(graph, 0, nullptr) == 0);
    CHECK(graph_data_get_point(nullptr, 0, &point) == 0);
    CHECK(graph_data_type(nullptr) == RAW_GRAPH_TYPE_NONE);

    const slic3r_api::ConfigOption option_view(option);
    const std::optional<slic3r_api::GraphData> graph_view = option_view.graph();
    REQUIRE(graph_view.has_value());
    CHECK(graph_view->size() == 2);
    CHECK(graph_view->type() == RAW_GRAPH_TYPE_SPLINE);
    CHECK(graph_view->point(0).x == Approx(2.5));
    CHECK(graph_view->point(0).y == Approx(3.75));
    const std::vector<slic3r_api::ConfigPoint> copied_points = graph_view->points();
    REQUIRE(copied_points.size() == 2);
    CHECK(copied_points[1].x == Approx(8.0));
    CHECK(copied_points[1].y == Approx(9.5));
    CHECK(graph_view->validate());
    CHECK_THROWS_AS(graph_view->point(2), std::out_of_range);

    config_option_handle *graphs_option =
        config_get_or_add_mutable(config, "graphs", SLIC3R_CONFIG_OPTION_GRAPHS);
    REQUIRE(graphs_option != nullptr);
    REQUIRE(config_option_deserialize(
        graphs_option, "0:2:1:0x0:1x1,0:2:1:2x2:3x3", 0) == 1);
    config_option_vector_handle *graphs = config_option_vector_cast_mutable(graphs_option);
    REQUIRE(graphs != nullptr);

    CHECK(config_option_vector_deserialize_at(graphs, 1, "0:2:0:4.5x6.5:8.5x10.5") == 1);
    const graph_data_handle *replaced = config_option_get_graph(graphs_option, 1);
    REQUIRE(replaced != nullptr);
    CHECK(graph_data_type(replaced) == RAW_GRAPH_TYPE_SQUARE);
    REQUIRE(graph_data_get_point(replaced, 0, &point) == 1);
    CHECK(point.x == Approx(4.5));
    CHECK(point.y == Approx(6.5));

    char replaced_serialized[128] = {};
    REQUIRE(config_option_vector_serialize_at(graphs, 1, replaced_serialized, sizeof(replaced_serialized)) > 0);
    CHECK(config_option_vector_deserialize_at(graphs, 1, "invalid") == 0);
    char after_invalid[128] = {};
    REQUIRE(config_option_vector_serialize_at(graphs, 1, after_invalid, sizeof(after_invalid)) > 0);
    CHECK(std::strcmp(replaced_serialized, after_invalid) == 0);
    CHECK(config_option_vector_deserialize_at(
        graphs, 1, "invalid,0:2:1:2x2:3x3") == 0);
    char after_partially_invalid[128] = {};
    REQUIRE(config_option_vector_serialize_at(
        graphs, 1, after_partially_invalid, sizeof(after_partially_invalid)) > 0);
    CHECK(std::strcmp(replaced_serialized, after_partially_invalid) == 0);
    const char *const malformed_graphs[] = {
        "0:2:1:2x2:3x3suffix",
        "0:2:1:2x2:3x3;invalid",
        "0:2:99:2x2:3x3"
    };
    for (const char *malformed_graph : malformed_graphs) {
        CAPTURE(malformed_graph);
        CHECK(config_option_vector_deserialize_at(graphs, 1, malformed_graph) == 0);
        char after_malformed[128] = {};
        REQUIRE(config_option_vector_serialize_at(
            graphs, 1, after_malformed, sizeof(after_malformed)) > 0);
        CHECK(std::strcmp(replaced_serialized, after_malformed) == 0);
    }
    CHECK(config_option_vector_deserialize_at(graphs, -1, "0:0:1") == 0);
    CHECK(config_option_vector_deserialize_at(graphs, 2, "0:0:1") == 0);
    CHECK(config_option_vector_deserialize_at(nullptr, 0, "0:0:1") == 0);
    CHECK(config_option_vector_deserialize_at(graphs, 0, nullptr) == 0);
}

TEST_CASE("Vector item deserialization rejects disabled bools for non-disableable options",
          "[plugins][config][serialization][vector]")
{
    Slic3r::ConfigOptionBools values{true, false};
    config_option_handle *option = reinterpret_cast<config_option_handle *>(&values);
    config_option_vector_handle *vector = config_option_vector_cast_mutable(option);
    REQUIRE(vector != nullptr);
    REQUIRE_FALSE(values.can_be_disabled());

    CHECK(config_option_vector_deserialize_at(vector, 0, "!0") == 0);
    REQUIRE(values.size() == 2);
    CHECK(values.get_at(0));
    CHECK_FALSE(values.get_at(1));
}

TEST_CASE("Vector item deserialization preserves strings beginning with an exclamation mark",
          "[plugins][config][serialization][vector]")
{
    Slic3r::ConfigOptionStrings values{"before", "after"};
    config_option_handle *option = reinterpret_cast<config_option_handle *>(&values);
    config_option_vector_handle *vector = config_option_vector_cast_mutable(option);
    REQUIRE(vector != nullptr);

    CHECK(config_option_vector_deserialize_at(vector, 1, "!hello") == 1);
    CHECK(values.get_at(1) == "!hello");

    char serialized[32] = {};
    REQUIRE(config_option_vector_serialize_at(vector, 1, serialized, sizeof(serialized)) > 0);
    CHECK(std::strcmp(serialized, "!hello") == 0);
    CHECK(config_option_vector_deserialize_at(vector, 0, serialized) == 1);
    CHECK(values.get_at(0) == "!hello");
}

TEST_CASE("Vector item deserialization supports disabled strings when permitted",
          "[plugins][config][serialization][vector]")
{
    Slic3r::ConfigOptionStrings values{"active", "before"};
    values.set_can_be_disabled();
    config_option_handle *option = reinterpret_cast<config_option_handle *>(&values);
    config_option_vector_handle *vector = config_option_vector_cast_mutable(option);
    REQUIRE(vector != nullptr);

    REQUIRE(config_option_vector_deserialize_at(vector, 1, "!:hello") == 1);
    CHECK(values.get_at(1) == "hello");
    CHECK(config_option_is_enabled(option, 0) == 1);
    CHECK(config_option_is_enabled(option, 1) == 0);
    CHECK(config_option_is_enabled(option, -1) == 1);

    char serialized[32] = {};
    REQUIRE(config_option_vector_serialize_at(vector, 1, serialized, sizeof(serialized)) > 0);
    CHECK(std::strcmp(serialized, "!:hello") == 0);
}

TEST_CASE("Vector string items round-trip through their element serialization format",
          "[plugins][config][serialization][vector]")
{
    const std::string source_values[] = {"a;b", "", "first\nsecond", "  leading spaces"};
    for (const std::string &source_value : source_values) {
        CAPTURE(source_value);
        Slic3r::ConfigOptionStrings values{source_value, "destination"};
        config_option_handle *option = reinterpret_cast<config_option_handle *>(&values);
        config_option_vector_handle *vector = config_option_vector_cast_mutable(option);
        REQUIRE(vector != nullptr);

        char serialized[128] = {};
        CHECK(config_option_vector_serialize_at(vector, 0, serialized, sizeof(serialized)) ==
              values.serialize_at(0).size());
        CHECK(config_option_vector_deserialize_at(vector, 1, serialized) == 1);
        CHECK(values.get_at(1) == source_value);
    }
}

TEST_CASE("Replacing one vector item recomputes the option's global enabled state",
          "[plugins][config][serialization][vector]")
{
    Slic3r::ConfigOptionFloats values{1.0, 2.0};
    values.set_can_be_disabled();
    config_option_handle *option = reinterpret_cast<config_option_handle *>(&values);
    config_option_vector_handle *vector = config_option_vector_cast_mutable(option);
    REQUIRE(vector != nullptr);

    REQUIRE(config_option_is_enabled(option, -1) == 1);
    REQUIRE(config_option_is_enabled(option, 0) == 1);
    REQUIRE(config_option_is_enabled(option, 1) == 1);

    REQUIRE(config_option_vector_deserialize_at(vector, 1, "!3") == 1);
    CHECK(values.get_at(1) == Approx(3.0));
    CHECK(config_option_is_enabled(option, 0) == 1);
    CHECK(config_option_is_enabled(option, 1) == 0);
    CHECK(config_option_is_enabled(option, -1) == 1);

    REQUIRE(config_option_vector_deserialize_at(vector, 0, "!4") == 1);
    CHECK(config_option_is_enabled(option, -1) == 0);
    REQUIRE(config_option_vector_deserialize_at(vector, 1, "5") == 1);
    CHECK(config_option_is_enabled(option, 0) == 0);
    CHECK(config_option_is_enabled(option, 1) == 1);
    CHECK(config_option_is_enabled(option, -1) == 1);
}

TEST_CASE("Every vector option type round-trips one canonical item",
          "[plugins][config][serialization][vector][types]")
{
    SECTION("Bool") {
        Slic3r::ConfigOptionBools values{true, false};
        check_vector_item_round_trip(values);
        values.set_can_be_disabled();
        values.set_enabled(false, 0);
        values.set_enabled(true, 1);
        check_vector_item_round_trip(values);
    }
    SECTION("Int") {
        Slic3r::ConfigOptionInts values{-17, 42};
        check_vector_item_round_trip(values);
        values.set_can_be_disabled();
        values.set_enabled(false, 0);
        values.set_enabled(true, 1);
        check_vector_item_round_trip(values);
    }
    SECTION("Float") {
        Slic3r::ConfigOptionFloats values{-3.25, 4.5};
        check_vector_item_round_trip(values);
        values.set_can_be_disabled();
        values.set_enabled(false, 0);
        values.set_enabled(true, 1);
        check_vector_item_round_trip(values);
    }
    SECTION("Percent") {
        Slic3r::ConfigOptionPercents values{12.5, 75.0};
        check_vector_item_round_trip(values);
        values.set_can_be_disabled();
        values.set_enabled(false, 0);
        values.set_enabled(true, 1);
        check_vector_item_round_trip(values);
    }
    SECTION("Float or percent") {
        Slic3r::ConfigOptionFloatsOrPercents values{{12.5, true}, {-3.25, false}};
        check_vector_item_round_trip(values);
        values.set_can_be_disabled();
        values.set_enabled(false, 0);
        values.set_enabled(true, 1);
        check_vector_item_round_trip(values);
    }
    SECTION("String") {
        Slic3r::ConfigOptionStrings values{"first; line\n", "destination"};
        check_vector_item_round_trip(values);
        values.set_can_be_disabled();
        values.set_enabled(false, 0);
        values.set_enabled(true, 1);
        check_vector_item_round_trip(values);
    }
    SECTION("Point") {
        Slic3r::ConfigOptionPoints values{
            Slic3r::Vec2d(-1.5, 2.25), Slic3r::Vec2d(3.0, 4.0)};
        check_vector_item_round_trip(values);
        values.set_can_be_disabled();
        values.set_enabled(false, 0);
        values.set_enabled(true, 1);
        check_vector_item_round_trip(values);
    }
    SECTION("Graph") {
        const Slic3r::GraphData source(
            0, 2, Slic3r::GraphData::GraphType::LINEAR,
            {Slic3r::Vec2d(0.0, 1.0), Slic3r::Vec2d(2.0, 3.0)});
        const Slic3r::GraphData destination(
            0, 1, Slic3r::GraphData::GraphType::SQUARE,
            {Slic3r::Vec2d(4.0, 5.0)});
        Slic3r::ConfigOptionGraphs values{source, destination};
        check_vector_item_round_trip(values);
        values.set_can_be_disabled();
        values.set_enabled(false, 0);
        values.set_enabled(true, 1);
        check_vector_item_round_trip(values);
    }
}

TEST_CASE("Every vector option type rejects malformed items atomically",
          "[plugins][config][serialization][vector][types]")
{
    SECTION("Bool") {
        Slic3r::ConfigOptionBools values{true, false};
        check_vector_item_rejections(
            values, {"", "2", "true", "1,!0", "1,0", "1,"});
    }
    SECTION("Int") {
        Slic3r::ConfigOptionInts values{-17, 42};
        check_vector_item_rejections(
            values, {"", "1.5", "1suffix", "1,2", "2147483648", "!1"});
    }
    SECTION("Float") {
        Slic3r::ConfigOptionFloats values{-3.25, 4.5};
        check_vector_item_rejections(
            values, {"", "NaN", "inf", "1suffix", "1,2", "!1"});
    }
    SECTION("Percent") {
        Slic3r::ConfigOptionPercents values{12.5, 75.0};
        check_vector_item_rejections(
            values, {"", "NaN", "10%%", "10%suffix", "10,20", "!10%"});
    }
    SECTION("Float or percent") {
        Slic3r::ConfigOptionFloatsOrPercents values{{12.5, true}, {-3.25, false}};
        check_vector_item_rejections(
            values, {"", "NaN", "10%%", "10%suffix", "10,20", "!10%"});
    }
    SECTION("String") {
        Slic3r::ConfigOptionStrings values{"first", "second"};
        check_vector_item_rejections(values, {"trailing\\", "!:disabled"});
    }
    SECTION("Point") {
        Slic3r::ConfigOptionPoints values{
            Slic3r::Vec2d(-1.5, 2.25), Slic3r::Vec2d(3.0, 4.0)};
        check_vector_item_rejections(
            values, {"", "1x", "x2", "1x2x3", "1x2suffix", "1x2,3x4", "!1x2"});
    }
    SECTION("Graph") {
        const Slic3r::GraphData graph(
            0, 1, Slic3r::GraphData::GraphType::LINEAR,
            {Slic3r::Vec2d(0.0, 1.0)});
        Slic3r::ConfigOptionGraphs values{graph, graph};
        check_vector_item_rejections(
            values, {"", "0:1:99:0x1", "0:1:1:0x1suffix", "0:1:1:0x1;invalid", "!0:1:1:0x1"});
    }
}

TEST_CASE("Vector item deserialization rejects invalid floating-point text atomically",
          "[plugins][config][serialization][vector]")
{
    Slic3r::ConfigOptionFloats values{1.0, 2.0};
    config_option_handle *option = reinterpret_cast<config_option_handle *>(&values);
    config_option_vector_handle *vector = config_option_vector_cast_mutable(option);
    REQUIRE(vector != nullptr);

    CHECK(config_option_vector_deserialize_at(vector, 1, "invalid") == 0);
    REQUIRE(values.size() == 2);
    CHECK(values.get_at(0) == Approx(1.0));
    CHECK(values.get_at(1) == Approx(2.0));

    CHECK(config_option_vector_deserialize_at(vector, 1, "3.5") == 1);
    CHECK(values.get_at(1) == Approx(3.5));
    CHECK(config_option_vector_deserialize_at(vector, 1, "4.5suffix") == 0);
    CHECK(values.get_at(1) == Approx(3.5));
}

TEST_CASE("SCFG C API follows the two-call buffer contract", "[plugins][config][serialization]")
{
    Slic3r::DynamicConfig source;
    source.set_key_value("value", new Slic3r::ConfigOptionString("left:\nright"));
    const std::string serialized = serialize_all(Slic3r::ApiHost::to_config_handle(&source));
    CHECK(serialized.rfind("SCFG1\n", 0) == 0);

    Slic3r::DynamicConfig restored;
    REQUIRE(config_deserialize_all(Slic3r::ApiHost::to_config_handle(&restored), serialized.c_str()) == 1);
    CHECK(restored.option<Slic3r::ConfigOptionString>("value")->value == "left:\nright");
}

TEST_CASE("SCFG merge is atomic and allows an incoming type change", "[plugins][config][serialization]")
{
    Slic3r::DynamicConfig destination;
    destination.set_key_value("keep", new Slic3r::ConfigOptionInt(7));
    destination.set_key_value("replace", new Slic3r::ConfigOptionInt(8));

    Slic3r::DynamicConfig incoming;
    incoming.set_key_value("replace", new Slic3r::ConfigOptionString("changed"));
    incoming.set_key_value("added", new Slic3r::ConfigOptionBool(true));
    const std::string incoming_serialized = serialize_all(Slic3r::ApiHost::to_config_handle(&incoming));

    REQUIRE(config_deserialize_all(
        Slic3r::ApiHost::to_config_handle(&destination), incoming_serialized.c_str()) == 1);
    CHECK(destination.option< Slic3r::ConfigOptionInt>("keep")->value == 7);
    CHECK(destination.option<Slic3r::ConfigOptionString>("replace")->value == "changed");
    CHECK(destination.option<Slic3r::ConfigOptionBool>("added")->value);

    const std::string before_failure = serialize_all(Slic3r::ApiHost::to_config_handle(&destination));
    CHECK(config_deserialize_all(Slic3r::ApiHost::to_config_handle(&destination), "SCFG2\n0\n") == 0);
    CHECK(serialize_all(Slic3r::ApiHost::to_config_handle(&destination)) == before_failure);

    REQUIRE(config_clear(Slic3r::ApiHost::to_config_handle(&destination)) == 1);
    REQUIRE(config_deserialize_all(
        Slic3r::ApiHost::to_config_handle(&destination), incoming_serialized.c_str()) == 1);
    CHECK(destination.keys() == incoming.keys());
}

TEST_CASE("StoredConfig provides move-only C++ ownership", "[plugins][config][cpp]")
{
    Slic3r::PluginStorage storage;
    storage_handle *storage_api = reinterpret_cast<storage_handle *>(&storage);

    {
        slic3r_api::StoredConfig source(storage_api);
        slic3r_api::MutableConfigOption layer =
            source.get_or_add("layer_num", SLIC3R_CONFIG_OPTION_INT);
        layer.set_int(12);
        const std::string serialized = source.serialize_all();

        slic3r_api::StoredConfig destination(storage_api);
        destination.get_or_add("preserved", SLIC3R_CONFIG_OPTION_BOOL).set_bool(true);
        destination.deserialize_all(serialized);
        CHECK(destination.int_or_default("layer_num", 0) == 12);
        CHECK(destination.bool_or_default("preserved", false));

        slic3r_api::StoredConfig moved(std::move(destination));
        CHECK(moved.int_or_default("layer_num", 0) == 12);
        moved.clear();
        CHECK(moved.keys().empty());
    }

    CHECK(storage_size(storage_api) == 0);
}

TEST_CASE("Configuration definition keys are ordered and filterable",
          "[plugins][config][definition]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Slic3r::Orchestrator &orchestrator = Slic3r::Orchestrator::instance();
    const orchestrator_handle *handle = reinterpret_cast<const orchestrator_handle *>(&orchestrator);

    CHECK(orchestrator_get_option_keys(nullptr, RAW_CO_NONE, nullptr, 0) == 0);
    const uint32_t total = orchestrator_get_option_keys(handle, RAW_CO_NONE, nullptr, 0);
    REQUIRE(total > 2);

    std::vector<raw_config_option_key> keys(total);
    CHECK(orchestrator_get_option_keys(handle, RAW_CO_NONE, keys.data(), uint32_t(keys.size())) == total);
    CHECK(std::is_sorted(keys.begin(), keys.end(),
        [](const raw_config_option_key &lhs, const raw_config_option_key &rhs) {
            return std::strcmp(lhs.key, rhs.key) < 0;
        }));

    std::array<raw_config_option_key, 2> prefix = {};
    CHECK(orchestrator_get_option_keys(handle, RAW_CO_NONE, prefix.data(), uint32_t(prefix.size())) == total);
    CHECK(std::strcmp(prefix[0].key, keys[0].key) == 0);
    CHECK(std::strcmp(prefix[1].key, keys[1].key) == 0);

    for (int value = int(RAW_CO_BOOL); value <= int(RAW_CO_POINT3); ++value) {
        const raw_config_option_type filter = static_cast<raw_config_option_type>(value);
        const uint32_t filtered_count = orchestrator_get_option_keys(handle, filter, nullptr, 0);
        std::vector<raw_config_option_key> filtered(filtered_count);
        CHECK(orchestrator_get_option_keys(
            handle, filter, filtered.data(), uint32_t(filtered.size())) == filtered_count);
        for (const raw_config_option_key &entry : filtered)
            CHECK(entry.type == filter);
    }

    const std::vector<raw_config_option_key>::const_iterator point3 = std::find_if(
        keys.begin(), keys.end(), [](const raw_config_option_key &entry) {
            return std::strcmp(entry.key, "test_config_definition_point3_introspection") == 0;
        });
    REQUIRE(point3 != keys.end());
    CHECK(point3->type == RAW_CO_POINT3);
}

TEST_CASE("Configuration definitions expose normalized plugin metadata",
          "[plugins][config][definition]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Slic3r::Orchestrator &orchestrator = Slic3r::Orchestrator::instance();
    const orchestrator_handle *handle = reinterpret_cast<const orchestrator_handle *>(&orchestrator);

    raw_config_option_def raw;
    std::memset(&raw, 0xA5, sizeof(raw));
    CHECK(orchestrator_get_option_def(handle, "missing.test.option", &raw) == 0);
    const unsigned char *bytes = reinterpret_cast<const unsigned char *>(&raw);
    CHECK(std::all_of(bytes, bytes + sizeof(raw), [](unsigned char value) { return value == 0; }));
    CHECK(orchestrator_get_option_def(nullptr, "layer_height", &raw) == 0);
    CHECK(orchestrator_get_option_def(handle, nullptr, &raw) == 0);
    CHECK(orchestrator_get_option_def(handle, "layer_height", nullptr) == 0);

    REQUIRE(orchestrator_get_option_def(handle, "test_config_definition_introspection", &raw) == 1);
    CHECK(std::string(raw.opt_key) == "test_config_definition_introspection");
    CHECK(raw.type == RAW_CO_FLOAT);
    CHECK(raw.category == RAW_OPTION_CATEGORY_ADVANCED);
    CHECK(raw.level == RAW_OPTION_LEVEL_EXPERT);
    CHECK(raw.container_type == RAW_CONTAINER_TYPE_PROJECT);
    CHECK(raw.option_preset_type == RAW_PRESET_TYPE_FFF_PRINT);
    CHECK(raw.gui_type == RAW_GUI_TYPE_F_ENUM_OPEN);
    CHECK(raw.printer_technology == RAW_PT_FFF);
    CHECK(raw.invalidates_step == STEP_INFILL);
    CHECK(raw.is_optional == 1);
    CHECK(std::string(raw.gui_flags) == "serialized,show_value");
    CHECK(std::string(raw.label) == "Introspection value");
    CHECK(std::string(raw.full_label) == "Plugin configuration introspection value");
    CHECK(std::string(raw.tooltip) == "Exercises the complete normalized definition snapshot.");
    CHECK(std::string(raw.sidetext) == "mm");
    CHECK(std::string(raw.cli) == "test-config-definition-introspection=f");
    CHECK(std::string(raw.ratio_over) == "nozzle_diameter");
    CHECK(raw.multiline == 1);
    CHECK(raw.full_width == 1);
    CHECK(raw.is_code == 1);
    CHECK(raw.is_vector_extruder == 0);
    CHECK(raw.readonly == 1);
    CHECK(raw.can_phony == 1);
    CHECK(raw.can_be_disabled == 1);
    CHECK(raw.height == 7);
    CHECK(raw.width == 19);
    CHECK(raw.label_width == 23);
    CHECK(raw.aligned_label_left == 1);
    CHECK(raw.sidetext_width == 11);
    CHECK(raw.has_min == 1);
    CHECK(raw.min_value == Approx(-2.5));
    CHECK(raw.has_max == 1);
    CHECK(raw.max_value == Approx(8.5));
    CHECK(raw.has_max_literal == 1);
    CHECK(raw.max_literal_value == Approx(4.5));
    CHECK(raw.max_literal_is_percent == 1);
    CHECK(raw.precision == 4);
    CHECK(raw.mode == RAW_CONFIG_OPTION_MODE_EXPERT);
    CHECK(raw.flags == uint64_t(0x1020304050607080ULL));
    REQUIRE(raw.aliases.size == 2);
    CHECK(std::string(raw.aliases.items[0]) == "test_introspection_old");
    CHECK(std::string(raw.aliases.items[1]) == "test_introspection_legacy");
    REQUIRE(raw.shortcut.size == 2);
    CHECK(std::string(raw.shortcut.items[0]) == "perimeter_speed");
    CHECK(std::string(raw.shortcut.items[1]) == "infill_speed");
    REQUIRE(raw.depends_on.size == 2);
    CHECK(std::string(raw.depends_on.items[0]) == "layer_height#0");
    CHECK(std::string(raw.depends_on.items[1]) == "nozzle_diameter#0");
    CHECK(std::string(raw.default_serialized_value) == "1.25");
    REQUIRE(raw.enum_def.value_label_pairs.count == 2);
    CHECK(std::string(raw.enum_def.value_label_pairs.items[0].value) == "0.5");
    CHECK(std::string(raw.enum_def.value_label_pairs.items[0].label) == "Half");
    CHECK(std::string(raw.enum_def.value_label_pairs.items[1].value) == "1.25");
    CHECK(std::string(raw.enum_def.value_label_pairs.items[1].label) == "Nominal");
    CHECK(raw.graph_settings == nullptr);
    for (uint64_t value : raw.reserved_u64)
        CHECK(value == 0);
    for (const void *value : raw.reserved_ptr)
        CHECK(value == nullptr);

    REQUIRE(orchestrator_get_option_def(
        handle, "test_config_definition_vector_introspection", &raw) == 1);
    CHECK(raw.type == RAW_CO_VECTOR_FLOAT);
    CHECK(raw.is_vector_extruder == 1);

    REQUIRE(orchestrator_get_option_def(
        handle, "test_config_definition_graph_introspection", &raw) == 1);
    REQUIRE(raw.graph_settings != nullptr);
    const raw_graph_settings &graph = *raw.graph_settings;
    CHECK(std::string(graph.title) == "Plugin graph");
    CHECK(std::string(graph.description) == "Configure a value along the X axis.");
    CHECK(std::string(graph.x_label) == "Position");
    CHECK(std::string(graph.y_label) == "Value");
    CHECK(graph.min_x == Approx(0.0));
    CHECK(graph.max_x == Approx(100.0));
    CHECK(graph.step_x == Approx(1.0));
    CHECK(graph.min_y == Approx(0.0));
    CHECK(graph.max_y == Approx(50.0));
    CHECK(graph.step_y == Approx(0.5));
    REQUIRE(graph.allowed_type_count == 2);
    CHECK(graph.allowed_types[0] == RAW_GRAPH_TYPE_LINEAR);
    CHECK(graph.allowed_types[1] == RAW_GRAPH_TYPE_SPLINE);
    CHECK(graph.reset_values.begin_index == 0);
    CHECK(graph.reset_values.end_index == 2);
    CHECK(graph.reset_values.type == RAW_GRAPH_TYPE_SPLINE);
    REQUIRE(graph.reset_values.point_count == 2);
    CHECK(graph.reset_values.points[1].x == Approx(100.0));
    CHECK(graph.reset_values.points[1].y == Approx(20.0));
    REQUIRE(graph.enforced_value_count == 2);
    CHECK(std::isnan(graph.enforced_values[0].x));
    CHECK(graph.enforced_values[0].y == Approx(10.0));
}

TEST_CASE("Configuration definitions use sentinel values when limits are absent",
          "[plugins][config][definition]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Slic3r::Orchestrator &orchestrator = Slic3r::Orchestrator::instance();
    const orchestrator_handle *handle = reinterpret_cast<const orchestrator_handle *>(&orchestrator);
    const double no_min = -std::numeric_limits<float>::max();
    const double no_max = std::numeric_limits<float>::max();

    const Slic3r::ConfigOptionDef *native_definition =
        Slic3r::PrintConfigDef::instance().get("test_config_definition_vector_introspection");
    REQUIRE(native_definition != nullptr);
    CHECK(native_definition->min == no_min);
    CHECK(native_definition->max == no_max);
    CHECK(native_definition->max_literal.value == 0.0);
    CHECK_FALSE(native_definition->max_literal.percent);

    raw_config_option_def raw = raw_config_option_def_init();
    REQUIRE(orchestrator_get_option_def(
        handle, "test_config_definition_vector_introspection", &raw) == 1);
    CHECK(raw.has_min == 0);
    CHECK(raw.min_value == no_min);
    CHECK(raw.has_max == 0);
    CHECK(raw.max_value == no_max);
    CHECK(raw.has_max_literal == 0);
    CHECK(raw.max_literal_value == 0.0);
    CHECK(raw.max_literal_is_percent == 0);
}

TEST_CASE("Identical graph definitions accept NaN endpoint sentinels",
          "[plugins][config][definition][graph]")
{
    // Runtime initialization registers this graph definition twice. Reaching
    // this assertion proves that the second declaration was compatible.
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const orchestrator_handle *handle = reinterpret_cast<const orchestrator_handle *>(
        &Slic3r::Orchestrator::instance());
    raw_config_option_def definition = {};

    REQUIRE(orchestrator_get_option_def(
        handle, "test_config_definition_graph_introspection", &definition) == 1);
    REQUIRE(definition.graph_settings != nullptr);
    REQUIRE(definition.graph_settings->enforced_value_count == 2);
    CHECK(std::isnan(definition.graph_settings->enforced_values[0].x));
    CHECK(definition.graph_settings->enforced_values[0].y == Approx(10.0));
}

TEST_CASE("Explicit graph reset values remain distinct from the option default",
          "[plugins][config][definition][graph]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const orchestrator_handle *handle = reinterpret_cast<const orchestrator_handle *>(
        &Slic3r::Orchestrator::instance());

    const Slic3r::ConfigOptionDef *native_definition =
        Slic3r::PrintConfigDef::instance().get("test_config_definition_graph_introspection");
    REQUIRE(native_definition != nullptr);
    REQUIRE(native_definition->graph_settings != nullptr);
    CHECK(native_definition->graph_settings->has_reset_vals);
    CHECK(native_definition->graph_settings->reset_vals.type == Slic3r::GraphData::GraphType::SPLINE);
    REQUIRE(static_cast<bool>(native_definition->default_value));
    CHECK(native_definition->get_default_value<Slic3r::ConfigOptionGraph>()->value.type ==
          Slic3r::GraphData::GraphType::LINEAR);

    raw_config_option_def native_graph = {};
    REQUIRE(orchestrator_get_option_def(handle, "extruder_clearance", &native_graph) == 1);
    REQUIRE(native_graph.graph_settings != nullptr);
    CHECK(native_graph.graph_settings->reset_values.type == RAW_GRAPH_TYPE_NONE);
    CHECK(native_graph.graph_settings->reset_values.point_count == 0);
}

TEST_CASE("C++ configuration definition views own their copied data",
          "[plugins][config][definition][cpp]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Slic3r::Orchestrator &orchestrator = Slic3r::Orchestrator::instance();
    slic3r_api::OrchestratorView view(reinterpret_cast<orchestrator_handle *>(&orchestrator));

    const std::vector<slic3r_api::ConfigOptionKey> keys = view.option_keys(RAW_CO_POINT3);
    REQUIRE_FALSE(keys.empty());
    CHECK(std::any_of(keys.begin(), keys.end(), [](const slic3r_api::ConfigOptionKey &entry) {
        return entry.key == "test_config_definition_point3_introspection" && entry.type == RAW_CO_POINT3;
    }));

    const std::optional<slic3r_api::ConfigOptionDefinition> definition =
        view.option_definition("test_config_definition_introspection");
    REQUIRE(definition.has_value());
    REQUIRE(view.option_definition("layer_height").has_value());
    CHECK(definition->label == std::optional<std::string>("Introspection value"));
    CHECK(definition->aliases == std::vector<std::string>{
        "test_introspection_old", "test_introspection_legacy"});
    REQUIRE(definition->enum_entries.size() == 2);
    CHECK(definition->enum_entries[1].label == std::optional<std::string>("Nominal"));

    const std::optional<slic3r_api::ConfigOptionDefinition> graph_definition =
        view.option_definition("test_config_definition_graph_introspection");
    REQUIRE(graph_definition.has_value());
    REQUIRE(graph_definition->graph_settings.has_value());
    CHECK(graph_definition->graph_settings->title == std::optional<std::string>("Plugin graph"));
    CHECK(graph_definition->graph_settings->allowed_types == std::vector<raw_graph_type>{
        RAW_GRAPH_TYPE_LINEAR, RAW_GRAPH_TYPE_SPLINE});
    REQUIRE(graph_definition->graph_settings->reset_values.points.size() == 2);
    CHECK(graph_definition->graph_settings->reset_values.points[1].x == Approx(100.0));
    CHECK(graph_definition->graph_settings->enforced_values.size() == 2);
    CHECK_FALSE(view.option_definition("missing.test.option").has_value());
}

TEST_CASE("Graph editor definitions reject incompatible option types",
          "[plugins][config][definition][graph]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Slic3r::Orchestrator &orchestrator = Slic3r::Orchestrator::instance();
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&orchestrator);

    raw_graph_settings graph_settings = {};
    const raw_graph_type allowed[] = {RAW_GRAPH_TYPE_LINEAR};
    graph_settings.min_x = 0.0;
    graph_settings.max_x = 100.0;
    graph_settings.step_x = 1.0;
    graph_settings.min_y = 0.0;
    graph_settings.max_y = 50.0;
    graph_settings.step_y = 0.5;
    graph_settings.allowed_types = allowed;
    graph_settings.allowed_type_count = 1;

    raw_config_option_def definition = raw_config_option_def_init();
    definition.opt_key = "test.invalid.graph.settings";
    definition.type = RAW_CO_FLOAT;
    definition.container_type = RAW_CONTAINER_TYPE_PROJECT;
    definition.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    definition.printer_technology = RAW_PT_FFF;
    definition.default_serialized_value = "1";
    definition.graph_settings = &graph_settings;
    CHECK(orchestrator_create_option_def(handle, &definition) == OPTION_DEF_ERROR_INVALID_ARGUMENT);

    definition.type = RAW_CO_GRAPH;
    graph_settings.allowed_types = nullptr;
    CHECK(orchestrator_create_option_def(handle, &definition) == OPTION_DEF_ERROR_INVALID_ARGUMENT);
}

TEST_CASE("Graph editor definitions reject unusable ranges and increments",
          "[plugins][config][definition][graph]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Slic3r::Orchestrator &orchestrator = Slic3r::Orchestrator::instance();
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&orchestrator);
    const raw_graph_type allowed[] = {RAW_GRAPH_TYPE_LINEAR};

    raw_graph_settings graph_settings = {};
    graph_settings.min_x = 0.0;
    graph_settings.max_x = 100.0;
    graph_settings.step_x = 1.0;
    graph_settings.min_y = 0.0;
    graph_settings.max_y = 50.0;
    graph_settings.step_y = 0.5;
    graph_settings.allowed_types = allowed;
    graph_settings.allowed_type_count = 1;

    raw_config_option_def definition = raw_config_option_def_init();
    definition.opt_key = "test.invalid.graph.range";
    definition.type = RAW_CO_GRAPH;
    definition.container_type = RAW_CONTAINER_TYPE_PROJECT;
    definition.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    definition.printer_technology = RAW_PT_FFF;
    definition.default_serialized_value = "0:2:1:0x10:100x20";
    definition.graph_settings = &graph_settings;

    SECTION("zero-initialized settings")
    {
        raw_graph_settings zero_settings = {};
        definition.graph_settings = &zero_settings;
        CHECK(orchestrator_create_option_def(handle, &definition) == OPTION_DEF_ERROR_INVALID_ARGUMENT);
    }
    SECTION("non-finite X bound")
    {
        graph_settings.min_x = std::numeric_limits<double>::quiet_NaN();
        CHECK(orchestrator_create_option_def(handle, &definition) == OPTION_DEF_ERROR_INVALID_ARGUMENT);
    }
    SECTION("non-finite Y bound")
    {
        graph_settings.max_y = std::numeric_limits<double>::infinity();
        CHECK(orchestrator_create_option_def(handle, &definition) == OPTION_DEF_ERROR_INVALID_ARGUMENT);
    }
    SECTION("empty X range")
    {
        graph_settings.max_x = graph_settings.min_x;
        CHECK(orchestrator_create_option_def(handle, &definition) == OPTION_DEF_ERROR_INVALID_ARGUMENT);
    }
    SECTION("reversed Y range")
    {
        graph_settings.min_y = graph_settings.max_y + 1.0;
        CHECK(orchestrator_create_option_def(handle, &definition) == OPTION_DEF_ERROR_INVALID_ARGUMENT);
    }
    SECTION("zero X increment")
    {
        graph_settings.step_x = 0.0;
        CHECK(orchestrator_create_option_def(handle, &definition) == OPTION_DEF_ERROR_INVALID_ARGUMENT);
    }
    SECTION("negative Y increment")
    {
        graph_settings.step_y = -0.5;
        CHECK(orchestrator_create_option_def(handle, &definition) == OPTION_DEF_ERROR_INVALID_ARGUMENT);
    }
    SECTION("non-finite increment")
    {
        graph_settings.step_x = std::numeric_limits<double>::infinity();
        CHECK(orchestrator_create_option_def(handle, &definition) == OPTION_DEF_ERROR_INVALID_ARGUMENT);
    }
    SECTION("small positive range remains supported")
    {
        raw_config_option_def small_definition = {};
        REQUIRE(orchestrator_get_option_def(
            handle, "test_config_definition_small_graph_range", &small_definition) == 1);
        REQUIRE(small_definition.graph_settings != nullptr);
        CHECK(small_definition.graph_settings->min_y == 0.0);
        CHECK(small_definition.graph_settings->max_y == Approx(0.001));
        CHECK(small_definition.graph_settings->step_y == Approx(0.0001));
    }
}

TEST_CASE("Configuration definition snapshots are isolated between threads",
          "[plugins][config][definition][threading]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Slic3r::Orchestrator &orchestrator = Slic3r::Orchestrator::instance();
    const orchestrator_handle *handle = reinterpret_cast<const orchestrator_handle *>(&orchestrator);
    std::atomic<int> ready {0};
    std::array<std::string, 2> before;
    std::array<std::string, 2> after;
    std::array<uintptr_t, 2> addresses = {};
    std::array<bool, 2> succeeded = {};

    const auto query = [&](size_t index, const char *key) {
        raw_config_option_def raw = {};
        succeeded[index] = orchestrator_get_option_def(handle, key, &raw) && raw.label != nullptr;
        if (succeeded[index]) {
            before[index] = raw.label;
            addresses[index] = reinterpret_cast<uintptr_t>(raw.label);
        }
        ready.fetch_add(1, std::memory_order_release);
        while (ready.load(std::memory_order_acquire) < 2)
            std::this_thread::yield();
        if (succeeded[index])
            after[index] = raw.label;
    };

    std::thread first(query, 0, "test_config_definition_introspection");
    std::thread second(query, 1, "layer_height");
    first.join();
    second.join();

    REQUIRE(ready.load(std::memory_order_relaxed) == 2);
    REQUIRE(succeeded[0]);
    REQUIRE(succeeded[1]);
    CHECK(before == after);
    CHECK(addresses[0] != addresses[1]);
}


namespace {
// Deliberately collide two unequal dynamic values to exercise the equality
// check inside a hash bucket, independently of the platform hash algorithm.
class CollidingRegionOption : public Slic3r::ConfigOptionInt
{
public:
    explicit CollidingRegionOption(int32_t value) : ConfigOptionInt(value) {}
    size_t hash() const noexcept override { return 17; }
    Slic3r::ConfigOption *clone() const override { return new CollidingRegionOption(*this); }
};
}

TEST_CASE("Print registry validates and owns immutable regional snapshots", "[plugins][config][print-region-registry]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    print_handle *handle = reinterpret_cast<print_handle *>(&print);
    DynamicPrintConfig overrides;
    overrides.set_key_value("perimeters", new ConfigOptionInt(7));
    const config_handle *input = ApiHost::to_config_handle(&overrides);
    const print_region_handle *first = print_register_print_region_config(handle, input);
    REQUIRE(first != nullptr);
    CHECK(print_register_print_region_config(handle, input) == first);
    CHECK(print.num_print_regions() == 0); // registration does not activate geometry
    const PrintRegion &native = *reinterpret_cast<const PrintRegion *>(first);
    CHECK(native.config().perimeters.value == 7);
    CHECK(native.config().fill_density == print.default_region_config().fill_density);
    overrides.set_key_value("perimeters", new ConfigOptionInt(8));
    CHECK(native.config().perimeters.value == 7);
    CHECK(print_register_print_region_config(handle, input) != first);
    CHECK(print_register_print_region_config(nullptr, input) == nullptr);
    CHECK(print_register_print_region_config(handle, nullptr) == nullptr);

    SECTION("invalid options do not alter existing snapshots") {
        for (const std::string &key : {std::string("not_a_region_option"), std::string("bed_shape")}) {
            DynamicPrintConfig invalid;
            invalid.set_key_value(key, new ConfigOptionInt(1));
            CHECK(print_register_print_region_config(handle, ApiHost::to_config_handle(&invalid)) == nullptr);
        }
        DynamicPrintConfig wrong_type;
        wrong_type.set_key_value("perimeters", new ConfigOptionString("7"));
        CHECK(print_register_print_region_config(handle, ApiHost::to_config_handle(&wrong_type)) == nullptr);
        CHECK(native.config().perimeters.value == 7);
    }
    SECTION("registered plugin options participate in the snapshot") {
        DynamicPrintConfig dynamic;
        dynamic.set_deserialize_strict("seam_fixed_direction", "front");
        const print_region_handle *front = print_register_print_region_config(handle, ApiHost::to_config_handle(&dynamic));
        REQUIRE(front != nullptr);
        const PrintRegion &front_region = *reinterpret_cast<const PrintRegion *>(front);
        const std::string saved_value = front_region.config().optptr("seam_fixed_direction")->serialize();
        dynamic.set_deserialize_strict("seam_fixed_direction", "rear");
        CHECK(print_register_print_region_config(handle, ApiHost::to_config_handle(&dynamic)) != front);
        CHECK(front_region.config().optptr("seam_fixed_direction")->serialize() == saved_value);
    }
    SECTION("simultaneous registrations return one canonical handle") {
        std::array<const print_region_handle *, 16> results{};
        std::vector<std::thread> threads;
        for (size_t idx = 0; idx < results.size(); ++idx)
            threads.emplace_back([&, idx]() {
                results[idx] = print_register_print_region_config(handle, input);
                if (results[idx] != nullptr)
                    reinterpret_cast<const PrintRegion *>(results[idx])->config().keys();
            });
        for (std::thread &thread : threads) thread.join();
        REQUIRE(results.front() != nullptr);
        for (const print_region_handle *result : results) CHECK(result == results.front());
    }
    SECTION("C++ wrapper uses the same native registry") {
        const slic3r_api::Print view(handle);
        const slic3r_api::PrintRegion region = view.register_print_region_config(slic3r_api::Config(input));
        CHECK(region.handle() == print_register_print_region_config(handle, input));
    }
    print.clear();
    CHECK(print.num_print_regions() == 0);
    // Old handles are invalid now; do not dereference them or compare allocation addresses.
    REQUIRE(print_register_print_region_config(handle, input) != nullptr);
}

TEST_CASE("Print registry distinguishes equal hashes and detaches parent configs", "[plugins][config][print-region-registry]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConfigOptionDef definition;
    definition.opt_key = "test_registry_collision";
    definition.type = coInt;
    definition.container_type = ConfigOptionContainerType::Region;
    definition.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    definition.set_default_value(new CollidingRegionOption(1));
    PrintRegionConfig first;
    first.add_plugin_option(definition);
    PrintRegionConfig second = first;
    REQUIRE(second.optptr(definition.opt_key) != nullptr);
    second.optptr(definition.opt_key)->set_int(2);
    REQUIRE(first.hash() == second.hash());
    REQUIRE(first != second);
    Print print;
    const PrintRegion *a = &print.register_print_region(first);
    const PrintRegion *b = &print.register_print_region(second);
    CHECK(a != b);
    CHECK(&print.register_print_region(first) == a);
    CHECK(&print.register_print_region(second) == b);
    DynamicPrintConfig parent;
    parent.set_key_value("test_parent_only", new ConfigOptionInt(9));
    first.parent = &parent;
    Print other_print;
    CHECK(other_print.register_print_region(first).config().parent == nullptr);
}

TEST_CASE("Print apply reuses the Slicing checkpoint for region and object option changes",
          "[plugins][config][checkpoint][invalidation][integration]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();

    const char *key = nullptr;
    const char *changed_value = nullptr;
    ConfigOptionType option_type = coNone;
    bool region_option = false;
    SECTION("REGION perimeters") {
        key = "perimeters";
        changed_value = "4";
        option_type = coInt;
        region_option = true;
    }
    SECTION("OBJECT support material spacing") {
        key = "support_material_spacing";
        changed_value = "3.5";
        option_type = coFloat;
    }
    CAPTURE(key);
    const ConfigOptionDef *definition = PrintConfigDef::instance().get(key);
    REQUIRE(definition != nullptr);
    REQUIRE(definition->type == option_type);
    REQUIRE(definition->invalidates_step != STEP_ANY);
    REQUIRE(definition->invalidates_step != STEP_NONE);
    REQUIRE(Steps::executes_before(STEP_POST_SLICING, definition->invalidates_step));
    REQUIRE(Steps::checkpoint_before_step(definition->invalidates_step) == Steps::Checkpoint::Slicing);

    // Built-in scopes are defined by static config stores, not plugin container metadata.
    const PrintRegionConfig region_scope;
    const PrintObjectConfig object_scope;
    REQUIRE((region_scope.optptr(key) != nullptr) == region_option);
    REQUIRE((object_scope.optptr(key) != nullptr) == !region_option);

    const std::array<const char *, 7> stage_labels {
        "Creating the layer height", "StepSlicing", "Post-processing slices",
        "Preparing perimeters", "Generating perimeters", "Generating support material",
        "Finalizing slicing"
    };
    std::array<size_t, 7> stage_counts {};
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("perimeters", "2");
    config.set_deserialize_strict("support_material_spacing", "2.5");
    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    print.status().set_callback([&](const PrintBase::SlicingStatus &status) {
        if ((status.flags & PrintBase::SlicingStatus::MAIN_STATE) == 0)
            return;
        for (size_t idx = 0; idx < stage_labels.size(); ++idx)
            if (status.main_text == stage_labels[idx])
                ++stage_counts[idx];
    });
    print.process();
    REQUIRE(print.current_checkpoint() == Steps::Checkpoint::Print);
    for (size_t idx = 0; idx < stage_labels.size(); ++idx) {
        CAPTURE(stage_labels[idx]);
        REQUIRE(stage_counts[idx] == 1);
    }
    REQUIRE(print.objects().size() == 1);
    REQUIRE(print.num_print_regions() == 1);
    const PrintObject *source_object = &print.object(0);
    const std::vector<coord_t> profile = source_object->layer_profile();
    REQUIRE_FALSE(profile.empty());
    REQUIRE(source_object->layer_count() > 0);

    struct SliceSnapshot {
        const Layer *layer;
        ExPolygons islands;
        ExPolygons raw_slices;
    };
    std::vector<SliceSnapshot> slices;
    for (size_t layer_idx = 0; layer_idx < source_object->layer_count(); ++layer_idx) {
        const Layer &layer = source_object->layer(layer_idx);
        REQUIRE(layer.region_count() == 1);
        REQUIRE_FALSE(layer.islands().empty());
        REQUIRE_FALSE(layer.region(0).get_raw_slices().empty());
        SliceSnapshot snapshot {&layer, {}, layer.region(0).get_raw_slices()};
        for (const LayerSliceIsland &island : layer.islands())
            snapshot.islands.push_back(island.get_slice());
        slices.push_back(std::move(snapshot));
    }
    const auto check_retained_slices = [&]() {
        REQUIRE(print.objects().size() == 1);
        const PrintObject &object = print.object(0);
        CHECK(&object == source_object);
        CHECK(object.layer_profile() == profile);
        REQUIRE(object.layer_count() == slices.size());
        for (size_t layer_idx = 0; layer_idx < slices.size(); ++layer_idx) {
            CAPTURE(layer_idx);
            const Layer &layer = object.layer(layer_idx);
            const SliceSnapshot &snapshot = slices[layer_idx];
            CHECK(&layer == snapshot.layer);
            REQUIRE(layer.region_count() == 1);
            CHECK(layer.region(0).get_raw_slices() == snapshot.raw_slices);
            REQUIRE(layer.islands().size() == snapshot.islands.size());
            for (size_t island_idx = 0; island_idx < snapshot.islands.size(); ++island_idx)
                CHECK(layer.island(island_idx).get_slice() == snapshot.islands[island_idx]);
        }
    };

    DynamicPrintConfig changed_config = print.full_print_config();
    changed_config.set_deserialize_strict(key, changed_value);
    REQUIRE(print.full_print_config().diff(changed_config) == t_config_option_keys {key});
    CHECK(print.apply(model, changed_config) == Print::APPLY_STATUS_INVALIDATED);
    CHECK(print.current_checkpoint() == Steps::Checkpoint::Slicing);
    CHECK(print.should_execute_step(STEP_PRE_PERIMETER));
    CHECK(print.should_execute_step(definition->invalidates_step));
    CHECK_FALSE(print.should_execute_step(STEP_LAYER_HEIGHT));
    CHECK_FALSE(print.should_execute_step(STEP_SLICING));
    CHECK_FALSE(print.should_execute_step(STEP_POST_SLICING));
    check_retained_slices();
    REQUIRE(print.num_print_regions() == 1);
    const ConfigOption *applied_option = region_option ?
        print.print_region(0).config().optptr(key) : print.object(0).config().optptr(key);
    REQUIRE(applied_option != nullptr);
    CHECK(applied_option->serialize() == changed_value);

    print.process();
    CHECK(print.current_checkpoint() == Steps::Checkpoint::Print);
    CHECK_FALSE(print.should_execute_step(STEP_PRE_PERIMETER));
    CHECK_FALSE(print.should_execute_step(STEP_SLICING));
    for (size_t idx = 0; idx < stage_labels.size(); ++idx) {
        CAPTURE(stage_labels[idx]);
        CHECK(stage_counts[idx] == (idx < 3 ? 1 : 2));
    }
    check_retained_slices();
}

TEST_CASE("Former skirt catch-all options resume at their verified first consumers",
          "[plugins][config][checkpoint][invalidation][replay-options]")
{
    using namespace Slic3r;
    using Steps::Checkpoint;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    struct Routing {
        const char *key;
        const char *value;
        slicing_step_t consumer;
        Checkpoint checkpoint;
    };
    // The wipe-tower tool-change plugin reads both temperatures, so their invalidation is tightened
    // from the G-code step to the tower step, which still replays every later step.
    const Routing routing = GENERATE(
        Routing{"gcode_flavor", "klipper", STEP_ORDERING, Checkpoint::Print},
        Routing{"travel_speed", "187", STEP_LAYER_EXTRUSION_EDIT, Checkpoint::Ordering},
        Routing{"travel_speed_z", "13", STEP_GCODE, Checkpoint::Plan},
        Routing{"first_layer_temperature", "205", STEP_WIPETOWER, Checkpoint::Ordering},
        Routing{"temperature", "215", STEP_WIPETOWER, Checkpoint::Ordering},
        Routing{"idle_temperature", "160", STEP_GCODE, Checkpoint::Plan},
        Routing{"ooze_prevention", "1", STEP_GCODE, Checkpoint::Plan},
        Routing{"z_offset", "0.05", STEP_GCODE, Checkpoint::Plan},
        Routing{"skirt_distance", "9", STEP_SKIRT_BRIM, Checkpoint::Slicing});
    CAPTURE(routing.key);
    const ConfigOptionDef *definition = PrintConfigDef::instance().get(routing.key);
    REQUIRE(definition != nullptr);
    CHECK(definition->invalidates_step == routing.consumer);
    CHECK(Steps::checkpoint_before_step(definition->invalidates_step) == routing.checkpoint);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("gcode_flavor", "marlin");
    config.set_deserialize_strict("step_gcode_plugin", "gcode.printing_plan_file_writer");
    config.set_deserialize_strict("gcode_firmware_plugin", "gcode.firmware.marlin2");
    Model model;
    Print print;
    Test::init_print({make_cube(5., 5., 1.)}, print, model, config);
    const std::array<const char *, 5> labels {
        "Creating the layer height", "StepSlicing", "Post-processing slices",
        "Preparing perimeters", "Ordering extrusions"
    };
    std::array<size_t, 5> calls{};
    print.status().set_callback([&](const PrintBase::SlicingStatus &status) {
        if ((status.flags & PrintBase::SlicingStatus::MAIN_STATE) == 0)
            return;
        for (size_t idx = 0; idx < labels.size(); ++idx)
            if (status.main_text == labels[idx])
                ++calls[idx];
    });
    const boost::filesystem::path path = Test::GCode::temporary_gcode_path();
    struct OutputCleanup {
        boost::filesystem::path path;
        ~OutputCleanup() { Test::GCode::remove_output_pair(path); }
    } output_cleanup{path};
    Steps::StepPipeline::run_slice(Orchestrator::instance(), print);
    Steps::StepPipeline::run_gcode(Orchestrator::instance(), print, path.string());
    REQUIRE(print.current_checkpoint() == Checkpoint::GCode);
    REQUIRE(print.has_ordering_checkpoint());
    const Layer *source_layer = &print.object(0).layer(0);
    const std::vector<coord_t> profile = print.object(0).layer_profile();
    const ExPolygons raw_slices = source_layer->region(0).get_raw_slices();
    for (const size_t count : calls)
        REQUIRE(count == 1);

    DynamicPrintConfig changed = print.full_print_config();
    changed.set_deserialize_strict(routing.key, routing.value);
    REQUIRE(print.full_print_config().diff(changed) == t_config_option_keys{routing.key});
    CHECK(print.apply(model, changed) == Print::APPLY_STATUS_INVALIDATED);
    REQUIRE(print.current_checkpoint() == routing.checkpoint);
    const slicing_step_t resume = Steps::checkpoint_resume_step(routing.checkpoint);
    for (const slicing_step_t step : Steps::execution_order()) {
        CAPTURE(step);
        CHECK(print.should_execute_step(step) == !Steps::executes_before(step, resume));
    }
    CHECK_FALSE(print.should_execute_step(STEP_LAYER_HEIGHT));
    CHECK_FALSE(print.should_execute_step(STEP_SLICING));
    CHECK_FALSE(print.should_execute_step(STEP_POST_SLICING));
    CHECK(&print.object(0).layer(0) == source_layer);
    CHECK(print.object(0).layer_profile() == profile);
    CHECK(source_layer->region(0).get_raw_slices() == raw_slices);
    Steps::StepPipeline::run_gcode(Orchestrator::instance(), print, path.string());
    CHECK(print.current_checkpoint() == Checkpoint::GCode);
    for (size_t idx = 0; idx < calls.size(); ++idx) {
        CAPTURE(labels[idx]);
        const size_t expected = idx < 3 ? 1 :
            idx == 3 ? (routing.checkpoint == Checkpoint::Slicing ? 2 : 1) :
                       (routing.checkpoint <= Checkpoint::Print ? 2 : 1);
        CHECK(calls[idx] == expected);
    }
    CHECK(&print.object(0).layer(0) == source_layer);
    CHECK(print.object(0).layer_profile() == profile);
    CHECK(source_layer->region(0).get_raw_slices() == raw_slices);
    CHECK_FALSE(Test::GCode::read_text_file(path).empty());
}

// Read registered option metadata without mutating source owners and compare exact producer/checkpoint routing; missing definitions fail explicitly.
TEST_CASE("The skirt option audit keeps unproven and native-prefix consumers conservative",
          "[plugins][config][checkpoint][invalidation][replay-options]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::array<const char *, 12> kept {
        "complete_objects", "draft_shield", "min_skirt_length", "skirt_brim", "skirt_distance",
        "skirt_distance_from_brim", "skirt_height", "skirt_extrusion_width", "skirts",
        "filament_type", "priming_position", "single_extruder_multi_material"
    };
    for (const char *key : kept) {
        CAPTURE(key);
        const ConfigOptionDef *definition = PrintConfigDef::instance().get(key);
        REQUIRE(definition != nullptr);
        CHECK(definition->invalidates_step == STEP_SKIRT_BRIM);
        CHECK(Steps::checkpoint_before_step(definition->invalidates_step) == Steps::Checkpoint::Slicing);
    }
    // Tower envelopes and controls, and the purge, ramming and filament-swap settings read by
    // the tool-change wipe, replay the ordered plan rather than source slicing.
    for (const char *key : {"filament_max_wipe_tower_speed", "filament_minimal_purge_on_wipe_tower",
         "wiping_volumes_extruders", "wiping_volumes_matrix",
         "cooling_tube_retraction", "cooling_tube_length", "parking_pos_retraction", "extra_loading_move",
         "high_current_on_filament_swap", "filament_loading_speed", "filament_loading_speed_start",
         "filament_unloading_speed", "filament_unloading_speed_start", "filament_cooling_moves",
         "filament_cooling_initial_speed", "filament_cooling_final_speed", "filament_enable_toolchange_temp",
         "filament_toolchange_temp", "filament_enable_toolchange_part_fan", "filament_toolchange_part_fan_speed",
         "filament_use_skinnydip", "filament_use_fast_skinnydip", "filament_skinnydip_distance",
         "filament_dip_insertion_speed", "filament_dip_extraction_speed", "filament_melt_zone_pause",
         "filament_cooling_zone_pause", "filament_toolchange_delay", "filament_ramming_parameters",
         "filament_multitool_ramming", "filament_multitool_ramming_volume", "filament_multitool_ramming_flow",
         "single_extruder_multi_material_priming",
         "wipe_tower_speed", "wipe_tower_wipe_starting_speed", "wipe_tower_extrusion_width",
         "wipe_tower_no_sparse_layers", "wipe_tower_rest_in_middle", "wipe_tower_cone_angle", "wipe_tower_extra_spacing",
         "wipe_tower_bridging", "wipe_tower_extruder_toolchange", "wipe_tower_brim_width",
         "wipe_tower_x", "wipe_tower_y", "wipe_tower_width", "wipe_tower_rotation_angle", "wipe_tower_extruder",
         "wipe_tower_object_x", "wipe_tower_object_y"}) {
        CAPTURE(key);
        const ConfigOptionDef *definition = PrintConfigDef::instance().get(key);
        REQUIRE(definition != nullptr);
        CHECK(definition->invalidates_step == STEP_WIPETOWER);
        CHECK(Steps::checkpoint_before_step(definition->invalidates_step) == Steps::Checkpoint::Ordering);
    }
    // The wipe-into ordering plugin reads the tower presence at STEP_ORDERING, which still replays
    // the tower step and everything after it.
    for (const char *key : {"wipe_tower", "wipe_tower_per_object"}) {
        CAPTURE(key);
        const ConfigOptionDef *definition = PrintConfigDef::instance().get(key);
        REQUIRE(definition != nullptr);
        CHECK(definition->invalidates_step == STEP_ORDERING);
        CHECK(Steps::checkpoint_before_step(definition->invalidates_step) == Steps::Checkpoint::Print);
    }
    // The tool-change unretraction wipe reads the retraction and speed settings of the incoming
    // tool at STEP_LAYER_EXTRUSION_EDIT, so their invalidation is tightened there from the G-code
    // step. The layered speed percentages need solid_infill_speed as their fallback base.
    // filament_diameter keeps the earlier skirt/brim consumer instead.
    for (const char *key : {"retract_restart_wipe_toolchange", "deretract_speed",
         "use_firmware_retraction", "support_material_speed", "solid_infill_speed"}) {
        CAPTURE(key);
        const ConfigOptionDef *definition = PrintConfigDef::instance().get(key);
        REQUIRE(definition != nullptr);
        CHECK(definition->invalidates_step == STEP_LAYER_EXTRUSION_EDIT);
        CHECK(Steps::checkpoint_before_step(definition->invalidates_step) == Steps::Checkpoint::Ordering);
    }
}

// Check a minimal definition/config against borrowed provider metadata without
// changing registration. Resolver counts prove absent/default values skip selection.
TEST_CASE("Active settings require support from the selected exclusive-group plugin",
          "[plugins][config][gcode][setting-support]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const Plugin *marlin = Orchestrator::instance().get_plugin("gcode.firmware.marlin2");
    REQUIRE(marlin != nullptr);
    REQUIRE_FALSE(marlin->get_exclusive_group_label().empty());
    PluginWithoutSettingSupport unsupported(*marlin);
    ConfigDef definitions;
    ConfigOptionDef *definition = definitions.add("single_extruder_multi_material", coBool);
    definition->label = "Material multiplexing";
    definition->required_plugin_group = "gcode_firmware_plugin";
    definition->set_default_value(new ConfigOptionBool(false));
    DynamicConfig config;
    config.set_key_value("single_extruder_multi_material", new ConfigOptionBool(true));
    const Plugin *selected = marlin;
    size_t resolutions = 0;
    const std::function<const Plugin *(const std::string &)> resolver =
        [&selected, &resolutions](const std::string &group_id) -> const Plugin * {
            CHECK(group_id == "gcode_firmware_plugin");
            ++resolutions;
            return selected;
        };

    SECTION("a default value is inactive") {
        config.set_key_value("single_extruder_multi_material", new ConfigOptionBool(false));
        CHECK_FALSE(Steps::unsupported_active_setting(definitions, config, resolver));
        CHECK(resolutions == 0);
    }
    SECTION("an absent value is skipped") {
        config.clear();
        CHECK_FALSE(Steps::unsupported_active_setting(definitions, config, resolver));
        CHECK(resolutions == 0);
    }
    SECTION("an empty requirement is skipped") {
        definition->required_plugin_group.clear();
        CHECK_FALSE(Steps::unsupported_active_setting(definitions, config, resolver));
        CHECK(resolutions == 0);
    }
    SECTION("the selected provider declares the key") {
        CHECK_FALSE(Steps::unsupported_active_setting(definitions, config, resolver));
        CHECK(resolutions == 1);
    }
    SECTION("the selected provider lacks the key") {
        selected = &unsupported;
        const std::optional<std::string> error = Steps::unsupported_active_setting(definitions, config, resolver);
        REQUIRE(error);
        CHECK(error->find("single_extruder_multi_material") != std::string::npos);
        CHECK(error->find("Material multiplexing") != std::string::npos);
        CHECK(error->find(marlin->get_exclusive_group_label()) != std::string::npos);
        CHECK(error->find(marlin->get_name()) != std::string::npos);
        CHECK(error->find(marlin->get_id()) != std::string::npos);
        CHECK(resolutions == 1);
    }
    SECTION("no provider is selected") {
        selected = nullptr;
        const std::optional<std::string> error = Steps::unsupported_active_setting(definitions, config, resolver);
        REQUIRE(error);
        CHECK(error->find("single_extruder_multi_material") != std::string::npos);
        CHECK(error->find("Material multiplexing") != std::string::npos);
        CHECK(error->find("gcode_firmware_plugin") != std::string::npos);
        CHECK(error->find("no plugin is selected") != std::string::npos);
        CHECK(resolutions == 1);
    }
    SECTION("false is active when the default is true") {
        definition->set_default_value(new ConfigOptionBool(true));
        config.set_key_value("single_extruder_multi_material", new ConfigOptionBool(false));
        selected = &unsupported;
        CHECK(Steps::unsupported_active_setting(definitions, config, resolver));
        CHECK(resolutions == 1);
    }
}

// Validate an applied model with an explicit built-in firmware selection, and
// audit that the SEMM definition targets its exclusive group, not a numeric step.
TEST_CASE("Print validation accepts active SEMM with a supporting built-in firmware",
          "[plugins][config][gcode][setting-support][integration]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ConfigOptionDef *definition = PrintConfigDef::instance().get("single_extruder_multi_material");
    REQUIRE(definition != nullptr);
    CHECK(definition->required_plugin_group == "gcode_firmware_plugin");
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("single_extruder_multi_material", "1");
    config.set_deserialize_strict("gcode_firmware_plugin", "gcode.firmware.marlin2");
    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    const Plugin *selected = Steps::selected_active_plugin_from_group(
        Orchestrator::instance(), GCODE_FIRMWARE, "gcode_firmware_plugin", &print.full_print_config());
    REQUIRE(selected != nullptr);
    CHECK(selected->get_id() == "gcode.firmware.marlin2");
    const std::pair<PrintBase::PrintValidationError, std::string> validation = print.validate();
    INFO(validation.second);
    CHECK(validation.first == PrintBase::PrintValidationError::pveNone);
    CHECK(validation.second.empty());
}
