///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include <catch2/catch.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <set>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

// PrintConfig.hpp hash macros call boost::hash_combine without its definition;
// Release builds of this file need it to instantiate the template.
#include <boost/container_hash/hash.hpp>
#include <boost/log/core.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/make_shared.hpp>

#include "plugin_test_helpers.hpp"
#include "gcode_test_helpers.hpp"
#include "plugin_e2e_test_helpers.hpp"
#include "support_grid_test_helpers.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_ordering.h"
#include "test_data.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintObjectRegion.hpp"
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
#include "libslic3r/Config/Preset.hpp"
#include "libslic3r/Config/PresetBundle.hpp"
#include "libslic3r/SLA/SLAPrintConfig.hpp"
#include "libslic3r/Format/3mf.hpp"

#include "libslic3r/Format/AMF.hpp"
#include "libslic3r/Format/BambuConfigExport.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/Semver.hpp"
#include "libslic3r/miniz_extension.hpp"

/*
Tests for plugin-owned temporary configurations.
Graph inverse checks use local point arrays and verify that internal evaluation types
cannot enter persisted settings or public C type conversions.
  Graph inverse test -> create_inverse / inverse_interpolate -> native interpolation
                     -> deserialize / C vector setter -> public-type validation
Serialization checks use options that throw or change length between calls to verify
that C functions report failures and C++ views throw instead of returning empty text.
Call graph for these checks:
  TEST_CASE (serialization failures)

The suite exercises the C ABI as a plugin sees it, then uses the C++ wrappers
to verify buffer ownership. The core SCFG codec is covered separately
by libslic3r tests; this suite verifies the ABI buffer and merge semantics.
Invalidation checks compare each option's earliest consuming step with the
checkpoint used to replay its affected pipeline suffix.
Print-sequence checks compare cloned object plans with native ordinary-layer
plans, exercise effective provider selection and load the obsolete switch through
the supported text and project paths. The STL export catalogue varies each
declared object-builder setting independently.
XY order checks distinguish both directions and stable copy ties, including shared first layers and INI/3MF persistence.
Auxiliary scenes independently enumerate source layers, owner markers and shifted
extrusion endpoints. Explicit model indices distinguish every object sort mode.
The plugins_inactive_builder_import CTest entry runs the isolated import case
with SLIC3R_TEST_WITHOUT_BY_OBJECT so the requested provider is absent when the
startup enum is finalized.
The plugins_inactive_builder_settings entry uses the same isolated startup to
check that optional object settings do not disable other providers or scene defaults.
Setting-support checks compare definition defaults with config values and read
only provider metadata; unregistered copies model providers with no used keys.
Definition nature checks exercise classification transport and its permission
table; the persistence checks write configurations to files, 3MF and AMF archives
and read the text back, to verify that each destination only receives the keys that
the nature of their definition allows. Redefinition diagnostics are captured from
the host log, independently of the returned registration status.
Contact-cone settings round-trip both units through INI and 3MF, including object overrides.
The Prusa checks read Metadata/Slic3r_PE.config inside the 3MF archive.
They distinguish compatible plugin options from SuperSlicer-only options.
Print sequence checks compare cloned extrusion sources and placements with native
plans. Positive object steps are checked by independently assigning each source Z
to an integer interval; placement and wipe-tower validation are checked separately.

Operation notes:
- config_option_serialize :  ThrowingSerializedOption::serialize
- config_option_vector_serialize_at
- slic3r_api::ConfigOption::serialize :  ChangingSerializedOption::serialize

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- Steps::checkpoint_before_step()
        |-- Test::init_print()
        |-- make_cube()
        |-- Steps::StepPipeline::run_slice()
        |-- Steps::StepPipeline::run_gcode()
         `-- print.apply()
    TEST_CASE (print sequence)
        |-- PluginE2E::linear_cases() -> check setting coverage
        |-- Orchestrator::export_gcode_cli() -> export the small STL scene
        |-- build_printing_plan_by_object() -> supply ordinary-layer expectations
        |-- check_sequence_builder() -> compare source coverage and Z intervals
        `-- ConfigBase::load_from_gcode_string_legacy() -> check legacy conversion
*/

namespace {

class ChangingSerializedOption final : public Slic3r::ConfigOptionString
{
public:
    // Set the text length returned by the first serialization call.
    explicit ChangingSerializedOption(size_t first_size) : m_first_size(first_size) {}
    // Return text one byte longer after the first call to simulate a change between the length query and copy.
    std::string serialize() const override
    {
        return std::string(m_first_size + (m_calls++ == 0 ? 0 : 1), 'x');
    }
private:
    size_t m_first_size;
    mutable unsigned int m_calls = 0;
};

class ThrowingSerializedOption final : public Slic3r::ConfigOptionString
{
public:
    // Set the serialization call number that will throw.
    explicit ThrowingSerializedOption(unsigned int fail_on) : ConfigOptionString("value"), m_fail_on(fail_on) {}
    // Throw an integer on the selected call to check that the C API catches exceptions beyond std::exception.
    std::string serialize() const override
    {
        if (++m_calls == m_fail_on) throw 1;
        return value;
    }
private:
    unsigned int m_fail_on;
    mutable unsigned int m_calls = 0;
};

// Check C error returns and buffer clearing, and C++ exceptions for serialization failures and changed lengths.
TEST_CASE("Option serialization failures stay inside the C ABI and reach C++ callers",
          "[plugins][config][c-api]")
{
    ThrowingSerializedOption failing(1);
    const config_option_handle *handle = reinterpret_cast<const config_option_handle *>(&failing);
    char out[] = "unchanged";
    CHECK(config_option_serialize(handle, out, sizeof(out)) == std::numeric_limits<uint32_t>::max());
    CHECK(out[0] == '\0');
    CHECK(config_option_serialize(nullptr, nullptr, 0) == 0);
    Slic3r::ConfigOptionString empty;
    CHECK(config_option_serialize(reinterpret_cast<const config_option_handle *>(&empty), nullptr, 0) == 0);
    for (unsigned int fail_on : {1u, 2u}) {
        ThrowingSerializedOption option(fail_on);
        const slic3r_api::ConfigOption view(reinterpret_cast<const config_option_handle *>(&option));
        CHECK_THROWS_WITH(view.serialize(), "The configuration option could not be serialized.");
    }
    Slic3r::ConfigOptionFloat scalar(1.0);
    Slic3r::ConfigOptionFloats vector{1.0};
    for (Slic3r::ConfigOption *option : {static_cast<Slic3r::ConfigOption *>(&scalar),
                                      static_cast<Slic3r::ConfigOption *>(&vector)}) {
        config_option_handle *mutable_handle = reinterpret_cast<config_option_handle *>(option);
        config_option_set_float(mutable_handle, std::numeric_limits<double>::quiet_NaN(), 0);
        char buffer[] = "unchanged";
        CHECK(config_option_serialize(mutable_handle, buffer, sizeof(buffer)) == std::numeric_limits<uint32_t>::max());
        CHECK(buffer[0] == '\0');
        const slic3r_api::ConfigOption view(mutable_handle);
        CHECK_THROWS_WITH(view.serialize(), "The configuration option could not be serialized.");
    }
    const config_option_vector_handle *vector_handle = config_option_vector_cast(
        reinterpret_cast<const config_option_handle *>(&vector));
    char buffer[] = "unchanged";
    CHECK(config_option_vector_serialize_at(vector_handle, 0, buffer, sizeof(buffer)) == std::numeric_limits<uint32_t>::max());
    CHECK(buffer[0] == '\0');
    CHECK(config_option_vector_serialize_at(nullptr, 0, nullptr, 0) == 0);
    CHECK(config_option_vector_serialize_at(vector_handle, -1, nullptr, 0) == 0);
    CHECK(config_option_vector_serialize_at(vector_handle, 1, nullptr, 0) == 0);
    for (size_t first_size : {size_t(5), size_t(0)}) {
        ChangingSerializedOption option(first_size);
        const slic3r_api::ConfigOption view(reinterpret_cast<const config_option_handle *>(&option));
        CHECK_THROWS_WITH(view.serialize(), "The configuration option changed while it was being serialized.");
    }
}

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

// Test-owned output files of the persistence checks live in one unique directory removed at scope exit.
class ScopedNatureDirectory
{
public:
    // Create one unique directory below the current working directory.
    ScopedNatureDirectory()
        : m_path(boost::filesystem::current_path() / boost::filesystem::unique_path("slic3r-nature-%%%%-%%%%-%%%%"))
    {
        boost::filesystem::create_directories(m_path);
    }

    // Remove the directory without throwing during stack unwinding.
    ~ScopedNatureDirectory()
    {
        boost::system::error_code ignored;
        boost::filesystem::remove_all(m_path, ignored);
    }

    // Return the borrowed path owned by this scope.
    const boost::filesystem::path &path() const { return m_path; }

private:
    boost::filesystem::path m_path;
};

// Preset has a protected constructor; the test builds a standalone printer preset through this derived type.
class NaturePreset final : public Slic3r::Preset
{
public:
    NaturePreset() : Preset(Slic3r::Preset::TYPE_PRINTER, "nature printer") {}
};

// One key with the value written by the persistence checks and the destinations that must contain it.
struct NatureKeyExpectation
{
    const char *key;
    const char *value;
    bool in_local;
    bool in_project;
    bool in_gcode;
};

// Keys of every nature: real keys of the print, printer and project, and the registered test keys of each nature.
const NatureKeyExpectation k_nature_keys[] = {
    {"layer_height", "0.31", true, true, true},
    {"test_config_nature_normal", "11", true, true, true},
    {"printhost_apikey", "local_secret_sentinel", true, false, false},
    {"host_type", "octoprint", true, false, false},
    {"test_config_nature_local", "12", true, false, false},
    {"print_settings_id", "bookkeeping_sentinel", true, true, true},
    {"test_config_nature_bookkeeping", "13", true, true, true},
    {"inherits", "inherits_sentinel", true, true, false},
    {"compatible_printers", "compatible_sentinel", true, true, false},
    {"default_print_profile", "default_profile_sentinel", true, true, false},
    {"test_config_nature_preset_meta", "14", true, true, false},
    {"test_config_nature_volatile", "15", false, false, false},
};

// Fill a configuration with one sentinel value for every key of k_nature_keys.
void set_nature_sentinels(Slic3r::DynamicPrintConfig &config);
// Return whether the text contains the complete line "key = value", with the given line prefix before the key.
bool has_config_line(const std::string &text, const std::string &prefix, const NatureKeyExpectation &expectation);
// Return whether the text has any line for the key, including a line that only has an empty value.
bool has_config_key(const std::string &text, const std::string &prefix, const NatureKeyExpectation &expectation);
// Split a text into its lines, without the line terminator ("\n" or "\r\n").
std::vector<std::string> split_lines(const std::string &text);
// Read one archive entry as text: the named entry, or the first one when no name is given; an absent entry gives an empty string.
std::string read_archive_entry(const boost::filesystem::path &archive_path, const char *entry_name = nullptr);

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

// Check that inactive stored points do not affect endpoint clamps or a single active point.
TEST_CASE("Graph interpolation clamps to the active range", "[plugins][config][graph]")
{
    using Slic3r::GraphData;
    for (const GraphData::GraphType type : {GraphData::LINEAR, GraphData::SQUARE,
                                          GraphData::SQUARE_INVERSE, GraphData::SPLINE}) {
        const GraphData range(1, 3, type, {{0., 90.}, {2., 10.}, {4., 30.}, {6., 80.}});
        CHECK(range.interpolate(-1.) == 10.);
        CHECK(range.interpolate(0.) == 10.);
        CHECK(range.interpolate(2.) == 10.);
        CHECK(range.interpolate(4.) == 30.);
        CHECK(range.interpolate(5.) == 30.);
        CHECK(range.interpolate(7.) == 30.);
        const GraphData single(1, 2, type, range.graph_points);
        for (const double x : {-1., 0., 2., 3., 7.})
            CHECK(single.interpolate(x) == 10.);
    }
}

// Owned host graphs keep copied points alive across owner moves and use host inversion.
TEST_CASE("Plugins own independent graphs with exact square inversion", "[plugins][config][graph][abi]")
{
    CHECK(SLIC3R_PLUGIN_API_CONFIG_OPTION_MAJOR == 3u);
    CHECK(SLIC3R_PLUGIN_API_CONFIG_OPTION_MINOR == 1u);
    std::vector<c_vec2d> points{{0., 0.}, {2., 1.}, {8., 5.}};
    slic3r_api::OwnedGraphData owner(points, RAW_GRAPH_TYPE_SQUARE);
    const slic3r_api::GraphData view = owner.view();
    points[1].x = 99.;
    slic3r_api::OwnedGraphData moved(std::move(owner));
    CHECK(view.point(1).x == 2.);
    CHECK(moved.view().type() == RAW_GRAPH_TYPE_SQUARE);
    CHECK(view.interpolate(3.) == 1.);
    CHECK(view.inverse_interpolate(1.00005) == 8.);
    slic3r_api::OwnedGraphData linear({{0., 0.}, {8., 4.}}, RAW_GRAPH_TYPE_LINEAR);
    CHECK(linear.view().interpolate(4.) == 2.);
    CHECK(linear.view().inverse_interpolate(2.) == 4.);
    slic3r_api::OwnedGraphData single({{0., 4.}}, RAW_GRAPH_TYPE_LINEAR);
    CHECK(single.view().size() == 1);
    CHECK(single.view().inverse_interpolate(3.) == 0.);
    CHECK(graph_data_create(nullptr, 1, RAW_GRAPH_TYPE_LINEAR) == nullptr);
    CHECK(graph_data_create(points.data(), 3, RAW_GRAPH_TYPE_NONE) == nullptr);
    CHECK(graph_data_create(points.data(), 3, RAW_GRAPH_TYPE_LINEAR) == nullptr);
    for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::infinity(),
                                 -std::numeric_limits<double>::infinity()}) {
        const c_vec2d invalid_x{invalid, 0.};
        const c_vec2d invalid_y{0., invalid};
        CHECK(graph_data_create(&invalid_x, 1, RAW_GRAPH_TYPE_LINEAR) == nullptr);
        CHECK(graph_data_create(&invalid_y, 1, RAW_GRAPH_TYPE_LINEAR) == nullptr);
    }
    graph_data_free(nullptr);
}

// Compare interior spline values with a graph containing only the active points.
TEST_CASE("Spline interpolation ignores inactive points when solving the active range",
          "[plugins][config][graph]")
{
    using Slic3r::GraphData;
    const GraphData pair(1, 3, GraphData::SPLINE, {{0., 90.}, {2., 10.}, {4., 30.}, {6., 80.}});
    const GraphData active_pair(0, 2, GraphData::SPLINE, pair.data());
    CHECK(pair.interpolate(3.) == Approx(active_pair.interpolate(3.)));
    const GraphData range(2, 6, GraphData::SPLINE,
                         {{-2., 80.}, {0., 90.}, {2., 10.}, {3., 25.}, {5., 15.}, {8., 40.}, {10., 70.}});
    const GraphData active_range(0, 4, GraphData::SPLINE, range.data());
    for (const double x : {2.25, 2.75, 3.5, 4.5, 5.5, 7.5})
        CHECK(range.interpolate(x) == Approx(active_range.interpolate(x)));
}

// Check inverse graph evaluation and its exclusion from persistent and C API graph types.
TEST_CASE("Graph inverses preserve point thresholds without exposing the internal step type",
          "[plugins][config][graph]")
{
    using Slic3r::GraphData;
    const GraphData square(0, 3, GraphData::SQUARE, {{0., 0.}, {1., 1.}, {2., 2.}});
    const GraphData inverse = square.create_inverse();
    CHECK(inverse.type == GraphData::SQUARE_INVERSE);
    for (const double y : {0., 1., 2.})
        CHECK(square.inverse_interpolate(y) == y);
    CHECK(square.inverse_interpolate(0.5) == 1.);
    CHECK(square.inverse_interpolate(1.5) == 2.);
    CHECK(square.inverse_interpolate(0.99995) == 1.);
    CHECK(square.inverse_interpolate(1.00005) == 2.);
    CHECK(square.inverse_interpolate(-1.) == 0.);
    CHECK(square.inverse_interpolate(3.) == 2.);
    CHECK(square.interpolate(0.5) == 0.);
    CHECK(graph_inverse_interpolate(reinterpret_cast<const graph_data_handle *>(&square), 0.5) == 1.);
    CHECK(graph_data_type(reinterpret_cast<const graph_data_handle *>(&inverse)) == RAW_GRAPH_TYPE_NONE);

    // Double inversion checks stored data and type only, not exact inversion of a spline curve.
    for (const GraphData::GraphType type : {GraphData::SQUARE, GraphData::SQUARE_INVERSE,
                                          GraphData::LINEAR, GraphData::SPLINE}) {
        const GraphData original(1, 3, type, {{0., 0.}, {2., 1.}, {4., 3.}, {6., 5.}});
        const GraphData twice = original.create_inverse().create_inverse();
        CHECK(twice.type == original.type);
        CHECK(twice.graph_points == original.graph_points);
        CHECK(twice.begin_idx == original.begin_idx);
        CHECK(twice.end_idx == original.end_idx);
    }
    const GraphData linear(0, 3, GraphData::LINEAR, {{0., 0.}, {2., 1.}, {4., 3.}});
    CHECK(linear.inverse_interpolate(0.5) == Approx(1.));
    CHECK(linear.inverse_interpolate(2.) == Approx(3.));
    CHECK(linear.inverse_interpolate(-1.) == 0.);
    CHECK(linear.inverse_interpolate(4.) == 4.);

    const GraphData range(1, 3, GraphData::SQUARE, {{0., 0.}, {2., 1.}, {4., 3.}, {6., 5.}});
    CHECK(range.inverse_interpolate(0.) == 2.);
    CHECK(range.inverse_interpolate(1.) == 2.);
    CHECK(range.inverse_interpolate(2.) == 4.);
    CHECK(range.inverse_interpolate(3.) == 4.);
    CHECK(range.inverse_interpolate(5.) == 4.);
    const GraphData single(1, 2, GraphData::SQUARE, {{0., 0.}, {2., 1.}, {4., 3.}});
    for (const double y : {-1., 1., 5.})
        CHECK(single.inverse_interpolate(y) == 2.);
    const GraphData point(0, 1, GraphData::SQUARE, {{2., 1.}});
    for (const double y : {-1., 1., 5.})
        CHECK(point.inverse_interpolate(y) == 2.);
    CHECK(GraphData().inverse_interpolate(0.) == 1.);

    // Decreasing Y is not repaired; this lower plateau selects the first endpoint at its exact Y.
    const GraphData decreasing(0, 3, GraphData::SQUARE, {{0., 2.}, {1., 1.}, {2., 0.}});
    CHECK_FALSE(decreasing.create_inverse().validate());
    const GraphData plateau(0, 3, GraphData::SQUARE, {{0., 0.}, {1., 0.}, {2., 2.}});
    CHECK(plateau.create_inverse().validate());
    CHECK(plateau.inverse_interpolate(0.) == 0.);
    CHECK(plateau.inverse_interpolate(1.) == 2.);

    CHECK_THROWS_AS(inverse.serialize(), std::invalid_argument);
    const std::string hidden = "0:3:" + std::to_string(int(GraphData::SQUARE_INVERSE)) + ":0x0:1x1:2x2";
    GraphData decoded;
    CHECK_FALSE(decoded.deserialize(hidden));
    CHECK_FALSE(decoded.deserialize("0:3:3:0x0:1x1:2x2"));
    Slic3r::ConfigOptionGraph option(square);
    config_option_handle *handle = reinterpret_cast<config_option_handle *>(&option);
    CHECK(config_option_deserialize(handle, hidden.c_str(), 0) == 0);
    CHECK(option.value == square);
    Slic3r::ConfigOptionGraphs options({square});
    config_option_vector_handle *vector_handle = config_option_vector_cast_mutable(
        reinterpret_cast<config_option_handle *>(&options));
    REQUIRE(vector_handle != nullptr);
    CHECK(config_option_vector_deserialize_at(vector_handle, 0, hidden.c_str()) == 0);
    CHECK(options.get_values().front() == square);
    config_option_handle *options_handle = reinterpret_cast<config_option_handle *>(&options);
    const std::string original_options = options.serialize();
    const std::string mixed = square.serialize() + "," + hidden;
    for (const int32_t append : {0, 1}) {
        CHECK(config_option_deserialize(options_handle, hidden.c_str(), append) == 0);
        CHECK(options.serialize() == original_options);
        CHECK(config_option_deserialize(options_handle, mixed.c_str(), append) == 0);
        CHECK(options.serialize() == original_options);
    }
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

// Query borrowed definitions and verify literal withdrawal flags through native and C snapshots; missing definitions fail assertions without ownership changes.
TEST_CASE("Configuration definition natures round-trip through the C API",
          "[plugins][config][definition][nature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CHECK(Slic3r::Test::Plugins::plugin_test_roundtrip_nature_result == 0);
    const orchestrator_handle *handle = reinterpret_cast<const orchestrator_handle *>(&Slic3r::Orchestrator::instance());
    const char *keys[] = {
        "test_config_nature_normal", "test_config_nature_local", "test_config_nature_bookkeeping",
        "test_config_nature_preset_meta", "test_config_nature_volatile", "test_config_nature_combined"};
    CHECK(raw_config_option_def_init().nature == 0);
    CHECK(static_cast<uint32_t>(Slic3r::ConfigOptionDef{}.nature) == 0);
    const uint32_t raw_values[] = {RAW_CONFIG_NATURE_NORMAL, RAW_CONFIG_NATURE_LOCAL,
        RAW_CONFIG_NATURE_BOOKKEEPING, RAW_CONFIG_NATURE_PRESET_META, RAW_CONFIG_NATURE_VOLATILE, 10};
    const Slic3r::ConfigOptionNature native_values[] = {Slic3r::ConfigOptionNature::Normal, Slic3r::ConfigOptionNature::Local,
        Slic3r::ConfigOptionNature::Bookkeeping, Slic3r::ConfigOptionNature::PresetMeta, Slic3r::ConfigOptionNature::Volatile,
        static_cast<Slic3r::ConfigOptionNature>(10)};
    const uint32_t expected_values[] = {0, 6, 8, 4, 15, 10};
    for (size_t index = 0; index < 6; ++index) {
        const uint32_t expected = expected_values[index];
        CAPTURE(expected);
        CHECK(raw_values[index] == expected);
        CHECK(static_cast<uint32_t>(native_values[index]) == expected);
        raw_config_option_def raw = raw_config_option_def_init();
        REQUIRE(orchestrator_get_option_def(handle, keys[index], &raw) == 1);
        CHECK(raw.nature == expected);
        const Slic3r::ConfigOptionDef *native = Slic3r::PrintConfigDef::instance().get(keys[index]);
        REQUIRE(native != nullptr);
        CHECK(static_cast<uint32_t>(native->nature) == expected);
    }
}

// Inspect saved registration results and catalogue membership for unknown bits and a valid unnamed combination; incorrect acceptance or rejection fails assertions without catalogue mutation.
TEST_CASE("Configuration definitions reject an invalid nature before finalization",
          "[plugins][config][definition][nature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CHECK(Slic3r::Test::Plugins::plugin_test_invalid_nature_result == 2);
    CHECK(Slic3r::PrintConfigDef::instance().get("test_config_nature_invalid") == nullptr);
    CHECK(Slic3r::Test::Plugins::plugin_test_high_invalid_nature_result == 2);
    CHECK(Slic3r::PrintConfigDef::instance().get("test_config_nature_high_invalid") == nullptr);
    CHECK(Slic3r::Test::Plugins::plugin_test_combined_nature_result == 0);
    CHECK(Slic3r::PrintConfigDef::instance().get("test_config_nature_valid_unnamed") != nullptr);
}

// Capture application logging for borrowed UTF-8 input and null arguments without changing package reports or settings.
// Remove the test-owned output collector before assertions. No plugin-library logger participates.
TEST_CASE("Plugin logging uses the application's configured log core", "[plugins][logging]")
{
    typedef boost::log::sinks::synchronous_sink<boost::log::sinks::text_ostream_backend> LogSink;
    const boost::shared_ptr<std::ostringstream> messages = boost::make_shared<std::ostringstream>();
    const boost::shared_ptr<LogSink> sink = boost::make_shared<LogSink>();
    sink->locked_backend()->add_stream(messages);
    boost::log::core::get()->add_sink(sink);
    orchestrator_send_log(nullptr, RAW_PLUGIN_LOG_ERROR, "Host log bridge UTF-8: \xc3\xa9");
    orchestrator_send_log(nullptr, RAW_PLUGIN_LOG_ERROR, nullptr);
    orchestrator_send_log(nullptr, static_cast<raw_plugin_log_level>(99), "Invalid severity must not log");
    sink->flush();
    boost::log::core::get()->remove_sink(sink);
    CHECK(messages->str().find("Host log bridge UTF-8: \xc3\xa9") != std::string::npos);
    CHECK(messages->str().find("Invalid severity must not log") == std::string::npos);
    CHECK(RAW_PLUGIN_LOG_FATAL == 0);
    CHECK(RAW_PLUGIN_LOG_ERROR == 1);
    CHECK(RAW_PLUGIN_LOG_WARNING == 2);
    CHECK(RAW_PLUGIN_LOG_INFO == 3);
    CHECK(RAW_PLUGIN_LOG_DEBUG == 4);
    CHECK(RAW_PLUGIN_LOG_TRACE == 5);
}

// Redeclare an existing borrowed snapshot with only nature changed, capture the host diagnostic, and assert rejection without replacing the owned definition.
TEST_CASE("Configuration redefinitions reject a different nature",
          "[plugins][config][definition][nature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&Slic3r::Orchestrator::instance());
    raw_config_option_def raw = raw_config_option_def_init();
    REQUIRE(orchestrator_get_option_def(handle, "test_config_nature_normal", &raw) == 1);
    raw.nature = 1;
    typedef boost::log::sinks::synchronous_sink<boost::log::sinks::text_ostream_backend> LogSink;
    const boost::shared_ptr<std::ostringstream> messages = boost::make_shared<std::ostringstream>();
    const boost::shared_ptr<LogSink> sink = boost::make_shared<LogSink>();
    sink->locked_backend()->add_stream(messages);
    boost::log::core::get()->add_sink(sink);
    const option_def_error_code result = orchestrator_create_option_def(handle, &raw);
    sink->flush();
    boost::log::core::get()->remove_sink(sink);
    CHECK(result == 1);
    CHECK(messages->str().find("already defined with incompatible nature.") != std::string::npos);
    REQUIRE(orchestrator_get_option_def(handle, "test_config_nature_normal", &raw) == 1);
    CHECK(raw.nature == 0);
}

// Evaluate all 16 withdrawal combinations against literal permissions; mismatches fail assertions without configuration effects or ownership changes.
TEST_CASE("Configuration nature permissions match all persistence targets",
          "[plugins][config][definition][nature]")
{
    const struct NatureExpectation { uint8_t nature; bool writes[3]; bool comparable; } expectations[] = {
        {0,  {true,  true,  true},  true},
        {1,  {false, true,  true},  true},
        {2,  {true,  false, true},  true},
        {3,  {false, false, true},  true},
        {4,  {true,  true,  false}, true},
        {5,  {false, true,  false}, true},
        {6,  {true,  false, false}, true},
        {7,  {false, false, false}, true},
        {8,  {true,  true,  true},  false},
        {9,  {false, true,  true},  false},
        {10, {true,  false, true},  false},
        {11, {false, false, true},  false},
        {12, {true,  true,  false}, false},
        {13, {false, true,  false}, false},
        {14, {true,  false, false}, false},
        {15, {false, false, false}, false}};
    const Slic3r::ConfigWriteTarget targets[] = {Slic3r::ConfigWriteTarget::Local, Slic3r::ConfigWriteTarget::Project,
        Slic3r::ConfigWriteTarget::GCode};
    for (const NatureExpectation &expected : expectations) {
        CAPTURE(expected.nature);
        const Slic3r::ConfigOptionNature nature = static_cast<Slic3r::ConfigOptionNature>(expected.nature);
        for (size_t target = 0; target < 3; ++target) {
            CAPTURE(target);
            CHECK(Slic3r::config_nature_writes(nature, targets[target]) == expected.writes[target]);
        }
        CHECK(Slic3r::config_nature_comparable(nature) == expected.comparable);
    }
}

// Query borrowed snapshots and assert normalized scalar fields and copied metadata; missing output is tested as a zeroed record before any pointer access.
TEST_CASE("Configuration definitions expose normalized plugin metadata",
          "[plugins][config][definition][nature]")
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
    CHECK(raw.nature == 8);
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

// Query through the borrowed orchestrator and retain owning views across subsequent queries; require successful lookups before asserting detached metadata.
TEST_CASE("C++ configuration definition views own their copied data",
          "[plugins][config][definition][cpp][nature]")
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
    CHECK(definition->nature == 8);
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

TEST_CASE("Print and object expose their default region configurations", "[plugins][config][default-region]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("perimeters", "3");
    // An object-level override changes the object's default region, not the print's.
    Model model;
    ModelObject *model_object = model.add_object();
    model_object->add_volume(make_cube(20., 20., 20.));
    model_object->config.set_deserialize_strict("perimeters", "5");
    model_object->add_instance();
    model.center_instances_around_point({100., 100.});
    model_object->ensure_on_bed();
    Print print;
    print.auto_assign_extruders(model_object);
    print.apply(model, config);
    REQUIRE(print.object(0).num_printing_regions() > 0);

    const print_handle *print_c = reinterpret_cast<const print_handle *>(&print);
    const auto perimeters = [](const slic3r_api::Config &region_config) {
        REQUIRE(region_config.valid());
        return region_config.get("perimeters").get_int();
    };
    REQUIRE(print.auxiliary_object() == nullptr);
    CHECK(perimeters(slic3r_api::Config(print_get_default_region_config(print_c))) == 3);
    CHECK(print_get_default_region_config(nullptr) == nullptr);

    const slic3r_api::Print print_view(print_c);
    CHECK(print_view.default_region_config().handle() == print_get_default_region_config(print_c));
    const object_handle *object_c = reinterpret_cast<const object_handle *>(&print.object(0));
    const slic3r_api::Object object(object_c);
    CHECK(perimeters(object.default_region_config(print_view)) == 5);
    CHECK(print_resolve_default_region_config(print_c, nullptr) == print_get_default_region_config(print_c));
    CHECK(print_resolve_default_region_config(print_c, object_c) == object.default_region_config(print_view).handle());
    CHECK(print_resolve_default_region_config(nullptr, object_c) == nullptr);
    CHECK(print_resolve_default_region_config(nullptr, nullptr) == nullptr);
    // Parallel readers must not manufacture the auxiliary object as a side effect.
    const config_handle *expected_default = print_get_default_region_config(print_c);
    const config_handle *expected_object_region = print_resolve_default_region_config(print_c, object_c);
    const config_handle *expected_object_default = print_get_default_object_config(print_c);
    std::atomic<bool> same_default{true};
    std::vector<std::thread> readers;
    for (size_t idx = 0; idx < 4; ++idx)
        readers.emplace_back([print_c, object_c, expected_default, expected_object_region,
                              expected_object_default, &same_default]() {
            for (size_t iteration = 0; iteration < 64; ++iteration)
                if (print_get_default_region_config(print_c) != expected_default ||
                    print_resolve_default_region_config(print_c, nullptr) != expected_default ||
                    print_resolve_default_region_config(print_c, object_c) != expected_object_region ||
                    print_get_default_object_config(print_c) != expected_object_default)
                    same_default = false;
        });
    for (std::thread &reader : readers)
        reader.join();
    CHECK(same_default.load());
    CHECK(print.auxiliary_object() == nullptr);
    // The auxiliary object's first region is the print's default region.
    const slic3r_api::Object auxiliary(reinterpret_cast<const object_handle *>(&print.mutable_auxiliary_object()));
    CHECK(perimeters(auxiliary.default_region_config(print_view)) == 3);
    CHECK(print_resolve_default_region_config(print_c, auxiliary.handle()) ==
          ApiHost::to_config_handle(&print.auxiliary_object()->default_region_config(print.default_region_config())));

    // Use the auxiliary-object constructor before any region has been published.
    class RegionlessObject final : public PrintObject {
    public:
        // Own an empty region list but borrow the same Print and preset defaults.
        explicit RegionlessObject(Print &owner) :
            PrintObject(&owner, Vec3crd::Zero(), std::make_shared<PrintObjectRegions>()) {}
    };
    const RegionlessObject regionless(print);
    REQUIRE(regionless.num_printing_regions() == 0);
    const object_handle *regionless_c = reinterpret_cast<const object_handle *>(&regionless);
    CHECK(print_resolve_default_region_config(print_c, regionless_c) == expected_default);
    CHECK(slic3r_api::Object(regionless_c).default_region_config(print_view).handle() == expected_default);

    // Reapplying the same Print must replace fallback values, not retain a region
    // from the previous object or confuse a volume override with the object default.
    model_object->config.set_deserialize_strict("perimeters", "7");
    print.apply(model, config);
    const slic3r_api::Object reapplied(reinterpret_cast<const object_handle *>(&print.object(0)));
    CHECK(perimeters(reapplied.default_region_config(print_view)) == 7);
    CHECK(perimeters(print_view.default_region_config()) == 3);
    model_object->volumes.front()->config.set_deserialize_strict("perimeters", "9");
    print.apply(model, config);
    const slic3r_api::Object with_volume_override(reinterpret_cast<const object_handle *>(&print.object(0)));
    CHECK(perimeters(with_volume_override.default_region_config(print_view)) == 7);
    bool found_volume_override = false;
    for (const PrintRegion &region : print.object(0).all_regions())
        found_volume_override |= perimeters(slic3r_api::Config(ApiHost::to_config_handle(&region.config()))) == 9;
    CHECK(found_volume_override);
}

TEST_CASE("Print exposes preset object defaults without creating a global tower owner",
          "[plugins][config][default-object]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("wipe_tower_x", "42");
    config.set_deserialize_strict("wipe_tower_width", "23");
    Model model;
    ModelObject *model_object = model.add_object();
    model_object->add_volume(make_cube(20., 20., 20.));
    model_object->config.set_deserialize_strict("wipe_tower_x", "7");
    model_object->add_instance();
    model.center_instances_around_point({100., 100.});
    model_object->ensure_on_bed();
    Print print;
    print.auto_assign_extruders(model_object);
    print.apply(model, config);

    const print_handle *print_c = reinterpret_cast<const print_handle *>(&print);
    const slic3r_api::Print print_view(print_c);
    REQUIRE(print.auxiliary_object() == nullptr);
    const slic3r_api::Config defaults(print_get_default_object_config(print_c));
    REQUIRE(defaults.valid());
    CHECK(defaults.handle() == ApiHost::to_config_handle(&print.default_object_config()));
    CHECK(defaults.get("wipe_tower_x").get_float() == 42.);
    CHECK(defaults.get("wipe_tower_width").get_float() == 23.);
    CHECK(print_get_default_object_config(nullptr) == nullptr);
    CHECK(print_view.default_object_config().handle() == defaults.handle());
    CHECK(print_view.object(0).config().get("wipe_tower_x").get_float() == 7.);
    CHECK(print.auxiliary_object() == nullptr);

    // Global tower geometry uses preset object settings, not a model override.
    const slic3r_api::Object tower_owner(reinterpret_cast<const object_handle *>(&print.mutable_auxiliary_object()));
    CHECK(tower_owner.config().get("wipe_tower_x").get_float() == 42.);
    CHECK(tower_owner.config().get("wipe_tower_width").get_float() == 23.);
    config.set_deserialize_strict("wipe_tower_x", "64");
    print.apply(model, config);
    CHECK(print_view.default_object_config().get("wipe_tower_x").get_float() == 64.);
    CHECK(print_view.object(0).config().get("wipe_tower_x").get_float() == 7.);
    const slic3r_api::Object reapplied_tower(reinterpret_cast<const object_handle *>(&print.mutable_auxiliary_object()));
    CHECK(reapplied_tower.config().get("wipe_tower_x").get_float() == 64.);
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
    // The wipe-tower plugin reads first_layer_temperature and temperature at STEP_WIPETOWER;
    // the temperature plugins read idle_temperature and ooze_prevention at STEP_LAYER_EXTRUSION_EDIT,
    // so each invalidation resumes at its first consumer and replays every later step.
    const Routing routing = GENERATE(
        Routing{"gcode_flavor", "klipper", STEP_ORDERING, Checkpoint::Print},
        Routing{"travel_speed", "187", STEP_LAYER_EXTRUSION_EDIT, Checkpoint::Ordering},
        Routing{"travel_speed_z", "13", STEP_GCODE, Checkpoint::Plan},
        Routing{"first_layer_temperature", "205", STEP_WIPETOWER, Checkpoint::Ordering},
        Routing{"temperature", "215", STEP_WIPETOWER, Checkpoint::Ordering},
        Routing{"idle_temperature", "160", STEP_LAYER_EXTRUSION_EDIT, Checkpoint::Ordering},
        Routing{"ooze_prevention", "1", STEP_LAYER_EXTRUSION_EDIT, Checkpoint::Ordering},
        Routing{"z_offset", "0.05", STEP_GCODE, Checkpoint::Plan},
        Routing{"print_plan_builder", "ordering.plan_builder.by_object", STEP_SKIRT_BRIM, Checkpoint::Slicing},
        Routing{"skirt_distance", "9", STEP_SKIRT_BRIM, Checkpoint::Slicing});
    CAPTURE(routing.key);
    const ConfigOptionDef *definition = PrintConfigDef::instance().get(routing.key);
    REQUIRE(definition != nullptr);
    CHECK(definition->invalidates_step == routing.consumer);
    CHECK(Steps::checkpoint_before_step(definition->invalidates_step) == routing.checkpoint);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("gcode_flavor", "marlin2");
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
        // The export is queued by an explicit request only, never by a replay.
        CHECK(print.should_execute_step(step) ==
              (step != STEP_EXPORT && !Steps::executes_before(step, resume)));
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
        "print_plan_builder", "draft_shield", "min_skirt_length", "skirt_brim", "skirt_distance",
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

// The fan timing options reshape the extrusion tree at the layer edit step. fan_speedup_time and
// fan_kickstart require a fan timing plugin, so a non-default value without one is refused;
// fan_speedup_overhangs only restricts the anticipation and requires none.
TEST_CASE("Fan timing options invalidate the layer edit and require a fan timing plugin",
          "[plugins][config][setting-support][fan-policy]")
{
    using namespace Slic3r;
    const PrintConfigDef &definitions = PrintConfigDef::instance();
    for (const char *key : {"fan_speedup_time", "fan_speedup_overhangs", "fan_kickstart"}) {
        CAPTURE(key);
        const ConfigOptionDef *definition = definitions.get(key);
        REQUIRE(definition != nullptr);
        CHECK(definition->invalidates_step == STEP_LAYER_EXTRUSION_EDIT);
    }
    CHECK(definitions.get("fan_speedup_time")->required_plugin_group == "fan_temporal_plugin");
    CHECK(definitions.get("fan_kickstart")->required_plugin_group == "fan_temporal_plugin");
    CHECK(definitions.get("fan_speedup_overhangs")->required_plugin_group.empty());

    const std::function<const Plugin *(const std::string &)> no_provider =
        [](const std::string &group_id) -> const Plugin * {
            CHECK(group_id == "fan_temporal_plugin");
            return nullptr;
        };
    DynamicConfig config;
    config.set_key_value("fan_kickstart", new ConfigOptionFloat(2.));
    const std::optional<std::string> error = Steps::unsupported_active_setting(Orchestrator::instance(), definitions, config, no_provider);
    REQUIRE(error);
    CHECK(error->find("fan_kickstart") != std::string::npos);
    CHECK(error->find("fan_temporal_plugin") != std::string::npos);

    config.set_key_value("fan_kickstart", new ConfigOptionFloat(0.));
    CHECK_FALSE(Steps::unsupported_active_setting(Orchestrator::instance(), definitions, config, no_provider));
}

// Check a minimal definition/config against borrowed provider metadata without
// changing registration. Resolver counts prove absent/default values skip selection.
TEST_CASE("Active settings require support from the selected exclusive-group plugin",
          "[plugins][config][gcode][setting-support]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const Plugin *marlin = Orchestrator::instance().get_plugin("marlin2");
    REQUIRE(marlin != nullptr);
    const Orchestrator::ExclusiveGroupDefinition *firmware_definition = Orchestrator::instance().exclusive_group_definition(
        GCODE_FIRMWARE, "gcode_flavor");
    REQUIRE(firmware_definition != nullptr);
    REQUIRE_FALSE(firmware_definition->label.empty());
    PluginWithoutSettingSupport unsupported(*marlin);
    ConfigDef definitions;
    ConfigOptionDef *definition = definitions.add("single_extruder_multi_material", coBool);
    definition->label = "Material multiplexing";
    definition->required_plugin_group = "gcode_flavor";
    definition->set_default_value(new ConfigOptionBool(false));
    DynamicConfig config;
    config.set_key_value("single_extruder_multi_material", new ConfigOptionBool(true));
    const Plugin *selected = marlin;
    size_t resolutions = 0;
    const std::function<const Plugin *(const std::string &)> resolver =
        [&selected, &resolutions](const std::string &group_id) -> const Plugin * {
            CHECK(group_id == "gcode_flavor");
            ++resolutions;
            return selected;
        };

    SECTION("a default value is inactive") {
        config.set_key_value("single_extruder_multi_material", new ConfigOptionBool(false));
        CHECK_FALSE(Steps::unsupported_active_setting(Orchestrator::instance(), definitions, config, resolver));
        CHECK(resolutions == 0);
    }
    SECTION("an absent value is skipped") {
        config.clear();
        CHECK_FALSE(Steps::unsupported_active_setting(Orchestrator::instance(), definitions, config, resolver));
        CHECK(resolutions == 0);
    }
    SECTION("an empty requirement is skipped") {
        definition->required_plugin_group.clear();
        CHECK_FALSE(Steps::unsupported_active_setting(Orchestrator::instance(), definitions, config, resolver));
        CHECK(resolutions == 0);
    }
    SECTION("the selected provider declares the key") {
        CHECK_FALSE(Steps::unsupported_active_setting(Orchestrator::instance(), definitions, config, resolver));
        CHECK(resolutions == 1);
    }
    SECTION("the selected provider lacks the key") {
        selected = &unsupported;
        const std::optional<std::string> error = Steps::unsupported_active_setting(Orchestrator::instance(), definitions, config, resolver);
        REQUIRE(error);
        CHECK(error->find("single_extruder_multi_material") != std::string::npos);
        CHECK(error->find("Material multiplexing") != std::string::npos);
        CHECK(error->find(firmware_definition->label) != std::string::npos);
        CHECK(error->find(marlin->get_name()) != std::string::npos);
        CHECK(error->find(marlin->get_id()) != std::string::npos);
        CHECK(resolutions == 1);
    }
    SECTION("no provider is selected") {
        selected = nullptr;
        const std::optional<std::string> error = Steps::unsupported_active_setting(Orchestrator::instance(), definitions, config, resolver);
        REQUIRE(error);
        CHECK(error->find("single_extruder_multi_material") != std::string::npos);
        CHECK(error->find("Material multiplexing") != std::string::npos);
        CHECK(error->find("gcode_flavor") != std::string::npos);
        CHECK(error->find("no plugin is selected") != std::string::npos);
        CHECK(resolutions == 1);
    }
    SECTION("false is active when the default is true") {
        definition->set_default_value(new ConfigOptionBool(true));
        config.set_key_value("single_extruder_multi_material", new ConfigOptionBool(false));
        selected = &unsupported;
        CHECK(Steps::unsupported_active_setting(Orchestrator::instance(), definitions, config, resolver));
        CHECK(resolutions == 1);
    }
}

// Validate an applied model with a non-default built-in firmware selection, and
// audit that the SEMM definition targets its exclusive group, not a numeric step.
TEST_CASE("Print validation accepts active SEMM with a supporting built-in firmware",
          "[plugins][config][gcode][setting-support][integration]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ConfigOptionDef *definition = PrintConfigDef::instance().get("single_extruder_multi_material");
    REQUIRE(definition != nullptr);
    CHECK(definition->required_plugin_group == "gcode_flavor");
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("single_extruder_multi_material", "1");
    config.set_deserialize_strict("gcode_flavor", "klipper");
    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    const Plugin *selected = Steps::selected_active_plugin_from_group(
        Orchestrator::instance(), GCODE_FIRMWARE, "gcode_flavor", &print.full_print_config());
    REQUIRE(selected != nullptr);
    CHECK(selected->get_id() == "klipper");
    const std::pair<PrintBase::PrintValidationError, std::string> validation = print.validate();
    INFO(validation.second);
    CHECK(validation.first == PrintBase::PrintValidationError::pveNone);
    CHECK(validation.second.empty());
}

namespace {

// Fill a configuration with one sentinel value for every key of k_nature_keys.
void set_nature_sentinels(Slic3r::DynamicPrintConfig &config)
{
    for (const NatureKeyExpectation &expectation : k_nature_keys)
        config.set_deserialize_strict(expectation.key, expectation.value);
}

// Return whether the text contains the complete line "key = value", with the given line prefix before the key.
bool has_config_line(const std::string &text, const std::string &prefix, const NatureKeyExpectation &expectation)
{
    const std::string wanted = prefix + expectation.key + " = " + expectation.value;
    const std::vector<std::string> lines = split_lines(text);
    return std::find(lines.begin(), lines.end(), wanted) != lines.end();
}

// Return whether the text has any line for the key, including a line that only has an empty value.
bool has_config_key(const std::string &text, const std::string &prefix, const NatureKeyExpectation &expectation)
{
    const std::string start = prefix + expectation.key + " =";
    for (const std::string &line : split_lines(text))
        if (line == start || line.compare(0, start.size() + 1, start + " ") == 0)
            return true;
    return false;
}

// Split a text into its lines, without the line terminator ("\n" or "\r\n").
std::vector<std::string> split_lines(const std::string &text)
{
    std::vector<std::string> lines;
    std::istringstream stream(text);
    std::string line;
    while (std::getline(stream, line)) {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        lines.push_back(line);
    }
    return lines;
}

// Read one archive entry as text: the named entry, or the first one when no name is given; an absent entry gives an empty string.
std::string read_archive_entry(const boost::filesystem::path &archive_path, const char *entry_name)
{
    mz_zip_archive archive;
    mz_zip_zero_struct(&archive);
    REQUIRE(Slic3r::open_zip_reader(&archive, archive_path.string()));
    size_t size = 0;
    void *data = entry_name == nullptr ? mz_zip_reader_extract_to_heap(&archive, 0, &size, 0) :
                                         mz_zip_reader_extract_file_to_heap(&archive, entry_name, &size, 0);
    std::string text = data == nullptr ? std::string() : std::string(static_cast<const char *>(data), size);
    mz_free(data);
    Slic3r::close_zip_reader(&archive);
    return text;
}

} // namespace

// Compare the nature stored in the definition of every classified key with a literal table, and a few ordinary witnesses.
TEST_CASE("Persistent settings have the literal nature of their classification",
          "[plugins][config][nature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    using Slic3r::ConfigOptionNature;
    const std::pair<const char *, ConfigOptionNature> natures[] = {
        {"print_host", ConfigOptionNature::Local}, {"printhost_apikey", ConfigOptionNature::Local},
        {"printhost_cafile", ConfigOptionNature::Local}, {"printhost_port", ConfigOptionNature::Local},
        {"printhost_client_cert", ConfigOptionNature::Local},
        {"printhost_client_cert_password", ConfigOptionNature::Local},
        {"printhost_user", ConfigOptionNature::Local}, {"printhost_password", ConfigOptionNature::Local},
        {"printhost_ssl_ignore_revoke", ConfigOptionNature::Local},
        {"printhost_authorization_type", ConfigOptionNature::Local}, {"host_type", ConfigOptionNature::Local},
        {"preset_name", ConfigOptionNature::Local}, {"preset_names", ConfigOptionNature::Local},
        {"print_version", ConfigOptionNature::Bookkeeping}, {"print_settings_id", ConfigOptionNature::Bookkeeping},
        {"filament_settings_id", ConfigOptionNature::Bookkeeping},
        {"printer_settings_id", ConfigOptionNature::Bookkeeping},
        {"physical_printer_settings_id", ConfigOptionNature::Bookkeeping},
        {"sla_print_settings_id", ConfigOptionNature::Bookkeeping},
        {"sla_material_settings_id", ConfigOptionNature::Bookkeeping},
        {"print_settings_modified", ConfigOptionNature::Bookkeeping},
        {"filament_settings_modified", ConfigOptionNature::Bookkeeping},
        {"printer_settings_modified", ConfigOptionNature::Bookkeeping},
        {"sla_print_settings_modified", ConfigOptionNature::Bookkeeping},
        {"sla_material_settings_modified", ConfigOptionNature::Bookkeeping},
        {"inherits", ConfigOptionNature::PresetMeta}, {"compatible_printers", ConfigOptionNature::PresetMeta},
        {"compatible_printers_condition", ConfigOptionNature::PresetMeta},
        {"compatible_prints", ConfigOptionNature::PresetMeta},
        {"compatible_prints_condition", ConfigOptionNature::PresetMeta},
        {"default_print_profile", ConfigOptionNature::PresetMeta},
        {"default_filament_profile", ConfigOptionNature::PresetMeta},
        {"default_sla_print_profile", ConfigOptionNature::PresetMeta},
        {"default_sla_material_profile", ConfigOptionNature::PresetMeta},
        {"printer_model", ConfigOptionNature::Normal}, {"printer_variant", ConfigOptionNature::Normal},
        {"printer_vendor", ConfigOptionNature::Normal}, {"filament_vendor", ConfigOptionNature::Normal},
        {"material_vendor", ConfigOptionNature::Normal}, {"layer_height", ConfigOptionNature::Normal},
        {"wiping_volumes_matrix", ConfigOptionNature::Normal},
    };
    for (const std::pair<const char *, ConfigOptionNature> &entry : natures) {
        CAPTURE(entry.first);
        const Slic3r::ConfigOptionDef *definition = Slic3r::PrintConfigDef::instance().get(entry.first);
        REQUIRE(definition != nullptr);
        CHECK(static_cast<uint32_t>(definition->nature) == static_cast<uint32_t>(entry.second));
    }
}

// Write one configuration holding a key of every nature to each destination and read the text back:
// a refused key is absent, not written empty.
TEST_CASE("ConfigBase save writes the keys that the destination allows",
          "[plugins][config][nature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedNatureDirectory directory;
    Slic3r::DynamicPrintConfig config;
    set_nature_sentinels(config);
    const Slic3r::ConfigWriteTarget targets[] = {Slic3r::ConfigWriteTarget::Local, Slic3r::ConfigWriteTarget::Project,
                                                 Slic3r::ConfigWriteTarget::GCode};
    for (size_t target = 0; target < 3; ++target) {
        CAPTURE(target);
        const boost::filesystem::path file = directory.path() / ("target" + std::to_string(target) + ".ini");
        config.save(file.string(), targets[target]);
        const std::string text = Slic3r::Test::GCode::read_text_file(file);
        for (const NatureKeyExpectation &expectation : k_nature_keys) {
            CAPTURE(expectation.key);
            const bool expected = target == 0 ? expectation.in_local : target == 1 ? expectation.in_project : expectation.in_gcode;
            CHECK(has_config_line(text, "", expectation) == expected);
            CHECK(has_config_key(text, "", expectation) == expected);
        }
    }
}

// Save a preset holding a LOCAL key and a VOLATILE key in its own file: the first stays, the second is dropped.
TEST_CASE("Preset save keeps the local keys and drops the volatile ones",
          "[plugins][config][nature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedNatureDirectory directory;
    NaturePreset preset;
    preset.file = (directory.path() / "nature printer.ini").string();
    preset.config.set_deserialize_strict("print_host", "local_host_sentinel");
    preset.config.set_deserialize_strict("test_config_nature_volatile", "15");
    preset.save();
    const std::string text = Slic3r::Test::GCode::read_text_file(preset.file);
    CHECK(has_config_line(text, "", {"print_host", "local_host_sentinel", true, true, true}));
    CHECK_FALSE(has_config_key(text, "", {"test_config_nature_volatile", "15", true, true, true}));
}

// Text of two profile sections, as PresetBundle::project_profile_sections() would write them.
const char *const k_profile_sections_text =
    "[print:Test print]\ninherits = Parent print\n\n[printer:Test printer]\ndefault_print_profile = Test print\ninherits = Parent printer\n\n";

// Export a project: the flat project files carry the keys of the G-code (no PRESET_META, no LOCAL), the file read by PrusaSlicer
// has its own LOCAL filter applied before the conversion, and the profile sections text is stored in its own entry and read back.
TEST_CASE("3MF project configuration files follow their destination",
          "[plugins][config][nature][project]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedNatureDirectory directory;
    // The model comes from a regular print; the exported configuration only holds the sentinel keys.
    Slic3r::Model model;
    Slic3r::Print print;
    Slic3r::Test::init_print({Slic3r::Test::TestMesh::cube_20x20x20}, print, model);
    Slic3r::DynamicPrintConfig config;
    set_nature_sentinels(config);
    const boost::filesystem::path archive = directory.path() / "project.3mf";
    REQUIRE(Slic3r::store_3mf(archive.string().c_str(), &model, &config,
                              Slic3r::OptionStore3mf{}.set_profile_sections(k_profile_sections_text)));

    const std::string project_texts[] = {read_archive_entry(archive, "Metadata/Slic3r.config"),
                                         read_archive_entry(archive, "Metadata/SuperSlicer.config")};
    for (const std::string &text : project_texts) {
        REQUIRE_FALSE(text.empty());
        for (const NatureKeyExpectation &expectation : k_nature_keys) {
            CAPTURE(expectation.key);
            CHECK(has_config_line(text, "; ", expectation) == expectation.in_gcode);
            CHECK(has_config_key(text, "; ", expectation) == expectation.in_gcode);
        }
    }

    // These test options do not declare Prusa support, so conversion excludes them regardless of their nature.
    const std::string prusa_text = read_archive_entry(archive, "Metadata/Slic3r_PE.config");
    REQUIRE_FALSE(prusa_text.empty());
    for (const NatureKeyExpectation &expectation : k_nature_keys) {
        CAPTURE(expectation.key);
        if (std::string(expectation.key).rfind("test_config_nature_", 0) == 0)
            CHECK_FALSE(has_config_key(prusa_text, "; ", expectation));
        else
            CHECK(has_config_key(prusa_text, "; ", expectation) == expectation.in_gcode);
    }

    CHECK(read_archive_entry(archive, "Metadata/Slic3r_profiles.ini") == k_profile_sections_text);
    Slic3r::Model loaded_model;
    Slic3r::DynamicPrintConfig loaded_config;
    Slic3r::ConfigSubstitutionContext substitutions{Slic3r::ForwardCompatibilitySubstitutionRule::EnableSilent};
    std::string loaded_sections;
    REQUIRE(Slic3r::load_3mf(archive.string().c_str(), loaded_config, substitutions, &loaded_model, false, false, &loaded_sections));
    CHECK(loaded_sections == k_profile_sections_text);
}

// Save both unit forms through the real INI and 3MF paths; object overrides must retain the same value and unit.
TEST_CASE("Contact cone growth keeps its units in presets and projects", "[plugins][config][project][support-contact-xy]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedNatureDirectory directory;
    const char *const key = "support_material_contact_cone_growth";
    for (const char *value : {"25%", "67%", "100%", "0.268", "0"}) {
        CAPTURE(value);
        Model model;
        Print print;
        DynamicPrintConfig config = Test::SupportGrid::config();
        config.set_deserialize_strict({{key, value}, {"support_material_contact_xy_spacing", "0.1"}});
        Test::init_print({Test::SupportGrid::slab()}, print, model, config);
        model.objects()[0].config.set_deserialize_strict(key, value);
        // Fixture-only dotted option names stop the legacy 3MF footer parser; save only the native settings tested here.
        DynamicPrintConfig saved;
        saved.set_deserialize_strict({{key, value}, {"support_material_contact_xy_spacing", "0.1"}});
        const boost::filesystem::path ini = directory.path() / "contact.ini";
        saved.save(ini.string(), ConfigWriteTarget::Local);
        DynamicPrintConfig loaded_ini;
        loaded_ini.load(ini.string(), ForwardCompatibilitySubstitutionRule::EnableSilent);
        REQUIRE(loaded_ini.has(key));
        CHECK(loaded_ini.opt_serialize(key) == value);
        const boost::filesystem::path archive = directory.path() / "contact.3mf";
        REQUIRE(store_3mf(archive.string().c_str(), &model, &saved));
        REQUIRE(read_archive_entry(archive, "Metadata/Slic3r.config").find(std::string(key) + " = " + value) != std::string::npos);
        DynamicPrintConfig loaded_config;
        Model loaded_model;
        ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::EnableSilent};
        REQUIRE(load_3mf(archive.string().c_str(), loaded_config, substitutions, &loaded_model, false, false));
        REQUIRE(loaded_config.has(key));
        CHECK(loaded_config.opt_serialize(key) == value);
        REQUIRE(loaded_model.objects().size() == 1);
        CHECK(loaded_model.objects()[0].config.opt_serialize(key) == value);
    }
}

// In the 3MF's Prusa-compatible configuration, keep thin_walls but exclude gap_fill_extension.
// The local printhost_apikey value must be absent. Export must leave the source config unchanged.
TEST_CASE("3MF Prusa configuration keeps compatible plugin options",
          "[plugins][config][project][3mf][prusa]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedNatureDirectory directory;
    const NatureKeyExpectation compatible{"thin_walls", "0", true, true, true};
    const NatureKeyExpectation incompatible{"gap_fill_extension", "0.25", true, true, true};
    const NatureKeyExpectation local{"printhost_apikey", "local_secret_sentinel", true, false, false};
    const ConfigOptionDef *compatible_def = PrintConfigDef::instance().get(compatible.key);
    const ConfigOptionDef *incompatible_def = PrintConfigDef::instance().get(incompatible.key);
    REQUIRE(compatible_def != nullptr);
    REQUIRE(incompatible_def != nullptr);
    REQUIRE((compatible_def->mode & comPrusa) != 0);
    REQUIRE((incompatible_def->mode & comPrusa) == 0);

    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model);
    DynamicPrintConfig config;
    for (const NatureKeyExpectation &setting : {compatible, incompatible, local})
        config.set_deserialize_strict(setting.key, setting.value);
    const boost::filesystem::path archive = directory.path() / "plugin_options.3mf";
    REQUIRE(store_3mf(archive.string().c_str(), &model, &config));

    const std::string prusa_text = read_archive_entry(archive, "Metadata/Slic3r_PE.config");
    REQUIRE_FALSE(prusa_text.empty());
    CHECK(has_config_line(prusa_text, "; ", compatible));
    CHECK_FALSE(has_config_key(prusa_text, "; ", incompatible));
    CHECK_FALSE(has_config_key(prusa_text, "; ", local));
    for (const char *entry : {"Metadata/Slic3r.config", "Metadata/SuperSlicer.config"}) {
        CAPTURE(entry);
        const std::string text = read_archive_entry(archive, entry);
        REQUIRE_FALSE(text.empty());
        CHECK(has_config_line(text, "; ", compatible));
        CHECK(has_config_line(text, "; ", incompatible));
        CHECK_FALSE(has_config_key(text, "; ", local));
    }
    CHECK(config.opt_serialize(compatible.key) == compatible.value);
    CHECK(config.opt_serialize(incompatible.key) == incompatible.value);
    CHECK(config.opt_serialize(local.key) == local.value);
}

// The profile sections go with the configuration: no configuration, no sections entry.
TEST_CASE("3MF without exported configuration has no profile sections",
          "[plugins][config][project]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedNatureDirectory directory;
    Slic3r::Model model;
    Slic3r::Print print;
    Slic3r::Test::init_print({Slic3r::Test::TestMesh::cube_20x20x20}, print, model);
    Slic3r::DynamicPrintConfig config;
    set_nature_sentinels(config);
    const boost::filesystem::path archive = directory.path() / "project.3mf";
    REQUIRE(Slic3r::store_3mf(archive.string().c_str(), &model, &config,
                              Slic3r::OptionStore3mf{}.set_export_config(false).set_profile_sections(k_profile_sections_text)));
    CHECK(read_archive_entry(archive, "Metadata/Slic3r_profiles.ini").empty());
    CHECK(read_archive_entry(archive, "Metadata/Slic3r.config").empty());
}

// Export an AMF: its flat configuration block follows the G-code destination, and the profile sections have their own metadata.
TEST_CASE("AMF configuration block follows the project destination",
          "[plugins][config][nature][project]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedNatureDirectory directory;
    Slic3r::Model model;
    Slic3r::Print print;
    Slic3r::Test::init_print({Slic3r::Test::TestMesh::cube_20x20x20}, print, model);
    Slic3r::DynamicPrintConfig config;
    set_nature_sentinels(config);
    config.set_deserialize_strict("print_host", "local_host_sentinel");
    // store_amf forces the ".zip.amf" suffix and reports the final path through its argument.
    std::string path = (directory.path() / "project.amf").string();
    REQUIRE(Slic3r::store_amf(path, &model, &config, Slic3r::OptionStoreAmf{}.set_profile_sections(k_profile_sections_text)));
    const std::string text = read_archive_entry(path);
    REQUIRE_FALSE(text.empty());
    for (const NatureKeyExpectation &expectation : k_nature_keys) {
        CAPTURE(expectation.key);
        CHECK(has_config_line(text, "; ", expectation) == expectation.in_gcode);
        CHECK(has_config_key(text, "; ", expectation) == expectation.in_gcode);
    }
    CHECK_FALSE(has_config_key(text, "; ", {"print_host", "local_host_sentinel", true, false, false}));

    Slic3r::Model loaded_model;
    Slic3r::DynamicPrintConfig loaded_config;
    Slic3r::ConfigSubstitutionContext substitutions{Slic3r::ForwardCompatibilitySubstitutionRule::EnableSilent};
    std::string loaded_sections;
    REQUIRE(Slic3r::load_amf(path.c_str(), &loaded_config, &substitutions, &loaded_model, false, &loaded_sections));
    CHECK(loaded_sections == k_profile_sections_text);
    // The sections are not read as flat configuration.
    CHECK_FALSE(loaded_config.has("inherits"));

    // Without exported configuration, the sections are not written either.
    std::string path_without_config = (directory.path() / "no_config.amf").string();
    REQUIRE(Slic3r::store_amf(path_without_config, &model, &config,
                              Slic3r::OptionStoreAmf{}.set_export_config(false).set_profile_sections(k_profile_sections_text)));
    CHECK(read_archive_entry(path_without_config).find("slic3r_profiles") == std::string::npos);
}
// Select the filament preset of one extruder of a bundle, after the filament presets are loaded.
static void add_extruder_filament(Slic3r::PresetBundle &bundle, size_t extruder, const char *name)
{
    bundle.extruders_filaments.emplace_back(Slic3r::ExtruderFilaments(&bundle.filaments, extruder, name));
}

// The sections hold the metadata of the edited presets: the print, each distinct filament once, the printer.
TEST_CASE("Project profile sections hold the preset metadata of each selected profile",
          "[plugins][config][project]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Slic3r::PresetBundle bundle;

    Slic3r::DynamicPrintConfig print_config = bundle.fff_prints.default_preset().config;
    print_config.opt_string("inherits", true) = "Parent print";
    print_config.opt_string("compatible_printers_condition", true) = "nozzle_diameter[0]==0.4";
    print_config.set_deserialize_strict("layer_height", "0.27");
    bundle.fff_prints.load_preset("", "Test print", print_config);

    Slic3r::DynamicPrintConfig filament_a = bundle.filaments.default_preset().config;
    filament_a.opt_string("inherits", true) = "Parent A";
    filament_a.opt_string("compatible_printers_condition", true) = "nozzle_diameter[0]==0.6";
    bundle.filaments.load_preset("", "Filament A", filament_a);
    Slic3r::DynamicPrintConfig filament_b = bundle.filaments.default_preset().config;
    filament_b.opt_string("inherits", true) = "Parent B";
    filament_b.opt_string("compatible_prints_condition", true) = "layer_height>0.1";
    bundle.filaments.load_preset("", "Filament B", filament_b);
    // Three extruders; the first and the third use the same filament.
    add_extruder_filament(bundle, 0, "Filament A");
    add_extruder_filament(bundle, 1, "Filament B");
    add_extruder_filament(bundle, 2, "Filament A");

    Slic3r::DynamicPrintConfig printer_config = bundle.printers.default_preset().config;
    printer_config.opt_string("inherits", true) = "Parent printer";
    printer_config.opt_string("default_print_profile", true) = "Test print";
    printer_config.set_deserialize_strict("printhost_apikey", "local_secret_sentinel");
    printer_config.set_deserialize_strict("printer_notes", "notes_sentinel");
    bundle.printers.load_preset("", "Test printer", printer_config);
    // One key of a plugin of each kind that matters: PRESET_META must be written, VOLATILE must not.
    bundle.printers.get_edited_preset().config.set_deserialize_strict("test_config_nature_preset_meta", "14");
    bundle.printers.get_edited_preset().config.set_deserialize_strict("test_config_nature_volatile", "15");

    const std::string expected =
        "[print:Test print]\n"
        "compatible_printers = \n"
        "compatible_printers_condition = nozzle_diameter[0]==0.4\n"
        "inherits = Parent print\n"
        "\n"
        "[filament:Filament A]\n"
        "compatible_printers = \n"
        "compatible_printers_condition = nozzle_diameter[0]==0.6\n"
        "compatible_prints = \n"
        "compatible_prints_condition = \n"
        "inherits = Parent A\n"
        "\n"
        "[filament:Filament B]\n"
        "compatible_printers = \n"
        "compatible_printers_condition = \n"
        "compatible_prints = \n"
        "compatible_prints_condition = layer_height>0.1\n"
        "inherits = Parent B\n"
        "\n"
        "[printer:Test printer]\n"
        "default_filament_profile = \n"
        "default_print_profile = Test print\n"
        "inherits = Parent printer\n"
        "\n";
    const std::string sections = bundle.project_profile_sections();
    CHECK(has_config_line(sections, "", {"test_config_nature_preset_meta", "14", true, true, false}));
    CHECK_FALSE(has_config_key(sections, "", {"test_config_nature_volatile", "15", false, false, false}));
    // The other options registered by plugins (the test keys) are part of the presets; only the real keys are compared.
    std::string actual;
    for (const std::string &line : split_lines(sections))
        if (line.rfind("test_config_", 0) != 0)
            actual += line + "\n";
    CHECK(actual == expected);
}

// With an SLA printer, the sections are those of the SLA print, the SLA material and the printer.
TEST_CASE("Project profile sections hold the preset metadata of the SLA profiles",
          "[plugins][config][project]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Slic3r::PresetBundle bundle;

    Slic3r::DynamicPrintConfig print_config = bundle.sla_prints.default_preset().config;
    print_config.opt_string("inherits", true) = "Parent SLA print";
    print_config.opt_string("default_sla_print_profile", true) = "SlaPrintProfile";
    bundle.sla_prints.load_preset("", "SLA print", print_config);

    Slic3r::DynamicPrintConfig material_config = bundle.sla_materials.default_preset().config;
    material_config.opt_string("inherits", true) = "Parent material";
    material_config.opt_string("default_sla_material_profile", true) = "SlaMaterialProfile";
    material_config.opt_string("compatible_prints_condition", true) = "layer_height>0.1";
    bundle.sla_materials.load_preset("", "SLA material", material_config);

    Slic3r::DynamicPrintConfig printer_config = bundle.printers.default_preset(1).config;
    printer_config.opt_string("inherits", true) = "Parent SLA printer";
    printer_config.opt_string("default_sla_print_profile", true) = "SlaPrintProfile";
    printer_config.opt_string("default_sla_material_profile", true) = "SlaMaterialProfile";
    // load_external_preset chooses the SLA default preset from the technology of the configuration, load_preset would not.
    bundle.printers.load_external_preset("", "SLA printer", "", printer_config);
    REQUIRE(bundle.printers.get_edited_preset().printer_technology() == Slic3r::ptSLA);

    const std::string expected =
        "[sla_print:SLA print]\n"
        "compatible_printers = \n"
        "compatible_printers_condition = \n"
        "default_sla_print_profile = SlaPrintProfile\n"
        "inherits = Parent SLA print\n"
        "\n"
        "[sla_material:SLA material]\n"
        "compatible_printers = \n"
        "compatible_printers_condition = \n"
        "compatible_prints = \n"
        "compatible_prints_condition = layer_height>0.1\n"
        "default_sla_material_profile = SlaMaterialProfile\n"
        "inherits = Parent material\n"
        "\n"
        "[printer:SLA printer]\n"
        "default_sla_material_profile = SlaMaterialProfile\n"
        "default_sla_print_profile = SlaPrintProfile\n"
        "inherits = Parent SLA printer\n"
        "\n";
    std::string actual;
    for (const std::string &line : split_lines(bundle.project_profile_sections()))
        if (line.rfind("test_config_", 0) != 0)
            actual += line + "\n";
    CHECK(actual == expected);
}
// Each reconstructed preset receives the metadata of the section named after its original profile, and nothing from another profile.
TEST_CASE("Loading a project applies each profile section to its own preset",
          "[plugins][config][project]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Slic3r::DynamicPrintConfig config;
    config.apply(Slic3r::FullPrintConfig::defaults());
    config.set_deserialize_strict({{"printer_technology", "FFF"},
                                   {"nozzle_diameter", "0.4,0.4"},
                                   {"filament_diameter", "1.75,1.75"},
                                   {"print_settings_id", "PrintX"},
                                   {"filament_settings_id", "FilA;FilB"},
                                   {"printer_settings_id", "PrinterX"}});
    Slic3r::Preset::normalize(config);
    const std::string sections =
        "[print:PrintX]\ninherits = Parent print\ncompatible_printers_condition = nozzle_diameter[0]==0.4\ncompatible_printers = PrinterA\n\n"
        "[filament:FilA]\ninherits = Parent A\ncompatible_printers_condition = nozzle_diameter[0]==0.6\n\n"
        "[filament:FilB]\ninherits = Parent B\ncompatible_prints_condition = layer_height>0.1\ncompatible_prints = PrintY\n\n"
        "[printer:PrinterX]\ninherits = Parent printer\ndefault_print_profile = PrintX\ndefault_filament_profile = FilA\n\n";

    SECTION("every profile reads its own section")
    {
        Slic3r::PresetBundle bundle;
        bundle.load_config_model("project.3mf", config, sections);
        const Slic3r::Preset *print = bundle.fff_prints.find_preset("project.3mf (PrintX)");
        const Slic3r::Preset *printer = bundle.printers.find_preset("project.3mf (PrinterX)");
        const Slic3r::Preset *filament_a = bundle.filaments.find_preset("project.3mf (FilA)");
        const Slic3r::Preset *filament_b = bundle.filaments.find_preset("project.3mf (FilB)");
        REQUIRE(print != nullptr);
        REQUIRE(printer != nullptr);
        REQUIRE(filament_a != nullptr);
        REQUIRE(filament_b != nullptr);
        CHECK(print->inherits() == "Parent print");
        CHECK(print->compatible_printers_condition() == "nozzle_diameter[0]==0.4");
        CHECK(print->config.opt<Slic3r::ConfigOptionStrings>("compatible_printers")->get_values() == std::vector<std::string>{"PrinterA"});
        CHECK(filament_a->inherits() == "Parent A");
        CHECK(filament_a->compatible_printers_condition() == "nozzle_diameter[0]==0.6");
        CHECK(filament_a->compatible_prints_condition().empty());
        CHECK(filament_b->inherits() == "Parent B");
        CHECK(filament_b->compatible_printers_condition().empty());
        CHECK(filament_b->compatible_prints_condition() == "layer_height>0.1");
        CHECK(filament_b->config.opt<Slic3r::ConfigOptionStrings>("compatible_prints")->get_values() == std::vector<std::string>{"PrintY"});
        CHECK(filament_a->config.opt<Slic3r::ConfigOptionStrings>("compatible_prints")->get_values().empty());
        CHECK(printer->inherits() == "Parent printer");
        CHECK(printer->config.opt_string("default_print_profile") == "PrintX");
        CHECK(printer->config.opt<Slic3r::ConfigOptionStrings>("default_filament_profile")->get_values() == std::vector<std::string>{"FilA"});
    }

    SECTION("a profile without section keeps no metadata of another one")
    {
        Slic3r::PresetBundle bundle;
        bundle.load_config_model("project.3mf", config, "[filament:FilA]\ninherits = Parent A\ncompatible_printers_condition = nozzle_diameter[0]==0.6\n\n");
        const Slic3r::Preset *filament_a = bundle.filaments.find_preset("project.3mf (FilA)");
        const Slic3r::Preset *filament_b = bundle.filaments.find_preset("project.3mf (FilB)");
        REQUIRE(filament_a != nullptr);
        REQUIRE(filament_b != nullptr);
        CHECK(filament_a->inherits() == "Parent A");
        CHECK(filament_b->inherits().empty());
        CHECK(filament_b->compatible_printers_condition().empty());
        CHECK(bundle.fff_prints.find_preset("project.3mf (PrintX)")->inherits().empty());
    }

    SECTION("a section only applies the keys that the project accepts and the G-code refuses")
    {
        Slic3r::PresetBundle bundle;
        bundle.load_config_model("project.3mf", config,
                                 "[print:PrintX]\ninherits = Parent print\npost_process = injected_script\nlayer_height = 0.9\n\n");
        const Slic3r::Preset *print = bundle.fff_prints.find_preset("project.3mf (PrintX)");
        REQUIRE(print != nullptr);
        CHECK(print->inherits() == "Parent print");
        CHECK(print->config.opt<Slic3r::ConfigOptionStrings>("post_process")->get_values().empty());
        CHECK(print->config.opt_float("layer_height") != Approx(0.9));
    }
    SECTION("an SLA project applies the sections of its print, material and printer")
    {
        Slic3r::DynamicPrintConfig sla_config;
        sla_config.apply(Slic3r::SLAFullPrintConfig::defaults());
        sla_config.set_deserialize_strict({{"printer_technology", "SLA"},
                                           {"sla_print_settings_id", "SlaPrintX"},
                                           {"sla_material_settings_id", "SlaMaterialX"},
                                           {"printer_settings_id", "SlaPrinterX"}});
        Slic3r::Preset::normalize(sla_config);
        Slic3r::PresetBundle bundle;
        bundle.load_config_model("project.3mf", sla_config,
                                 "[sla_print:SlaPrintX]\ninherits = Parent SLA print\ndefault_sla_print_profile = SlaPrintProfile\n\n"
                                 "[sla_material:SlaMaterialX]\ninherits = Parent material\ndefault_sla_material_profile = SlaMaterialProfile\n\n"
                                 "[printer:SlaPrinterX]\ninherits = Parent SLA printer\ndefault_sla_print_profile = SlaPrintProfile\n"
                                 "default_sla_material_profile = SlaMaterialProfile\n\n");
        const Slic3r::Preset *print = bundle.sla_prints.find_preset("project.3mf (SlaPrintX)");
        const Slic3r::Preset *material = bundle.sla_materials.find_preset("project.3mf (SlaMaterialX)");
        const Slic3r::Preset *printer = bundle.printers.find_preset("project.3mf (SlaPrinterX)");
        REQUIRE(print != nullptr);
        REQUIRE(material != nullptr);
        REQUIRE(printer != nullptr);
        CHECK(print->inherits() == "Parent SLA print");
        CHECK(print->config.opt_string("default_sla_print_profile") == "SlaPrintProfile");
        CHECK(material->inherits() == "Parent material");
        CHECK(material->config.opt_string("default_sla_material_profile") == "SlaMaterialProfile");
        CHECK(printer->inherits() == "Parent SLA printer");
        CHECK(printer->config.opt_string("default_sla_print_profile") == "SlaPrintProfile");
        CHECK(printer->config.opt_string("default_sla_material_profile") == "SlaMaterialProfile");
    }}

namespace {
// Run one initial builder against borrowed sources, compare ordinary groups with
// the native builder and independently enumerate auxiliary owners, sources and
// placements. Whole-object comparisons apply to plates accepted by strict validation;
// stack modes use independent source coverage and parent checks instead.
// The Print owns inspected data; assertion and plugin failures propagate.
void check_sequence_builder(Slic3r::Print &print, bool object_mode);

// Invoke By object and compare each role with source layers, owners, coordinates
// and lower-island links, independently of group boundaries. The Print owns sources.
void check_stack_sequence_sources(Slic3r::Print &print);

void check_sequence_builder(Slic3r::Print &print, bool object_mode)
{
    using namespace Slic3r;
    if (object_mode && (print.full_print_config().opt_float("complete_objects_step") > 0. ||
        print.full_print_config().opt_float("complete_objects_step_max_z") > 0. ||
        print.full_print_config().opt_bool("complete_islands") ||
        print.full_print_config().opt_bool("complete_objects_one_first_layer") ||
        print.validate().first != PrintBase::PrintValidationError::pveNone)) {
        check_stack_sequence_sources(print);
        return;
    }
    Printing::PrintingPlan ordinary = object_mode ?
        Printing::build_printing_plan_by_object(print) : Printing::build_printing_plan_by_layer(print);
    Orchestrator &host = Orchestrator::instance();
    Plugin *plugin = host.get_plugin(object_mode ? "ordering.plan_builder.by_object" : "ordering.plan_builder.by_layer");
    REQUIRE(plugin != nullptr);
    Printing::PrintingPlan &actual = print.mutable_printing_plan();
    run_ctx_extrusion_ordering payload = {};
    payload.print = reinterpret_cast<const print_handle *>(&print);
    payload.plan = reinterpret_cast<printing_plan_handle *>(&actual);
    plugin_host_context context = host.prepare_plugin_host_context(STEP_ORDERING, plugin, &print);
    plugin_run_context run = host.prepare_plugin_run_context(STEP_ORDERING, plugin, &context);
    run.data = &payload;
    plugin->setup(run, 1);
    plugin->setup_run(run);
    plugin->run(run);

    const PrintObject *global = print.auxiliary_object();
    const size_t leading = object_mode && global != nullptr && std::any_of(global->auxiliary_layers().begin(), global->auxiliary_layers().end(),
        [](const Layer &) { return true; }) ? 1 : 0;
    REQUIRE(actual.groups.size() == ordinary.groups.size() + leading);
    REQUIRE(actual.object_instances.size() == ordinary.object_instances.size());
    for (size_t id = 0; id < ordinary.object_instances.size(); ++id) {
        CHECK(actual.object_instances[id].object == ordinary.object_instances[id].object);
        CHECK(actual.object_instances[id].object_instance_idx == ordinary.object_instances[id].object_instance_idx);
    }
    if (leading)
        CHECK(actual.groups.front().printing_catalog_ids.empty());
    using Key = std::tuple<const LayerRegionIsland *, ExtrusionRole, uint32_t>;
    struct Source { size_t group; coord_t z; extruder_index_t tool; double volume; Point first; Point last; };
    std::map<Key, Source> expected;
    std::vector<std::map<coord_t, std::set<const Layer *>>> layers(actual.groups.size());
    for (size_t g = 0; g < ordinary.groups.size(); ++g) {
        CHECK(actual.groups[g + leading].printing_catalog_ids == ordinary.groups[g].printing_catalog_ids);
        for (const Printing::PrintingLayerGroup &layer : ordinary.groups[g].layers) {
            layers[g + leading][layer.print_z].insert(layer.layers.begin(), layer.layers.end());
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                for (const Printing::PrintingExtrusion &extrusion : tool.extrusions) {
                    REQUIRE(extrusion.root != nullptr);
                    const Key key{extrusion.region_island, extrusion.sregion_island_role, extrusion.printing_catalog_id};
                    REQUIRE(expected.emplace(key, Source{g + leading, layer.print_z, tool.extruder_id,
                        extrusion.root->total_volume(), extrusion.root->first_point(), extrusion.root->last_point()}).second);
                }
        }
    }
    const ExtrusionRole roles[] = {LayerRegionIsland::SUPPORT, LayerRegionIsland::SUPPORT_INTERFACE,
        LayerRegionIsland::PERIMETERS, LayerRegionIsland::GAP_FILLS, LayerRegionIsland::INFILLS,
        LayerRegionIsland::IRONINGS, LayerRegionIsland::MILLS};
    // Each source layer is projected independently; marked local layers select one copy.
    for (size_t object_index = 0; object_index <= print.objects().size(); ++object_index) {
        const bool is_global = object_index == print.objects().size();
        const PrintObject *object = is_global ? global : &print.object(object_index);
        if (object == nullptr)
            continue;
        for (const Layer &layer : object->auxiliary_layers()) {
            const LayerInstanceProperty *owner = layer.get_property<LayerInstanceProperty>();
            const size_t copies = is_global ? 1 : object->instances().size();
            for (size_t copy = 0; copy < copies; ++copy) {
                if (!is_global && owner != nullptr && owner->object_instance_idx != copy)
                    continue;
                uint32_t id = Printing::PRINTING_OBJECT_INSTANCE_ID_NONE;
                size_t group = 0;
                Point shift(0, 0);
                if (!is_global) {
                    const auto instance = std::find_if(ordinary.object_instances.begin(), ordinary.object_instances.end(),
                        [object, copy](const Printing::PrintingObjectInstance &item) { return item.object == object && item.object_instance_idx == copy; });
                    REQUIRE(instance != ordinary.object_instances.end());
                    id = uint32_t(instance - ordinary.object_instances.begin());
                    shift = object->instances()[copy].shift;
                    if (object_mode) {
                        const auto target = std::find_if(ordinary.groups.begin(), ordinary.groups.end(),
                            [id](const Printing::PrintingGroup &item) { return std::find(item.printing_catalog_ids.begin(), item.printing_catalog_ids.end(), id) != item.printing_catalog_ids.end(); });
                        REQUIRE(target != ordinary.groups.end());
                        group = size_t(target - ordinary.groups.begin()) + leading;
                    }
                }
                layers[group][layer.scaled_print_z()].insert(&layer);
                for (const LayerSliceIsland &slice : layer.islands())
                    for (const LayerRegionIsland &island : slice.regions_islands())
                        for (const ExtrusionRole role : roles)
                            if (island.has_extrusion(role)) {
                                const ExtrusionEntity &root = island.extrusion(role);
                                REQUIRE(expected.emplace(Key{&island, role, id}, Source{group, layer.scaled_print_z(),
                                    island.extruder_id(), root.total_volume(), root.first_point() + shift, root.last_point() + shift}).second);
                            }
            }
        }
    }
    std::map<Key, size_t> visits;
    for (size_t g = 0; g < actual.groups.size(); ++g) {
        REQUIRE(actual.groups[g].layers.size() == layers[g].size());
        auto expected_layer = layers[g].begin();
        for (const Printing::PrintingLayerGroup &layer : actual.groups[g].layers) {
            REQUIRE(expected_layer != layers[g].end());
            CHECK(layer.print_z == expected_layer->first);
            CHECK(std::set<const Layer *>(layer.layers.begin(), layer.layers.end()) == expected_layer->second);
            CHECK(layer.layers.size() == expected_layer->second.size());
            ++expected_layer;
            std::set<extruder_index_t> expected_tools;
            for (const auto &entry : expected)
                if (entry.second.group == g && entry.second.z == layer.print_z)
                    expected_tools.insert(entry.second.tool);
            REQUIRE(layer.tool_groups.size() == expected_tools.size());
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups) {
                CHECK(expected_tools.erase(tool.extruder_id) == 1);
                for (const Printing::PrintingExtrusion &extrusion : tool.extrusions) {
                    const Key key{extrusion.region_island, extrusion.sregion_island_role, extrusion.printing_catalog_id};
                    const auto source = expected.find(key);
                    REQUIRE(source != expected.end());
                    CHECK(source->second.group == g);
                    CHECK(source->second.z == layer.print_z);
                    CHECK(source->second.tool == tool.extruder_id);
                    REQUIRE(extrusion.root != nullptr);
                    CHECK(extrusion.root->total_volume() == Approx(source->second.volume));
                    CHECK(extrusion.root->first_point() == source->second.first);
                    CHECK(extrusion.root->last_point() == source->second.last);
                    ++visits[key];
                }
            }
        }
    }
    REQUIRE(visits.size() == expected.size());
    for (const auto &entry : expected)
        CHECK(visits[entry.first] == 1);
}
void check_stack_sequence_sources(Slic3r::Print &print)
{
    using namespace Slic3r;
    using Key = std::tuple<const LayerRegionIsland *, ExtrusionRole, const PrintObject *, size_t>;
    using UnitKey = std::tuple<const LayerSliceIsland *, const PrintObject *, size_t>;
    struct Source { const Layer *layer; const LayerSliceIsland *island; Point shift; };
    std::map<Key, Source> expected;
    std::set<UnitKey> expected_units;
    const ExtrusionRole roles[] = {LayerRegionIsland::SUPPORT, LayerRegionIsland::SUPPORT_INTERFACE,
        LayerRegionIsland::PERIMETERS, LayerRegionIsland::GAP_FILLS, LayerRegionIsland::INFILLS,
        LayerRegionIsland::IRONINGS, LayerRegionIsland::MILLS};
    const PrintObject *global = print.auxiliary_object();
    for (size_t idx = 0; idx <= print.objects().size(); ++idx) {
        const bool auxiliary_object = idx == print.objects().size();
        const PrintObject *object = auxiliary_object ? global : &print.object(idx);
        if (object == nullptr)
            continue;
        std::vector<const Layer *> layers;
        if (!auxiliary_object)
            for (const Layer &layer : object->layers())
                layers.push_back(&layer);
        for (const Layer &layer : object->auxiliary_layers())
            layers.push_back(&layer);
        for (size_t copy = 0; copy < (auxiliary_object ? 1 : object->instances().size()); ++copy) {
            const Point shift = auxiliary_object ? Point(0, 0) : object->instances()[copy].shift;
            for (const Layer *layer : layers) {
                const LayerInstanceProperty *owner = layer->get_property<LayerInstanceProperty>();
                if (!auxiliary_object && owner != nullptr && owner->object_instance_idx != copy)
                    continue;
                for (const LayerSliceIsland &island : layer->islands())
                    for (const LayerRegionIsland &region : island.regions_islands())
                        for (ExtrusionRole role : roles)
                            if (region.has_extrusion(role) && !region.extrusion(role).empty()) {
                                REQUIRE(expected.emplace(Key{&region, role, object, copy}, Source{layer, &island, shift}).second);
                                expected_units.insert(UnitKey{&island, object, copy});
                            }
            }
        }
    }
    Orchestrator &host = Orchestrator::instance();
    Plugin *builder = host.get_plugin("ordering.plan_builder.by_object");
    REQUIRE(builder != nullptr);
    run_ctx_extrusion_ordering payload = {};
    payload.print = reinterpret_cast<const print_handle *>(&print);
    payload.plan = reinterpret_cast<printing_plan_handle *>(&print.mutable_printing_plan());
    plugin_host_context context = host.prepare_plugin_host_context(STEP_ORDERING, builder, &print);
    plugin_run_context run = host.prepare_plugin_run_context(STEP_ORDERING, builder, &context);
    run.data = &payload;
    builder->setup(run, 1);
    builder->setup_run(run);
    builder->run(run);
    REQUIRE(print.printing_plan() != nullptr);
    const Printing::PrintingPlan &plan = *print.printing_plan();
    std::set<Key> visited;
    std::set<UnitKey> visited_units;
    for (const Printing::PrintingGroup &group : plan.groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers) {
            std::set<UnitKey> layer_units;
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                for (const Printing::PrintingExtrusion &extrusion : tool.extrusions) {
                    const PrintObject *object = global;
                    size_t copy = 0;
                    if (extrusion.printing_catalog_id != Printing::PRINTING_OBJECT_INSTANCE_ID_NONE) {
                        REQUIRE(extrusion.printing_catalog_id < plan.object_instances.size());
                        object = plan.object_instances[extrusion.printing_catalog_id].object;
                        copy = plan.object_instances[extrusion.printing_catalog_id].object_instance_idx;
                    }
                    const Key key{extrusion.region_island, extrusion.sregion_island_role, object, copy};
                    REQUIRE(expected.count(key) == 1);
                    CHECK(visited.insert(key).second);
                    const Source &source = expected.at(key);
                    CHECK(layer.print_z == source.layer->scaled_print_z());
                    CHECK(std::find(layer.layers.begin(), layer.layers.end(), source.layer) != layer.layers.end());
                    CHECK(tool.extruder_id == extrusion.region_island->extruder_id());
                    REQUIRE(extrusion.root != nullptr);
                    const ExtrusionEntity &root = extrusion.region_island->extrusion(extrusion.sregion_island_role);
                    CHECK(extrusion.root->total_volume() == Approx(root.total_volume()));
                    Points points, actual;
                    root.collect_points(points);
                    extrusion.root->collect_points(actual);
                    for (Point &point : points)
                        point += source.shift;
                    CHECK(actual == points);
                    const UnitKey unit{source.island, object, copy};
                    if (layer_units.insert(unit).second) {
                        CHECK(visited_units.insert(unit).second);
                        for (const auto &[parent, overlap] : source.island->overlaps_below) {
                            const UnitKey below{parent, object, copy};
                            if (expected_units.count(below))
                                CHECK(visited_units.count(below) == 1);
                        }
                    }
                }
        }
    CHECK(visited.size() == expected.size());
}
} // namespace

// Export small STL copies while varying every object-builder setting independently,
// then compare its groups, layers and extrusion sources with the native builder.
// Island schedules instead compare all source roles and translated clones across
// groups, since the native builder does not implement stack scheduling.
TEST_CASE("By object builder preserves source plans through the STL export catalogue", "[plugins][printing-plan][print-sequence][plugin-e2e]")
{
    using namespace Slic3r;
    namespace E2E = Slic3r::Test::PluginE2E;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    DynamicPrintConfig baseline = DynamicPrintConfig::full_print_config();
    baseline.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
        {"complete_objects_step", "0"}, {"complete_objects_sort", "object"},
        {"complete_objects_step_max_z", "0"}, {"complete_islands", "0"},
        {"skirts", "0"}, {"skirt_height", "1"}, {"brim_width", "0"}, {"brim_per_object", "1"},
        {"complete_objects_one_skirt", "0"}, {"layer_height", "0.3"}, {"first_layer_height", "0.3"},
        {"fill_density", "0%"}, {"support_material", "0"}, {"support_material_enforce_layers", "0"},
        {"step_gcode_plugin", "gcode.printing_plan_file_writer"}, {"gcode_flavor", "marlin2"}});
    const ConfigOptionGraphs flat_clearance({GraphData(0, 2, GraphData::GraphType::LINEAR,
        {{0., 0.}, {2., 0.1}})});
    const ConfigOptionGraphs wide_clearance({GraphData(0, 3, GraphData::GraphType::LINEAR,
        {{0., 0.}, {50., 0.1}, {51., 1.9}})});
    const std::vector<E2E::Domain> domains = {
        E2E::enum_domain("complete_objects_sort"),
        {{"complete_objects_one_first_layer"}, {
            {"first-layer-off", {{"complete_objects_one_first_layer", "0"}}},
            {"first-layer-on", {{"complete_objects_one_first_layer", "1"}}}}},
        {{"complete_objects_step", "extruder_clearance_height", "complete_objects_step_max_z", "skirts", "skirt_height", "complete_objects_one_skirt"}, {
            {"height-step", {{"complete_objects_step", "1"}}},
            {"clearance-cap", {{"complete_objects_step", "1"}, {"extruder_clearance_height", "0.6"}}},
            {"height-limit-without-step", {{"complete_objects_step_max_z", "0.6"}}},
            {"height-limit", {{"complete_objects_step", "1"}, {"complete_objects_step_max_z", "0.6"}}},
            {"height-limit-above", {{"complete_objects_step", "1"}, {"complete_objects_step_max_z", "100"}}},
            {"height-limit-local-skirt", {{"complete_objects_step", "1"}, {"complete_objects_step_max_z", "0.6"}, {"skirts", "1"}, {"skirt_height", "4"}}},
            {"height-limit-global-skirt", {{"complete_objects_step", "1"}, {"complete_objects_step_max_z", "0.6"}, {"skirts", "1"}, {"skirt_height", "4"}, {"complete_objects_one_skirt", "1"}}}}},
        {{"complete_islands", "complete_objects_step", "complete_objects_step_max_z", "extruder_clearance", "extruder_clearance_radius"}, {
            {"separate-islands", {{"complete_islands", "1"}}},
            {"island-stacks", {{"complete_islands", "1"}, {"complete_objects_step", "0.5"}}},
            {"zero-radius", {{"complete_islands", "1"}, {"complete_objects_step", "0.5"}, {"extruder_clearance_radius", "0"}}},
            {"large-radius", {{"complete_islands", "1"}, {"complete_objects_step", "0.5"}, {"extruder_clearance_radius", "60"}}},
            {"island-limit", {{"complete_islands", "1"}, {"complete_objects_step", "0.5"}, {"complete_objects_step_max_z", "0.6"}}},
            {"flat-clearance", {{"complete_islands", "1"}, {"complete_objects_step", "0.5"}, {"extruder_clearance", flat_clearance.serialize()}}},
            {"wide-clearance", {{"complete_islands", "1"}, {"complete_objects_step", "0.5"}, {"extruder_clearance", wide_clearance.serialize()}}}}}
    };
    TriangleMesh mesh = E2E::stl_fixture(false);
    mesh.scale(2.);
    Model layer_model;
    Print layer_print;
    Test::init_print({mesh}, layer_print, layer_model, baseline);
    layer_model.objects().front().instances.front()->set_offset(Vec3d(40., 40., 0.));
    layer_model.objects().front().add_instance(*layer_model.objects().front().instances.front())->set_offset(Vec3d(90., 40., 0.));
    layer_print.apply(layer_model, baseline);
    layer_print.process();
    check_sequence_builder(layer_print, false);
    for (const E2E::Case &test : E2E::linear_cases(host, {"ordering.plan_builder.by_object"}, baseline, domains)) {
        CAPTURE(test.name);
        DynamicPrintConfig config = E2E::apply_case(baseline, test);
        Print print;
        Model model;
        Test::init_print({mesh}, print, model, config);
        model.objects().front().instances.front()->set_offset(Vec3d(40., 40., 0.));
        model.objects().front().add_instance(*model.objects().front().instances.front())->set_offset(Vec3d(90., 40., 0.));
        print.apply(model, config);
        print.process();
        const boost::filesystem::path path = Test::GCode::temporary_gcode_path();
        std::string code;
        try {
            code = Test::GCode::read_text_file(host.export_gcode_cli(print, path.string()));
            Test::GCode::remove_output_pair(path);
        } catch (...) {
            Test::GCode::remove_output_pair(path);
            throw;
        }
        size_t extruding_moves = 0;
        GCodeReader reader;
        // Count actual XY deposition, not travel or header text, in the exported file.
        reader.parse_buffer(code, [&extruding_moves](GCodeReader &state, const GCodeReader::GCodeLine &line) {
            if (line.extruding(state) && line.dist_XY(state) > 0.)
                ++extruding_moves;
        });
        CHECK(extruding_moves > 0);
        if (config.opt_bool("complete_islands") && config.opt_float("complete_objects_step") > 0.) {
            // Stack boundaries differ from whole-object groups, but every source
            // role must still have one clone at its own instance's translation.
            const Printing::PrintingPlan expected = Printing::build_printing_plan_by_object(print);
            using Key = std::tuple<const LayerRegionIsland *, ExtrusionRole, size_t>;
            std::map<Key, const Printing::PrintingExtrusion *> sources;
            for (const Printing::PrintingGroup &group : expected.groups)
                for (const Printing::PrintingLayerGroup &layer : group.layers)
                    for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                        for (const Printing::PrintingExtrusion &extrusion : tool.extrusions) {
                            REQUIRE(extrusion.printing_catalog_id < expected.object_instances.size());
                            const size_t instance = expected.object_instances[extrusion.printing_catalog_id].object_instance_idx;
                            REQUIRE(sources.emplace(Key{extrusion.region_island, extrusion.sregion_island_role, instance}, &extrusion).second);
                        }
            REQUIRE_FALSE(sources.empty());
            Plugin *builder = host.get_plugin("ordering.plan_builder.by_object");
            REQUIRE(builder != nullptr);
            run_ctx_extrusion_ordering payload = {};
            payload.print = reinterpret_cast<const print_handle *>(&print);
            payload.plan = reinterpret_cast<printing_plan_handle *>(&print.mutable_printing_plan());
            plugin_host_context context = host.prepare_plugin_host_context(STEP_ORDERING, builder, &print);
            plugin_run_context run = host.prepare_plugin_run_context(STEP_ORDERING, builder, &context);
            run.data = &payload;
            builder->setup(run, 1);
            builder->setup_run(run);
            builder->run(run);
            REQUIRE(print.printing_plan() != nullptr);
            const Printing::PrintingPlan &actual = *print.printing_plan();
            std::set<Key> visits;
            for (const Printing::PrintingGroup &group : actual.groups)
                for (const Printing::PrintingLayerGroup &layer : group.layers)
                    for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                        for (const Printing::PrintingExtrusion &extrusion : tool.extrusions) {
                            REQUIRE(extrusion.printing_catalog_id < actual.object_instances.size());
                            const size_t instance = actual.object_instances[extrusion.printing_catalog_id].object_instance_idx;
                            const Key key{extrusion.region_island, extrusion.sregion_island_role, instance};
                            REQUIRE(sources.count(key) == 1);
                            CHECK(visits.insert(key).second);
                            REQUIRE(extrusion.root != nullptr);
                            const ExtrusionEntity &source = *sources.at(key)->root;
                            CHECK(extrusion.root->total_volume() == Approx(source.total_volume()));
                            CHECK(extrusion.root->first_point() == source.first_point());
                            CHECK(extrusion.root->last_point() == source.last_point());
                        }
            CHECK(visits.size() == sources.size());
        } else {
            check_sequence_builder(print, true);
        }
        if (config.opt_float("complete_objects_step_max_z") == 0.6) {
            REQUIRE(print.object(0).layer_count() > 1);
            CHECK(print.object(0).layer(0).scaled_print_z() < scale_to_layer_coord(0.6));
            CHECK(print.object(0).layers().back().scaled_print_z() > scale_to_layer_coord(0.6));
            REQUIRE(print.printing_plan()->groups.back().printing_catalog_ids.size() == 2);
            REQUIRE_FALSE(print.printing_plan()->groups.back().layers.empty());
            CHECK(print.printing_plan()->groups.back().layers.front().print_z >= scale_to_layer_coord(0.6));
        }
    }
}

// Build each auxiliary scene once, prove the requested geometry exists, then
// independently check both builders' complete source/owner coverage and placement.
TEST_CASE("Plan builders preserve auxiliary sources and owners in distinct scenes", "[plugins][printing-plan][print-sequence]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    for (const std::string scene : {"ordinary", "local-skirt", "shared-local-skirt", "global-skirt", "brim", "support", "wipe-tower"}) {
        CAPTURE(scene);
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
            {"complete_objects_step", "0"}, {"complete_objects_sort", "object"}, {"skirts", "0"},
            {"brim_width", "0"}, {"brim_width_interior", "0"}, {"support_material", "0"},
            {"support_material_enforce_layers", "0"}, {"wipe_tower", "0"}, {"layer_height", "0.3"},
            {"first_layer_height", "0.3"}, {"brim_skirt_trim", "0"}, {"draft_shield", "disabled"},
            {"step_gcode_plugin", "gcode.printing_plan_file_writer"}, {"gcode_flavor", "marlin2"}});
        if (scene == "local-skirt" || scene == "shared-local-skirt" || scene == "global-skirt")
            config.set_deserialize_strict({{"skirts", "1"}, {"skirt_height", "1"}, {"min_skirt_length", "0"},
                {"complete_objects_one_skirt", scene == "global-skirt" ? "1" : "0"}});
        if (scene == "local-skirt")
            config.set_deserialize_strict("skirt_distance", "30");
        if (scene == "brim")
            config.set_deserialize_strict({{"brim_width", "1"}, {"brim_per_object", "1"}});
        TriangleMesh mesh = make_cube(5., 5., 1.2);
        if (scene == "support") {
            mesh = Test::SupportGrid::slab(true);
            config = Test::SupportGrid::config("grid", false, 3, true);
            config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
                {"complete_objects_step", "0"}, {"filament_diameter", "1.75,1.75"},
                {"step_gcode_plugin", "gcode.printing_plan_file_writer"}, {"gcode_flavor", "marlin2"}});
        }
        if (scene == "wipe-tower") {
            config.set_num_extruders(2);
            config.set_deserialize_strict({{"nozzle_diameter", "0.4,0.4"}, {"filament_diameter", "1.75,1.75"},
                {"extrusion_multiplier", "1,1"}, {"wiping_volumes_matrix", "0,8,8,0"},
                {"filament_minimal_purge_on_wipe_tower", "2,2"}, {"wipe_tower", "1"},
                {"wipe_tower_width", "20"}, {"wipe_tower_object_x", "15"}, {"wipe_tower_object_y", "15"},
                {"perimeter_extruder", "1"}, {"infill_extruder", "2"}, {"solid_infill_extruder", "2"},
                {"step_wipetower_plugin", "wipe_tower.manager.default"}});
        }
        Model model;
        Print print;
        Test::init_print({mesh}, print, model, config);
        ModelObject &object = model.objects().front();
        object.instances.front()->set_offset(Vec3d(40., 40., 0.));
        object.add_instance(*object.instances.front())->set_offset(Vec3d(110., 60., 0.));
        // This placement makes at least one skirt belong to a single copy.
        if (scene == "local-skirt")
            object.add_instance(*object.instances.front())->set_offset(Vec3d(142., 60., 0.));
        print.apply(model, config);
        print.process();
        const boost::filesystem::path output = Test::GCode::temporary_gcode_path();
        try {
            host.export_gcode_cli(print, output.string());
            Test::GCode::remove_output_pair(output);
        } catch (...) {
            Test::GCode::remove_output_pair(output);
            throw;
        }
        size_t local_skirts = 0, global_skirts = 0, brims = 0, supports = 0;
        std::set<size_t> skirt_owners;
        for (const Layer &layer : print.object(0).auxiliary_layers()) {
            if (LayerAdhesionProperty::layer_is_normal_skirt(layer)) {
                ++local_skirts;
                const LayerInstanceProperty *owner = layer.get_property<LayerInstanceProperty>();
                if (owner != nullptr)
                    skirt_owners.insert(owner->object_instance_idx);
            }
            if (LayerAdhesionProperty::layer_is_brim(layer))
                ++brims;
            for (const LayerSliceIsland &slice : layer.islands())
                for (const LayerRegionIsland &island : slice.regions_islands())
                    if (island.has_extrusion(LayerRegionIsland::SUPPORT) || island.has_extrusion(LayerRegionIsland::SUPPORT_INTERFACE))
                        ++supports;
        }
        if (print.auxiliary_object() != nullptr)
            for (const Layer &layer : print.auxiliary_object()->auxiliary_layers()) {
                CHECK(layer.get_property<LayerInstanceProperty>() == nullptr);
                if (LayerAdhesionProperty::layer_is_normal_skirt(layer))
                    ++global_skirts;
            }
        if (scene == "local-skirt") {
            REQUIRE(local_skirts > 0);
            REQUIRE_FALSE(skirt_owners.empty());
        }
        if (scene == "global-skirt") {
            REQUIRE(global_skirts > 0);
        }
        if (scene == "shared-local-skirt") {
            REQUIRE(local_skirts > 0);
            REQUIRE(skirt_owners.empty());
        }
        if (scene == "brim") CHECK(brims > 0);
        if (scene == "support") CHECK(supports > 0);
        if (scene == "wipe-tower") {
            REQUIRE(print.printing_plan() != nullptr);
            bool tower_present = false;
            for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
                for (const Printing::PrintingLayerGroup &layer : group.layers)
                    for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                        for (const Printing::PrintingExtrusion &extrusion : tool.extrusions)
                            if (extrusion.sregion_island_role.is_wipetower() && extrusion.root)
                                tower_present = true;
            REQUIRE(tower_present);
        }
        // Tower geometry is generated after initial construction; the builders consume source layers only.
        check_sequence_builder(print, false);
        check_sequence_builder(print, true);
    }
}

// Distinct heights and XY positions distinguish model, height, axis and nearest
// ordering; expected model indices are literal, not derived from either builder.
TEST_CASE("Object sort settings produce distinct explicit instance orders", "[plugins][printing-plan][print-sequence][ordering]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
        {"complete_objects_step", "0"}, {"complete_objects_sort", "object"}, {"skirts", "0"},
        {"brim_width", "0"}, {"support_material", "0"}, {"wipe_tower", "0"}, {"layer_height", "0.3"}});
    Model model;
    Print print;
    Test::init_print({make_cube(2., 2., 3.), make_cube(2., 2., 1.), make_cube(2., 2., 4.), make_cube(2., 2., 2.)}, print, model, config);
    const Vec3d positions[] = {Vec3d(50., 50., 0.), Vec3d(150., 20., 0.), Vec3d(100., 150., 0.), Vec3d(60., 100., 0.)};
    for (size_t i = 0; i < 4; ++i)
        model.objects()[i].instances.front()->set_offset(positions[i]);
    print.apply(model, config);
    print.process();
    check_sequence_builder(print, false);
    struct Order { const char *setting; std::vector<size_t> indices; };
    const Order orders[] = {{"object", {0, 1, 2, 3}}, {"lowz", {1, 3, 0, 2}},
        {"lowy", {1, 0, 3, 2}}, {"nearest", {0, 3, 2, 1}},
        {"lowx", {0, 3, 2, 1}}, {"highx", {1, 2, 3, 0}}, {"highy", {2, 3, 0, 1}}};
    for (const Order &order : orders) {
        CAPTURE(order.setting);
        config.set_deserialize_strict("complete_objects_sort", order.setting);
        print.apply(model, config);
        print.process();
        check_sequence_builder(print, true);
        const Printing::PrintingPlan &plan = *print.printing_plan();
        REQUIRE(plan.groups.size() == 4);
        for (size_t g = 0; g < 4; ++g) {
            REQUIRE(plan.groups[g].printing_catalog_ids.size() == 1);
            const Printing::PrintingObjectInstance &instance = plan.object_instances[plan.groups[g].printing_catalog_ids.front()];
            CHECK(instance.object->model_object()->id() == model.objects()[order.indices[g]].id());
            CHECK(instance.object_instance_idx == 0);
        }
    }
}

// Repeated coordinates distinguish stable object/copy ties from reversing an ascending result.
// A shared first-layer group must visit its copies in the same order as the later whole-copy groups.
TEST_CASE("XY object sorts keep copy ties stable through the shared first layer",
          "[plugins][printing-plan][print-sequence][ordering][object-sort]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool shared_first_layer = GENERATE(false, true);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
        {"complete_objects_step", "0"}, {"complete_objects_sort", "object"},
        {"complete_objects_one_first_layer", shared_first_layer ? "1" : "0"},
        {"skirts", "0"}, {"brim_width", "0"}, {"support_material", "0"}, {"wipe_tower", "0"},
        {"extruder_clearance_radius", "0"}, {"layer_height", "0.3"}, {"first_layer_height", "0.3"}});
    Model model;
    Print print;
    Test::init_print({make_cube(2., 2., 1.2), make_cube(2., 2., 1.2)}, print, model, config);
    model.objects()[0].instances.front()->set_offset(Vec3d(20., 80., 0.));
    model.objects()[0].add_instance(*model.objects()[0].instances.front())->set_offset(Vec3d(20., 20., 0.));
    model.objects()[0].add_instance(*model.objects()[0].instances.front())->set_offset(Vec3d(80., 20., 0.));
    model.objects()[1].instances.front()->set_offset(Vec3d(20., 50., 0.));
    model.objects()[1].add_instance(*model.objects()[1].instances.front())->set_offset(Vec3d(80., 50., 0.));
    using Copy = std::pair<size_t, size_t>;
    struct Order { const char *setting; std::vector<Copy> copies; };
    const Order orders[] = {
        {"lowx", {{0, 0}, {0, 1}, {1, 0}, {0, 2}, {1, 1}}},
        {"highx", {{0, 2}, {1, 1}, {0, 0}, {0, 1}, {1, 0}}},
        {"lowy", {{0, 1}, {0, 2}, {1, 0}, {1, 1}, {0, 0}}},
        {"highy", {{0, 0}, {1, 0}, {1, 1}, {0, 1}, {0, 2}}}
    };
    for (const Order &order : orders) {
        CAPTURE(order.setting, shared_first_layer);
        config.set_deserialize_strict("complete_objects_sort", order.setting);
        print.apply(model, config);
        print.process();
        // The native builder has no shared first-layer group; its low-Y hull sort is a separate compatibility mode.
        if (!shared_first_layer && std::string(order.setting) != "lowy")
            check_sequence_builder(print, true);
        else
            check_stack_sequence_sources(print);
        REQUIRE(print.printing_plan() != nullptr);
        const Printing::PrintingPlan &plan = *print.printing_plan();
        const size_t prefix = shared_first_layer ? 1 : 0;
        REQUIRE(plan.groups.size() == order.copies.size() + prefix);
        if (shared_first_layer)
            REQUIRE(plan.groups.front().printing_catalog_ids.size() == order.copies.size());
        for (size_t i = 0; i < order.copies.size(); ++i) {
            const Printing::PrintingGroup &group = plan.groups[i + prefix];
            REQUIRE(group.printing_catalog_ids.size() == 1);
            const Printing::PrintingObjectInstance &instance = plan.object_instances[group.printing_catalog_ids.front()];
            CHECK(instance.object->model_object()->id() == model.objects()[order.copies[i].first].id());
            CHECK(instance.object_instance_idx == order.copies[i].second);
            if (shared_first_layer)
                CHECK(plan.groups.front().printing_catalog_ids[i] == group.printing_catalog_ids.front());
        }
    }
}

// Stack unequal boxes and support geometry, then compare source roles, owners,
// translations and parents independently of the stack boundaries.
TEST_CASE("By object steps cover every layer through empty slices", "[plugins][printing-plan][print-sequence][ordering][object-step]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::pair<double, const char *> cases[] = {
        {0.1, "object"}, {0.2, "object"}, {0.5, "object"},
        {0.5, "lowz"}, {0.5, "nearest"}, {2., "object"}
    };
    for (const std::pair<double, const char *> &test : cases) {
        const double step = test.first;
        const char *order = test.second;
        CAPTURE(step, order);
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
            {"complete_objects_step", std::to_string(step)}, {"complete_objects_sort", order},
            {"extruder_clearance_height", "0.7"}, {"skirts", "0"}, {"brim_width", "1"},
            {"brim_per_object", "1"}, {"support_material", "0"}, {"wipe_tower", "0"},
            {"first_layer_height", "0.2"}, {"layer_height", "0.3"}});
        Model model;
        Print print;
        Test::init_print({make_cube(3., 3., 1.4), make_cube(3., 3., 0.8)}, print, model, config);
        model.objects()[0].instances.front()->set_offset(Vec3d(40., 40., 0.));
        model.objects()[1].instances.front()->set_offset(Vec3d(100., 40., 0.));
        print.apply(model, config);
        print.process();
        size_t brims = 0;
        for (const PrintObject &object : print.objects())
            for (const Layer &layer : object.auxiliary_layers())
                if (LayerAdhesionProperty::layer_is_brim(layer))
                    for (const LayerSliceIsland &slice : layer.islands())
                        for (const LayerRegionIsland &island : slice.regions_islands())
                            for (const ExtrusionRole role : {LayerRegionIsland::SUPPORT, LayerRegionIsland::SUPPORT_INTERFACE,
                                LayerRegionIsland::PERIMETERS, LayerRegionIsland::GAP_FILLS, LayerRegionIsland::INFILLS,
                                LayerRegionIsland::IRONINGS, LayerRegionIsland::MILLS})
                                if (island.has_extrusion(role) && island.extrusion(role).total_volume() > 0.)
                                    ++brims;
        REQUIRE(brims > 0);
        check_sequence_builder(print, true);
        REQUIRE(print.printing_plan()->groups.size() >= 2);
        CHECK(print.printing_plan()->groups.front().layers.front().print_z == scale_to_layer_coord(0.2));
    }
    DynamicPrintConfig config = Test::SupportGrid::config("grid", false, 3, true);
    config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
        {"complete_objects_step", "0.5"}, {"filament_diameter", "1.75,1.75"}});
    Model model;
    Print print;
    Test::init_print({Test::SupportGrid::slab(true)}, print, model, config);
    print.process();
    size_t supports = 0;
    for (const Layer &layer : print.object(0).auxiliary_layers())
        for (const LayerSliceIsland &slice : layer.islands())
            for (const LayerRegionIsland &island : slice.regions_islands())
                for (const ExtrusionRole role : {LayerRegionIsland::SUPPORT, LayerRegionIsland::SUPPORT_INTERFACE})
                    if (island.has_extrusion(role) && island.extrusion(role).total_volume() > 0.)
                        ++supports;
    REQUIRE(supports > 0);
    check_sequence_builder(print, true);
    config.set_deserialize_strict("complete_objects_step_max_z", "0.6");
    print.apply(model, config);
    print.process();
    check_sequence_builder(print, true);
    for (const bool shared : {false, true}) {
    CAPTURE(shared);
    config.set_deserialize_strict({{"complete_objects_one_first_layer", shared ? "1" : "0"},
        {"raft_layers", "3"}, {"complete_objects_step_max_z", "0"}});
    Model raft_model;
    Print raft_print;
    Test::init_print({Test::SupportGrid::slab(true)}, raft_print, raft_model, config);
    raft_print.process();
    check_sequence_builder(raft_print, true);
    const coord_t first_z = raft_print.object(0).layer(0).scaled_print_z();
    const Printing::PrintingGroup &first = raft_print.printing_plan()->groups.front();
    size_t auxiliary_extrusions = 0;
    for (const Layer &source : raft_print.object(0).auxiliary_layers()) {
        CAPTURE(first_z, source.scaled_print_z());
        for (const LayerSliceIsland &slice : source.islands())
            for (const LayerRegionIsland &region : slice.regions_islands())
                for (const ExtrusionRole role : {LayerRegionIsland::SUPPORT, LayerRegionIsland::SUPPORT_INTERFACE})
                    if (source.scaled_print_z() <= first_z && region.has_extrusion(role) && !region.extrusion(role).empty())
                        ++auxiliary_extrusions;
        if (shared && source.scaled_print_z() <= first_z)
            CHECK(std::any_of(first.layers.begin(), first.layers.end(), [&source](const Printing::PrintingLayerGroup &layer) {
                return std::find(layer.layers.begin(), layer.layers.end(), &source) != layer.layers.end();
            }));
        if (shared && source.scaled_print_z() > first_z)
            CHECK(std::none_of(first.layers.begin(), first.layers.end(), [&source](const Printing::PrintingLayerGroup &layer) {
                return std::find(layer.layers.begin(), layer.layers.end(), &source) != layer.layers.end();
            }));
    }
    REQUIRE(auxiliary_extrusions > 0);
    if (shared)
        CHECK(first.layers.back().print_z == first_z);
    }
}

// Exercise a boundary on and between layers, below a step and above all objects.
// Independent source enumeration checks each extrusion and auxiliary owner once.
TEST_CASE("By object height limit merges the remainder by layer", "[plugins][printing-plan][ordering][object-step][object-step-max-z]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const double limit : {0., 0.1, 0.4, 0.45, 0.6, 0.7, 10.}) {
        for (const double step : {0., 0.5}) {
            CAPTURE(limit, step);
            DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
            config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
                {"complete_objects_step", std::to_string(step)}, {"complete_objects_step_max_z", std::to_string(limit)},
                {"complete_objects_sort", "object"}, {"skirts", "0"}, {"brim_width", "1"},
                {"brim_per_object", "1"}, {"wipe_tower", "0"}, {"first_layer_height", "0.2"}, {"layer_height", "0.2"}});
            Model model;
            Print print;
            Test::init_print({make_cube(3., 3., 1.4), make_cube(3., 3., 0.8)}, print, model, config);
            model.objects()[0].instances.front()->set_offset(Vec3d(40., 40., 0.));
            model.objects()[0].add_instance(*model.objects()[0].instances.front())->set_offset(Vec3d(80., 40., 0.));
            model.objects()[1].instances.front()->set_offset(Vec3d(120., 40., 0.));
            print.apply(model, config);
            print.process();
            check_sequence_builder(print, true);
            const Printing::PrintingPlan &plan = *print.printing_plan();
            const bool final = limit > 0. && limit <= 1.4;
            if (final) {
                REQUIRE(plan.groups.back().printing_catalog_ids.size() == 3);
                coord_t previous = 0;
                for (const Printing::PrintingLayerGroup &layer : plan.groups.back().layers) {
                    CHECK(layer.print_z >= scale_to_layer_coord(limit));
                    CHECK(layer.print_z > previous);
                    previous = layer.print_z;
                }
                for (size_t idx = 0; idx + 1 < plan.groups.size(); ++idx)
                    for (const Printing::PrintingLayerGroup &layer : plan.groups[idx].layers)
                        CHECK(layer.print_z < scale_to_layer_coord(limit));
            } else {
                CHECK(plan.groups.back().printing_catalog_ids.size() == 1);
            }
        }
    }
}

// Mark generated support for the shifted second copy and place the limit below the global skirt.
// Require real extrusion sources of both kinds and exactly one final-group visit
// for every source role; the shared checker also checks all other source placements.
TEST_CASE("Height limit retains global and owned auxiliary extrusions in the final group", "[plugins][printing-plan][ordering][object-step][object-step-max-z]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = Test::SupportGrid::config("grid", false, 3, true);
    config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
        {"complete_objects_step", "0.5"}, {"complete_objects_step_max_z", "0.1"},
        {"filament_diameter", "1.75,1.75"}, {"skirts", "1"}, {"skirt_height", "1"},
        {"complete_objects_one_skirt", "1"}, {"min_skirt_length", "0"}});
    Model model;
    Print print;
    Test::init_print({Test::SupportGrid::slab(true)}, print, model, config);
    model.objects().front().instances.front()->set_offset(Vec3d(40., 40., 0.));
    model.objects().front().add_instance(*model.objects().front().instances.front())->set_offset(Vec3d(110., 60., 0.));
    print.apply(model, config);
    print.process();
    using Source = std::pair<const LayerRegionIsland *, ExtrusionRole>;
    std::set<Source> global_sources, owned_sources;
    const coord_t limit = scale_to_layer_coord(0.1);
    const coord_t first_z = print.object(0).layer(0).scaled_print_z();
    const ExtrusionRole roles[] = {LayerRegionIsland::SUPPORT, LayerRegionIsland::SUPPORT_INTERFACE,
        LayerRegionIsland::PERIMETERS, LayerRegionIsland::GAP_FILLS, LayerRegionIsland::INFILLS,
        LayerRegionIsland::IRONINGS, LayerRegionIsland::MILLS};
    REQUIRE(print.auxiliary_object() != nullptr);
    for (const Layer &layer : print.auxiliary_object()->auxiliary_layers())
        if (layer.scaled_print_z() >= limit)
            for (const LayerSliceIsland &slice : layer.islands())
                for (const LayerRegionIsland &island : slice.regions_islands())
                    for (const ExtrusionRole role : roles)
                        if (island.has_extrusion(role) && island.extrusion(role).total_volume() > 0.)
                            global_sources.emplace(&island, role);
    for (const std::unique_ptr<Layer> &layer : print.object(0).mutable_auxiliary_layers()) {
        if (layer->scaled_print_z() < limit || layer->scaled_print_z() <= first_z)
            continue;
        layer->get_or_add_property<LayerInstanceProperty>().object_instance_idx = 1;
        REQUIRE(layer->get_property<LayerInstanceProperty>() != nullptr);
        for (const LayerSliceIsland &slice : layer->islands())
            for (const LayerRegionIsland &island : slice.regions_islands())
                for (const ExtrusionRole role : roles)
                    if (island.has_extrusion(role) && island.extrusion(role).total_volume() > 0.)
                        owned_sources.emplace(&island, role);
    }
    REQUIRE_FALSE(global_sources.empty());
    REQUIRE_FALSE(owned_sources.empty());
    check_sequence_builder(print, true);
    const Printing::PrintingPlan &plan = *print.printing_plan();
    REQUIRE_FALSE(plan.groups.empty());
    std::map<Source, size_t> final_visits;
    for (const Printing::PrintingLayerGroup &layer : plan.groups.back().layers)
        for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
            for (const Printing::PrintingExtrusion &extrusion : tool.extrusions) {
                ++final_visits[{extrusion.region_island, extrusion.sregion_island_role}];
                if (owned_sources.count({extrusion.region_island, extrusion.sregion_island_role}) != 0) {
                    REQUIRE(extrusion.printing_catalog_id < plan.object_instances.size());
                    const Printing::PrintingObjectInstance &owner = plan.object_instances[extrusion.printing_catalog_id];
                    CHECK(owner.object == &print.object(0));
                    CHECK(owner.object_instance_idx == 1);
                    REQUIRE(extrusion.root != nullptr);
                    const Point shift = print.object(0).instances()[1].shift;
                    const ExtrusionEntity &source = extrusion.region_island->extrusion(extrusion.sregion_island_role);
                    CHECK(extrusion.root->first_point() == source.first_point() + shift);
                    CHECK(extrusion.root->last_point() == source.last_point() + shift);
                }
            }
    for (const Source &source : global_sources)
        CHECK(final_visits[source] == 1);
    for (const Source &source : owned_sources)
        CHECK(final_visits[source] == 1);
}

// Reject zero scaled slice heights, then invoke the builder directly to prove its
// whole-object fallback terminates and retains every source layer and extrusion.
TEST_CASE("By object rejects steps with zero scaled slice height", "[plugins][print-sequence][validate][object-step]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const std::pair<double, double> values : {std::make_pair(0.5, 0.), std::make_pair(1e-12, 1.), std::make_pair(0., 0.)}) {
        CAPTURE(values.first, values.second);
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
            {"skirts", "0"}, {"brim_width", "0"}, {"wipe_tower", "0"}});
        config.set_key_value("complete_objects_step", new ConfigOptionFloat(values.first));
        config.set_key_value("extruder_clearance_height", new ConfigOptionFloat(values.second));
        Model model;
        Print print;
        Test::init_print({make_cube(3., 3., 1.)}, print, model, config);
        const std::pair<PrintBase::PrintValidationError, std::string> validation = print.validate();
        CHECK(validation.first == PrintBase::PrintValidationError::pveWrongSettings);
        CHECK(validation.second == "The By object height lead must be greater than zero at layer precision: the height step capped by the extruder clearance height, or the extruder clearance height when the step is zero.");
        print.process();
        check_sequence_builder(print, true);
        REQUIRE(print.printing_plan() != nullptr);
        CHECK(print.printing_plan()->groups.size() == 1);
    }
}

// Close and tall copies retain their warning contours without blocking slicing;
// a height step does not add a wipe-tower validation restriction.
TEST_CASE("By object clearance warns without rejecting copies or stepped towers", "[plugins][print-sequence][validation][object-step]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const double step = GENERATE(0., 0.5);
    const bool islands = GENERATE(false, true);
    for (const double distance : {8., 30.}) {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
            {"complete_objects_step", std::to_string(step)}, {"complete_islands", islands ? "1" : "0"}, {"extruder_clearance_height", "1"},
            {"extruder_clearance_radius", "10"}, {"skirts", "0"}, {"brim_width", "0"}, {"wipe_tower", "0"}});
        Model model;
        Print print;
        Test::init_print({make_cube(3., 3., 2.)}, print, model, config);
        ModelObject &object = model.objects().front();
        object.instances.front()->set_offset(Vec3d(40., 40., 0.));
        object.add_instance(*object.instances.front())->set_offset(Vec3d(40. + distance, 40., 0.));
        print.apply(model, config);
        std::vector<std::string> warnings;
        const auto result = print.validate(&warnings);
        CAPTURE(step, islands, distance, result.second);
        CHECK(result.first == PrintBase::PrintValidationError::pveNone);
        CHECK(std::count(warnings.begin(), warnings.end(), "_COPIES_TOO_CLOSE") == (distance < 10. ? 1 : 0));
        CHECK(print.get_sequential_print_clearance_contours().empty() == (distance >= 10.));
        config.set_deserialize_strict("extruder_clearance_radius", "0");
        config.set_deserialize_strict("extruder_clearance_height", "0");
        print.apply(model, config);
        warnings.clear();
        CHECK(print.validate(&warnings).first == PrintBase::PrintValidationError::pveWrongSettings);
        CHECK(print.get_sequential_print_clearance_contours().empty());
        CHECK(warnings.empty());
        config.set_deserialize_strict("extruder_clearance_height", "1");
        print.apply(model, config);
        warnings.clear();
        CHECK(print.validate(&warnings).first == PrintBase::PrintValidationError::pveNone);
        CHECK(print.get_sequential_print_clearance_contours().empty());
        CHECK(warnings.empty());
    }
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
        {"complete_objects_step", "0.5"}, {"wipe_tower", "1"}, {"skirts", "0"},
        {"perimeter_extruder", "1"}, {"infill_extruder", "2"}, {"solid_infill_extruder", "2"},
        {"nozzle_diameter", "0.4,0.4"}, {"filament_diameter", "1.75,1.75"}});
    Model model;
    Print print;
    Test::init_print({make_cube(3., 3., 1.)}, print, model, config);
    REQUIRE(print.has_wipe_tower());
    CHECK(print.validate().first == PrintBase::PrintValidationError::pveNone);
    config.set_deserialize_strict("complete_objects_step", "0");
    print.apply(model, config);
    CHECK(print.validate().second.empty());
}

// Independent warning tokens preserve clearance, clamped travel height and disabled
// support notices together; the support action belongs only to its own notice.
TEST_CASE("Clearance, travel height and disabled-support notices coexist in validation", "[plugins][validation][print-sequence][lift]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
        {"complete_objects_step", "0"}, {"extruder_clearance_radius", "10"},
        {"extruder_clearance_height", "20"}, {"support_material", "0"},
        {"lift_min", "12"}, {"max_print_height", "10"},
        {"skirts", "0"}, {"brim_width", "0"}, {"wipe_tower", "0"}});
    Model model;
    Print print;
    Test::init_print({make_cube(3., 3., 2.)}, print, model, config);
    ModelObject &object = model.objects().front();
    object.instances.front()->set_offset(Vec3d(40., 40., 0.));
    object.add_instance(*object.instances.front())->set_offset(Vec3d(48., 40., 0.));
    object.add_volume(make_cube(1., 1., 1.))->set_type(ModelVolumeType::SUPPORT_ENFORCER);
    print.apply(model, config);
    std::vector<std::string> warnings;
    CHECK(print.validate(&warnings).first == PrintBase::PrintValidationError::pveNone);
    CHECK(std::count(warnings.begin(), warnings.end(), "_SUPPORTS_OFF") == 1);
    CHECK(std::count(warnings.begin(), warnings.end(), "_COPIES_TOO_CLOSE") == 1);
    CHECK(std::count(warnings.begin(), warnings.end(), "_LIFT_MIN_CLAMPED") == 1);
    config.set_deserialize_strict("lift_min", "10");
    print.apply(model, config);
    warnings.clear();
    CHECK(print.validate(&warnings).first == PrintBase::PrintValidationError::pveNone);
    CHECK(std::count(warnings.begin(), warnings.end(), "_SUPPORTS_OFF") == 1);
    CHECK(std::count(warnings.begin(), warnings.end(), "_COPIES_TOO_CLOSE") == 1);
    CHECK(std::count(warnings.begin(), warnings.end(), "_LIFT_MIN_CLAMPED") == 0);
}

// The preferred arrange distance retains the radius but excludes a skirt that
// becomes global with a positive step, at bed level and above it.
TEST_CASE("By object preferred spacing excludes per-copy skirts with a step", "[plugins][skirt-brim][print-sequence][object-step]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
        {"complete_objects_step", "0"}, {"complete_objects_one_skirt", "0"},
        {"extruder_clearance_radius", "10"}, {"skirts", "1"}, {"skirt_height", "2"},
        {"skirt_distance", "6"}, {"brim_width", "0"}});
    CHECK(min_object_distance(&config, 0.) > 16.);
    CHECK(min_object_distance(&config, 0.2) > 16.);
    config.set_deserialize_strict("complete_objects_step", "0.5");
    for (const bool one_skirt : {false, true}) {
        config.set_deserialize_strict("complete_objects_one_skirt", one_skirt ? "1" : "0");
        CHECK(min_object_distance(&config, 0.) == Approx(10.));
        CHECK(min_object_distance(&config, 0.2) == Approx(10.));
    }
}

// Change only the builder on a processed support-enabled print and require all
// mode-dependent producers to run again while the source slices remain cached.
TEST_CASE("Changing the print sequence replays adhesion support and plan construction", "[plugins][config][print-sequence][invalidation]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_layer"},
        {"complete_objects_step", "0"}, {"skirts", "1"}, {"brim_width", "1"},
        {"support_material", "1"}, {"support_material_enforce_layers", "1"}, {"support_material_style", "grid"}});
    Model model;
    Print print;
    Test::init_print({make_cube(5., 5., 1.), make_cube(5., 5., 1.)}, print, model, config);
    const boost::filesystem::path output = Test::GCode::temporary_gcode_path();
    struct Cleanup {
        boost::filesystem::path path;
        // Remove both export files on success or assertion unwinding.
        ~Cleanup() { Test::GCode::remove_output_pair(path); }
    } cleanup{output};
    print.process();
    host.export_gcode_cli(print, output.string());
    const Layer *source = &print.object(0).layer(0);
    std::vector<std::pair<Plugin *, uint64_t>> counts;
    for (const char *id : {"skirt_brim.skirt.default", "skirt_brim.brim.default", "support.family.grid", "ordering.plan_builder.by_object"}) {
        Plugin *plugin = host.get_plugin(id);
        REQUIRE(plugin != nullptr);
        counts.emplace_back(plugin, plugin->get_run_count());
    }
    DynamicPrintConfig changed = print.full_print_config();
    changed.set_deserialize_strict("print_plan_builder", "ordering.plan_builder.by_object");
    REQUIRE(print.full_print_config().diff(changed) == t_config_option_keys{"print_plan_builder"});
    REQUIRE(print.apply(model, changed) == Print::APPLY_STATUS_INVALIDATED);
    CHECK(print.current_checkpoint() == Steps::Checkpoint::Slicing);
    print.process();
    host.export_gcode_cli(print, output.string());
    CHECK(&print.object(0).layer(0) == source);
    for (const auto &entry : counts) {
        CAPTURE(entry.first->get_id());
        CHECK(entry.first->get_run_count() > entry.second);
    }
    REQUIRE(print.printing_plan() != nullptr);
    CHECK(print.printing_plan()->groups.size() >= 2);
}

// Resolve actual active builders rather than trusting a stored selector; restore
// global activation on every exit so other slicing cases keep their providers.
TEST_CASE("Print sequence helpers follow the effective active builder", "[plugins][config][print-sequence][step-selector]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    REQUIRE(FullPrintConfig::defaults().option("print_plan_builder") != nullptr);
    Orchestrator &host = Orchestrator::instance();
    struct Activation {
        Orchestrator &host;
        std::vector<std::pair<std::string, bool>> saved;
        // Restore borrowed host activation flags, including when a Catch assertion exits early.
        ~Activation() { for (const auto &entry : saved) host.set_plugin_active(entry.first, entry.second); }
    } guard{host, {}};
    for (Plugin *plugin : host.registered_plugins())
        if (plugin->get_exclusive_group() == "ordering.plan_builder")
            guard.saved.emplace_back(plugin->get_id(), host.is_plugin_active(plugin));
    REQUIRE(guard.saved.size() == 2);
    for (const char *id : {"ordering.plan_builder.by_layer", "ordering.plan_builder.by_object",
         "skirt_brim.skirt.default", "skirt_brim.brim.default", "ordering.wipe_into_first.default", "wipe_tower.manager.default"}) {
        CHECK(Test::Plugins::plugin_test_startup_sets().active_provider_ids.count(id) == 1);
    }
    for (unsigned active = 1; active != 4; ++active) {
        REQUIRE(host.set_plugin_active("ordering.plan_builder.by_layer", (active & 1) != 0));
        REQUIRE(host.set_plugin_active("ordering.plan_builder.by_object", (active & 2) != 0));
        for (const bool stored_object : {false, true}) {
            DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
            config.set_deserialize_strict("print_plan_builder", stored_object ? "ordering.plan_builder.by_object" : "ordering.plan_builder.by_layer");
            const bool expected_object = active == 2 || (active == 3 && stored_object);
            for (const double step : {0., 5.}) {
                config.set_key_value("complete_objects_step", new ConfigOptionFloat(step));
                CAPTURE(active, stored_object, step);
                CHECK(print_by_object(config) == expected_object);
                CHECK(print_complete_objects(config) == (expected_object && step == 0.));
                const BambuConfigTranslation translated = translate_config_to_bambu(config);
                REQUIRE(translated.entries.count("print_sequence") == 1);
                CHECK(translated.entries.at("print_sequence").values == std::vector<std::string>{expected_object ? "by object" : "by layer"});
            }
        }
    }
}

// Load historical modes through INI, project deserialization and embedded G-code;
// require selector precedence and retain the documented partial-file limitation.
TEST_CASE("Legacy print sequence uses the obsolete switch as its conversion marker", "[plugins][config][legacy][print-sequence]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedNatureDirectory directory;
    Model model;
    Print print;
    Test::init_print({make_cube(5., 5., 1.)}, print, model);
    struct Row { const char *text; const char *builder; double step; };
    const Row rows[] = {
        {"complete_objects = 1\n", "ordering.plan_builder.by_object", 0.},
        {"complete_objects = 1\nparallel_objects_step = 5\n", "ordering.plan_builder.by_object", 0.},
        {"complete_objects = 0\nparallel_objects_step = 5\n", "ordering.plan_builder.by_object", 5.},
        {"complete_objects = 0\n", "ordering.plan_builder.by_layer", 0.},
        {"parallel_objects_step = 5\n", "ordering.plan_builder.by_layer", 5.},
        {"print_plan_builder = ordering.plan_builder.by_layer\ncomplete_objects = 1\nparallel_objects_step = 5\n", "ordering.plan_builder.by_layer", 5.},
        {"print_plan_builder = ordering.plan_builder.by_layer\nparallel_objects_step = 5\n", "ordering.plan_builder.by_layer", 5.},
        {"print_plan_builder = ordering.plan_builder.by_object\ncomplete_objects = 0\n", "ordering.plan_builder.by_object", 0.}
    };
    for (const Row &row : rows) {
        CAPTURE(row.text);
        for (int loader = 0; loader != 4; ++loader) {
            CAPTURE(loader);
            DynamicPrintConfig config;
            if (loader == 0) {
                config.load_from_ini_string(row.text, ForwardCompatibilitySubstitutionRule::Disable);
            } else {
                std::istringstream input(row.text);
                std::string line;
                std::map<std::string, std::string> pairs;
                std::string gcode = "; generated by SuperSlicer 2.7.0\n\n; print_version = SUSI_2.7.0\n";
                while (std::getline(input, line)) {
                    const size_t separator = line.find(" = ");
                    pairs.emplace(line.substr(0, separator), line.substr(separator + 3));
                    gcode += "; " + line + "\n";
                }
                ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Disable);
                if (loader == 1)
                    deserialize_maybe_from_prusa(pairs, config, substitutions, true, false);
                else if (loader == 2)
                    ConfigBase::load_from_gcode_string_legacy(config, gcode.c_str(), substitutions);
                else {
                    const boost::filesystem::path archive = directory.path() / "legacy-sequence.3mf";
                    // Write old serialized text directly: the current writer cannot export an obsolete option definition.
                    REQUIRE(store_3mf(archive.string().c_str(), &model, nullptr, OptionStore3mf{}.set_export_config(false)));
                    REQUIRE(mz_zip_add_mem_to_archive_file_in_place(archive.string().c_str(), "Metadata/Slic3r.config",
                        gcode.data(), gcode.size(), nullptr, 0, MZ_DEFAULT_COMPRESSION));
                    Model loaded;
                    REQUIRE(load_3mf(archive.string().c_str(), config, substitutions, &loaded, false, false));
                }
            }
            CHECK_FALSE(config.has("complete_objects"));
            const ConfigOption *builder = config.option("print_plan_builder");
            CHECK((builder == nullptr ? "ordering.plan_builder.by_layer" : builder->serialize()) == std::string(row.builder));
            const ConfigOption *step = config.option("complete_objects_step");
            CHECK((step == nullptr ? 0. : step->get_float()) == row.step);
        }
    }
    std::string key = "complete_objects";
    std::string value = "1";
    PrintConfigDef::handle_legacy_pair(key, value);
    CHECK(key == "print_plan_builder");
    CHECK(value == "ordering.plan_builder.by_object");
    key = "complete_objects";
    value = "0";
    PrintConfigDef::handle_legacy_pair(key, value);
    CHECK(key == "print_plan_builder");
    CHECK(value == "ordering.plan_builder.by_layer");
}

// Load the old object-step names through profile and project readers, then verify
// that both writers retain their values using only the current names.
TEST_CASE("Object step aliases load and save with current names", "[plugins][config][legacy][object-step-aliases]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedNatureDirectory directory;
    Model model;
    Print print;
    Test::init_print({make_cube(5., 5., 1.)}, print, model);
    const std::string legacy = "parallel_objects_step = 2.5\nparallel_objects_step_max_z = 8\nparallel_islands = 1\n";
    for (const bool project : {false, true}) {
        CAPTURE(project);
        DynamicPrintConfig config;
        if (!project) {
            config.load_from_ini_string(legacy, ForwardCompatibilitySubstitutionRule::Disable);
        } else {
            const boost::filesystem::path archive = directory.path() / "old-object-step.3mf";
            const std::string text = "; generated by SuperSlicer 2.7.0\n\n; print_version = SUSI_2.7.0\n"
                "; parallel_objects_step = 2.5\n; parallel_objects_step_max_z = 8\n; parallel_islands = 1\n";
            REQUIRE(store_3mf(archive.string().c_str(), &model, nullptr, OptionStore3mf{}.set_export_config(false)));
            REQUIRE(mz_zip_add_mem_to_archive_file_in_place(archive.string().c_str(), "Metadata/Slic3r.config",
                text.data(), text.size(), nullptr, 0, MZ_DEFAULT_COMPRESSION));
            ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Disable);
            Model loaded;
            REQUIRE(load_3mf(archive.string().c_str(), config, substitutions, &loaded, false, false));
        }
        REQUIRE(config.option("complete_objects_step") != nullptr);
        REQUIRE(config.option("complete_objects_step_max_z") != nullptr);
        REQUIRE(config.option("complete_islands") != nullptr);
        CHECK(config.opt_float("complete_objects_step") == 2.5);
        CHECK(config.opt_float("complete_objects_step_max_z") == 8.);
        CHECK(config.opt_bool("complete_islands"));
        for (const char *key : {"parallel_objects_step", "parallel_objects_step_max_z", "parallel_islands"})
            CHECK_FALSE(config.has(key));
        const boost::filesystem::path profile = directory.path() / "object-step.ini";
        config.save(profile.string(), ConfigWriteTarget::Local);
        const boost::filesystem::path saved = directory.path() / "object-step.3mf";
        REQUIRE(store_3mf(saved.string().c_str(), &model, &config));
        const std::string outputs[] = {Test::GCode::read_text_file(profile),
            read_archive_entry(saved, "Metadata/SuperSlicer.config")};
        for (const std::string &text : outputs) {
            CHECK(text.find("complete_objects_step = 2.5") != std::string::npos);
            CHECK(text.find("complete_objects_step_max_z = 8") != std::string::npos);
            CHECK(text.find("complete_islands = 1") != std::string::npos);
            CHECK(text.find("parallel_objects_step") == std::string::npos);
            CHECK(text.find("parallel_islands") == std::string::npos);
        }
    }
}

// Keep object-setting display precision and replay skirt generation when its optional height-step input changes.
TEST_CASE("Object settings preserve precision and replay their earliest readers", "[plugins][config][print-sequence][invalidation]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const char *key : { "complete_objects_sort", "complete_objects_step", "complete_objects_step_max_z", "complete_islands" }) {
        CAPTURE(key);
        const ConfigOptionDef *definition = PrintConfigDef::instance().get(key);
        REQUIRE(definition != nullptr);
        CHECK(definition->precision == 6);
        CHECK(definition->invalidates_step == (std::string(key) == "complete_objects_step" ? STEP_SUPPORT : STEP_ORDERING));
    }
    const std::optional<Steps::Checkpoint> checkpoint = Steps::config_option_replay_checkpoint("complete_objects_step");
    REQUIRE(checkpoint.has_value());
    CHECK(*checkpoint == Steps::Checkpoint::Slicing);
    CHECK(*checkpoint == Steps::checkpoint_before_step(STEP_SKIRT_BRIM));
}

// Resolve every serialized object order independently of the plugin enum's integer layout.
TEST_CASE("Object sort helper maps serialized names", "[plugins][config][print-sequence]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config;
    CHECK(complete_objects_sort(config) == cosObject);
    const std::pair<const char *, CompleteObjectSort> orders[] = {
        { "object", cosObject }, { "lowy", cosY }, { "lowz", cosZ }, { "nearest", cosNearest },
        { "lowx", cosX }, { "highx", cosMaxX }, { "highy", cosMaxY }
    };
    for (const auto &order : orders) {
        config.set_deserialize_strict("complete_objects_sort", order.first);
        CHECK(complete_objects_sort(config) == order.second);
    }
}

// Keep the dropdown's serialized names and labels explicit; presets and projects must retain each new choice unchanged.
TEST_CASE("Object XY sort choices persist through INI and 3MF", "[plugins][config][print-sequence][object-sort]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ConfigOptionDef *definition = PrintConfigDef::instance().get("complete_objects_sort");
    REQUIRE(definition != nullptr);
    REQUIRE(definition->enum_def.get() != nullptr);
    const std::vector<std::string> values{"object", "lowx", "highx", "lowy", "highy", "lowz", "nearest"};
    const std::vector<std::string> labels{"Right panel", "Min X", "Max X", "lowest Y", "Max Y", "lowest Z", "Nearest"};
    CHECK(definition->enum_def->values() == values);
    CHECK(definition->enum_def->labels() == labels);
    CHECK(definition->default_value->serialize() == "object");
    CHECK(definition->invalidates_step == STEP_ORDERING);
    ScopedNatureDirectory directory;
    Model model;
    Print print;
    Test::init_print({make_cube(2., 2., 1.)}, print, model);
    for (const char *value : {"lowx", "highx", "highy"}) {
        CAPTURE(value);
        DynamicPrintConfig saved;
        saved.set_deserialize_strict("complete_objects_sort", value);
        const boost::filesystem::path ini = directory.path() / "object-sort.ini";
        saved.save(ini.string(), ConfigWriteTarget::Local);
        DynamicPrintConfig loaded_ini;
        loaded_ini.load(ini.string(), ForwardCompatibilitySubstitutionRule::EnableSilent);
        CHECK(loaded_ini.opt_serialize("complete_objects_sort") == value);
        const boost::filesystem::path archive = directory.path() / "object-sort.3mf";
        REQUIRE(store_3mf(archive.string().c_str(), &model, &saved));
        DynamicPrintConfig loaded;
        Model loaded_model;
        ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::EnableSilent};
        REQUIRE(load_3mf(archive.string().c_str(), loaded, substitutions, &loaded_model, false, false));
        CHECK(loaded.opt_serialize("complete_objects_sort") == value);
    }
}

// Start without the object provider, retain dependent providers and build filtered scene defaults;
// slice and export with missing object settings so optional readers exercise their defaults.
TEST_CASE("Inactive object builder leaves scene defaults and other providers usable", "[.inactive-builder-settings]")
{
    using namespace Slic3r;
    REQUIRE(std::getenv("SLIC3R_TEST_WITHOUT_BY_OBJECT") != nullptr);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    REQUIRE_FALSE(host.is_plugin_active(host.get_plugin("ordering.plan_builder.by_object")));
    for (const char *id : { "skirt_brim.skirt.default", "skirt_brim.brim.default",
                           "wipe_tower.manager.default", "ordering.wipe_into_first.default" }) {
        CAPTURE(id);
        REQUIRE(host.get_plugin(id) != nullptr);
        CHECK(host.is_plugin_active(host.get_plugin(id)));
    }
    for (const char *key : { "complete_objects_sort", "complete_objects_step", "complete_objects_step_max_z", "complete_islands" })
        CHECK_FALSE(PrintConfigDef::instance().has(key));
    CHECK_FALSE(PrintConfigDef::instance().has("complete_objects_one_brim"));
    CHECK(PrintConfigDef::instance().has("complete_objects_one_skirt"));
    std::vector<std::string> scene_keys = { "bed_shape", "print_plan_builder", "complete_objects_step",
        "complete_objects_step_max_z", "complete_islands", "complete_objects_sort",
        "complete_objects_one_skirt", "brim_per_object", "duplicate_distance", "extruder_clearance_radius" };
    scene_keys.erase(std::remove_if(scene_keys.begin(), scene_keys.end(), [](const std::string &key) {
        return !PrintConfigDef::instance().has(key);
    }), scene_keys.end());
    const std::unique_ptr<DynamicPrintConfig> scene(DynamicPrintConfig::new_from_defaults_keys(scene_keys));
    CHECK(scene->has("bed_shape"));
    CHECK(complete_objects_sort(*scene) == cosObject);
    CHECK_FALSE(print_by_object(*scene));
    CHECK_FALSE(print_complete_objects(*scene));
    const slic3r_api::Config view(ApiHost::to_config_handle(scene.get()));
    CHECK(view.float_or_default("complete_objects_step", 0.) == 0.);
    CHECK(view.float_or_default("complete_objects_step_max_z", 0.) == 0.);
    CHECK_FALSE(view.bool_or_default("complete_islands", false));
    Model model;
    Print print;
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({ {"skirts", "1"}, {"brim_width", "1"}, {"fill_density", "0%"} });
    Test::init_print({make_cube(5., 5., 1.)}, print, model, config);
    print.process();
    const boost::filesystem::path path = Test::GCode::temporary_gcode_path();
    host.export_gcode_cli(print, path.string());
    CHECK_FALSE(Test::GCode::read_text_file(path).empty());
    Test::GCode::remove_output_pair(path);
}

// Import a plate selection absent from the startup enum, requiring the reader's
// supplied substitution context to report the fallback instead of throwing.
TEST_CASE("Bambu plate sequence imports with an inactive object builder", "[.inactive-builder-import]")
{
    using namespace Slic3r;
    REQUIRE(std::getenv("SLIC3R_TEST_WITHOUT_BY_OBJECT") != nullptr);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    REQUIRE_FALSE(host.is_plugin_active(host.get_plugin("ordering.plan_builder.by_object")));
    const ConfigOptionDef *definition = PrintConfigDef::instance().get("print_plan_builder");
    REQUIRE(definition != nullptr);
    REQUIRE(definition->enum_def);
    CHECK(std::find(definition->enum_def->values().begin(), definition->enum_def->values().end(), "ordering.plan_builder.by_object") == definition->enum_def->values().end());
    ScopedNatureDirectory directory;
    Model model;
    Print print;
    Test::init_print({make_cube(5., 5., 1.)}, print, model);
    const boost::filesystem::path archive = directory.path() / "inactive-builder.3mf";
    REQUIRE(store_3mf(archive.string().c_str(), &model, nullptr, OptionStore3mf{}.set_export_config(false)));
    const std::string metadata = "<config><plate><metadata key=\"plater_id\" value=\"1\"/><metadata key=\"print_sequence\" value=\"by object\"/></plate></config>";
    REQUIRE(mz_zip_add_mem_to_archive_file_in_place(archive.string().c_str(), "Metadata/model_settings.config", metadata.data(), metadata.size(), nullptr, 0, MZ_DEFAULT_COMPRESSION));
    Model loaded;
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Enable);
    PlateDataPtrs plates;
    struct Cleanup {
        PlateDataPtrs &plates;
        // Release the imported plate records owned by this test, including on assertion failure.
        ~Cleanup() { for (PlateData *plate : plates) delete plate; }
    } cleanup{plates};
    std::vector<Preset *> presets;
    bool bambu = false;
    Semver version;
    REQUIRE(load_bbs_3mf(archive.string().c_str(), &config, &substitutions, &loaded, &plates, &presets, &bambu, &version, nullptr, LoadStrategy::LoadModel | LoadStrategy::LoadConfig));
    REQUIRE(plates.size() == 1);
    CHECK(plates.front()->config.opt_serialize("print_plan_builder") == "ordering.plan_builder.by_layer");
    CHECK_FALSE(substitutions.get().empty());
    CHECK(presets.empty());
}

// The three accumulated metadata keys of older projects are obsolete: a plain configuration file reads them silently
// and does not keep them, while the 3MF reader reports them as obsolete keys.
TEST_CASE("The obsolete accumulated metadata keys are ignored",
          "[plugins][config][project]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *const keys[] = {"compatible_printers_condition_cummulative", "compatible_prints_condition_cummulative", "inherits_cummulative"};

    Slic3r::DynamicPrintConfig ini_config;
    const Slic3r::ConfigSubstitutions ini_substitutions = ini_config.load_from_ini_string(
        "compatible_printers_condition_cummulative = a\ncompatible_prints_condition_cummulative = b\ninherits_cummulative = c\n",
        Slic3r::ForwardCompatibilitySubstitutionRule::Enable);
    CHECK(ini_substitutions.empty());
    for (const char *key : keys) {
        CAPTURE(key);
        CHECK_FALSE(ini_config.has(key));
    }

    Slic3r::DynamicPrintConfig project_config;
    Slic3r::ConfigSubstitutionContext substitutions{Slic3r::ForwardCompatibilitySubstitutionRule::Enable};
    Slic3r::deserialize_maybe_from_prusa({{keys[0], "a"}, {keys[1], "b"}, {keys[2], "c"}}, project_config, substitutions, true, false);
    CHECK(substitutions.get().size() == 3);
    for (const char *key : keys) {
        CAPTURE(key);
        CHECK_FALSE(project_config.has(key));
    }
}
// A key without definition has no policy: it is written to every destination, by a configuration with a definition
// that does not know the key, and by a configuration that has no definition at all.
TEST_CASE("ConfigBase save writes the keys that have no definition",
          "[plugins][config][nature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedNatureDirectory directory;
    const NatureKeyExpectation undefined_key = {"undefined_nature_key", "undefined_value", true, true, true};
    const Slic3r::ConfigWriteTarget targets[] = {Slic3r::ConfigWriteTarget::Local, Slic3r::ConfigWriteTarget::Project,
                                                 Slic3r::ConfigWriteTarget::GCode};

    SECTION("a key unknown to the definition of a print configuration")
    {
        Slic3r::DynamicPrintConfig config;
        config.set_key_value(undefined_key.key, new Slic3r::ConfigOptionString(undefined_key.value));
        REQUIRE(Slic3r::PrintConfigDef::instance().get(undefined_key.key) == nullptr);
        for (size_t target = 0; target < 3; ++target) {
            CAPTURE(target);
            CHECK(config.writes_key(undefined_key.key, targets[target]));
            const boost::filesystem::path file = directory.path() / ("unknown" + std::to_string(target) + ".ini");
            config.save(file.string(), targets[target]);
            CHECK(has_config_line(Slic3r::Test::GCode::read_text_file(file), "", undefined_key));
        }
    }

    SECTION("a configuration without definition")
    {
        Slic3r::DynamicConfig config;
        REQUIRE(config.def() == nullptr);
        config.set_key_value(undefined_key.key, new Slic3r::ConfigOptionString(undefined_key.value));
        // Even a key that a definition would refuse is written: without definition there is no policy.
        config.set_key_value("test_config_nature_volatile", new Slic3r::ConfigOptionInt(15));
        const NatureKeyExpectation volatile_key = {"test_config_nature_volatile", "15", true, true, true};
        for (size_t target = 0; target < 3; ++target) {
            CAPTURE(target);
            CHECK_NOTHROW(config.writes_key(undefined_key.key, targets[target]));
            CHECK(config.writes_key(undefined_key.key, targets[target]));
            const boost::filesystem::path file = directory.path() / ("nodef" + std::to_string(target) + ".ini");
            CHECK_NOTHROW(config.save(file.string(), targets[target]));
            const std::string text = Slic3r::Test::GCode::read_text_file(file);
            CHECK(has_config_line(text, "", undefined_key));
            CHECK(has_config_line(text, "", volatile_key));
        }
    }
}

// compares_key follows the nature of the definition: every nature but BOOKKEEPING and VOLATILE is comparable, and a key or a
// configuration without definition has no policy.
TEST_CASE("Only the comparable natures take part in comparisons",
          "[plugins][config][compare]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::pair<const char *, bool> expectations[] = {
        {"layer_height", true}, {"print_host", true}, {"host_type", true}, {"inherits", true}, {"compatible_printers", true},
        {"printer_model", true}, {"print_settings_id", false}, {"filament_settings_modified", false},
        {"print_version", false}, {"test_config_nature_normal", true}, {"test_config_nature_local", true},
        {"test_config_nature_bookkeeping", false}, {"test_config_nature_preset_meta", true},
        {"test_config_nature_volatile", false}, {"undefined_nature_key", true},
    };
    Slic3r::DynamicPrintConfig config;
    for (const std::pair<const char *, bool> &expectation : expectations) {
        CAPTURE(expectation.first);
        CHECK_NOTHROW(config.compares_key(expectation.first));
        CHECK(config.compares_key(expectation.first) == expectation.second);
    }
    // Without definition there is no policy, even for a key that a definition refuses.
    Slic3r::DynamicConfig without_definition;
    REQUIRE(without_definition.def() == nullptr);
    CHECK(without_definition.compares_key("print_settings_id"));
    CHECK(without_definition.compares_key("test_config_nature_volatile"));
}

// equals ignores the keys that the nature refuses and the keys present on one side only; diff stays a plain difference of values.
TEST_CASE("Equality compares the comparable keys present in both configurations",
          "[plugins][config][compare]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    SECTION("a BOOKKEEPING key alone does not make a difference")
    {
        Slic3r::DynamicPrintConfig left, right;
        left.set_deserialize_strict("print_settings_id", "Left");
        right.set_deserialize_strict("print_settings_id", "Right");
        CHECK(left.equals(right));
        CHECK(left.equals(right, false));
        // The difference of values is not filtered: it decides what a caller copies.
        CHECK(left.diff(right) == Slic3r::t_config_option_keys{"print_settings_id"});
    }
    SECTION("a LOCAL or PRESET_META key makes a difference")
    {
        Slic3r::DynamicPrintConfig left, right;
        left.set_deserialize_strict("print_host", "Left");
        right.set_deserialize_strict("print_host", "Right");
        CHECK_FALSE(left.equals(right));
        CHECK(left.diff(right) == Slic3r::t_config_option_keys{"print_host"});
        Slic3r::DynamicPrintConfig meta_left, meta_right;
        meta_left.set_deserialize_strict("inherits", "Left");
        meta_right.set_deserialize_strict("inherits", "Right");
        CHECK_FALSE(meta_left.equals(meta_right));
    }
    SECTION("a NORMAL key makes a difference")
    {
        Slic3r::DynamicPrintConfig left, right;
        left.set_deserialize_strict("layer_height", "0.2");
        right.set_deserialize_strict("layer_height", "0.3");
        CHECK_FALSE(left.equals(right));
    }
    SECTION("a key present on one side only is not a difference, an empty value is a value")
    {
        Slic3r::DynamicPrintConfig left, right;
        left.set_deserialize_strict("print_host", "host");
        left.set_deserialize_strict("compatible_printers_condition", "");
        CHECK(left.equals(right));
        CHECK(right.equals(left));
        right.set_deserialize_strict("compatible_printers_condition", "nozzle_diameter[0]==0.4");
        CHECK_FALSE(left.equals(right));
    }
}

// The dirty state and the list of modified options follow the same rule, and no longer look at the presence of the compatibility lists.
TEST_CASE("The modified state ignores the keys that are not comparable",
          "[plugins][config][compare]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    NaturePreset reference;
    NaturePreset edited;
    reference.config.set_deserialize_strict("layer_height", "0.2");
    edited.config.set_deserialize_strict("layer_height", "0.2");

    SECTION("an identifier alone does not modify the preset")
    {
        reference.config.set_deserialize_strict("print_settings_id", "Reference");
        edited.config.set_deserialize_strict("print_settings_id", "Edited");
        reference.config.set_deserialize_strict("filament_settings_id", "A;B");
        edited.config.set_deserialize_strict("filament_settings_id", "A;C");
        CHECK_FALSE(Slic3r::PresetCollection::is_dirty(&edited, &reference));
        CHECK(Slic3r::PresetCollection::dirty_options(&edited, &reference).empty());
    }
    SECTION("the presence of the compatibility lists alone does not modify the preset")
    {
        edited.config.set_deserialize_strict("compatible_printers", "Printer");
        edited.config.set_deserialize_strict("compatible_prints", "");
        CHECK_FALSE(Slic3r::PresetCollection::is_dirty(&edited, &reference));
        CHECK(Slic3r::PresetCollection::dirty_options(&edited, &reference).empty());
        CHECK_FALSE(Slic3r::PresetCollection::is_dirty(&reference, &edited));
    }
    SECTION("a LOCAL or PRESET_META value modifies the preset and is listed")
    {
        reference.config.set_deserialize_strict("print_host", "Reference");
        edited.config.set_deserialize_strict("print_host", "Edited");
        reference.config.set_deserialize_strict("inherits", "Parent");
        edited.config.set_deserialize_strict("inherits", "Other parent");
        CHECK(Slic3r::PresetCollection::is_dirty(&edited, &reference));
        const std::map<Slic3r::OptionKeyIdx, uint16_t> options = Slic3r::PresetCollection::dirty_options(&edited, &reference);
        CHECK(options.size() == 2);
        CHECK(options.count(Slic3r::OptionKeyIdx::scalar("print_host")) == 1);
        CHECK(options.count(Slic3r::OptionKeyIdx::scalar("inherits")) == 1);
    }
    SECTION("a NORMAL vector value is still reported by index")
    {
        reference.config.set_deserialize_strict("nozzle_diameter", "0.4,0.4");
        edited.config.set_deserialize_strict("nozzle_diameter", "0.4,0.6");
        edited.config.set_deserialize_strict("print_settings_id", "Edited");
        CHECK(Slic3r::PresetCollection::is_dirty(&edited, &reference));
        const std::map<Slic3r::OptionKeyIdx, uint16_t> options = Slic3r::PresetCollection::dirty_options(&edited, &reference);
        CHECK(options.size() == 1);
        CHECK(options.count(Slic3r::OptionKeyIdx{"nozzle_diameter", 1}) == 1);
    }
    SECTION("a version that differs is ignored, even when one side is phony")
    {
        // print_version is BOOKKEEPING and can be phony: a phony side is a difference for the comparison of values,
        // but the nature is looked at first.
        reference.config.set_key_value("print_version", new Slic3r::ConfigOptionStringVersion("SUSI_2.7.0.0"));
        edited.config.set_key_value("print_version", new Slic3r::ConfigOptionStringVersion("SUSI_2.7.1.0"));
        CHECK_FALSE(Slic3r::PresetCollection::is_dirty(&edited, &reference));
        CHECK(Slic3r::PresetCollection::dirty_options(&edited, &reference).empty());
        edited.config.option("print_version")->set_phony(true);
        CHECK_FALSE(Slic3r::PresetCollection::is_dirty(&edited, &reference));
        CHECK(Slic3r::PresetCollection::dirty_options(&edited, &reference).empty());
        CHECK_FALSE(Slic3r::PresetCollection::is_dirty(&reference, &edited));
        CHECK(Slic3r::PresetCollection::dirty_options(&reference, &edited).empty());
    }
}
namespace {
// A project configuration as the loader builds it: the defaults with their optional values disabled, then the contents of the file.
Slic3r::DynamicPrintConfig project_like_config(const char *printer_name)
{
    Slic3r::DynamicPrintConfig config;
    config.apply(Slic3r::FullPrintConfig::defaults());
    config.disable_optionals();
    config.set_deserialize_strict({{"printer_technology", "FFF"},
                                   {"print_settings_id", "PrintX"},
                                   {"filament_settings_id", "FilX"},
                                   {"printer_settings_id", printer_name}});
    Slic3r::Preset::normalize(config);
    return config;
}

// Add a printer preset to the bundle, not selected and hidden until a loaded project selects it. It holds a hostname kind,
// a parent and a default print profile that a project or a G-code does not carry.
Slic3r::Preset &add_installed_printer(Slic3r::PresetBundle &bundle, const char *name)
{
    Slic3r::DynamicPrintConfig printer_config = bundle.printers.default_preset().config;
    printer_config.set_deserialize_strict("host_type", "octoprint");
    printer_config.opt_string("inherits", true) = "Parent printer";
    printer_config.opt_string("default_print_profile", true) = "Default print";
    Slic3r::Preset &preset = bundle.printers.load_preset("", name, printer_config, false);
    preset.is_visible = false;
    REQUIRE(bundle.printers.get_selected_preset_name() != name);
    return preset;
}

// Add a filament preset to the bundle, not selected. Its compatibility condition tells it apart from the other ones.
Slic3r::Preset &add_installed_filament(Slic3r::PresetBundle &bundle, const char *name, const char *parent, const char *condition, bool visible)
{
    Slic3r::DynamicPrintConfig filament_config = bundle.filaments.default_preset().config;
    filament_config.opt_string("inherits", true) = parent;
    filament_config.opt_string("compatible_printers_condition", true) = condition;
    Slic3r::Preset &preset = bundle.filaments.load_preset("", name, filament_config, false);
    preset.is_visible = visible;
    return preset;
}
} // namespace

// Recognizing an installed profile looks at the keys that a file of the G-code destination holds, with their default value when
// the file lacks them. The ones it does not hold, such as the hostname kind of a project, keep the value of the installed profile.
TEST_CASE("A loaded project recognizes the installed profile from the values that it holds",
          "[plugins][config][compare][project]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Slic3r::PresetBundle bundle;
    Slic3r::Preset &installed = add_installed_printer(bundle, "Installed printer");
    // The identifier of the installed profile differs from the one of the project: it is not comparable, so it is never copied.
    installed.config.set_deserialize_strict("printer_settings_id", "Installed sentinel");

    SECTION("the same values recognize the profile and keep its hostname kind")
    {
        Slic3r::DynamicPrintConfig config = project_like_config("Installed printer");
        bundle.load_config_model("project.3mf", config, "");
        CHECK(bundle.printers.get_selected_preset_name() == "Installed printer");
        CHECK_FALSE(bundle.printers.get_edited_preset().is_dirty);
        CHECK(bundle.printers.get_edited_preset().config.opt_serialize("host_type") == "octoprint");
        // The profile was hidden: loading it reports it as an installed profile that is now visible.
        CHECK(bundle.tmp_installed_presets.count("Installed printer") == 1);
        CHECK(bundle.printers.find_preset("Installed printer", false, false)->is_visible);
    }
    SECTION("a profile that is already visible is recognized as well")
    {
        bundle.printers.find_preset("Installed printer", false, false)->is_visible = true;
        Slic3r::DynamicPrintConfig config = project_like_config("Installed printer");
        bundle.load_config_model("project.3mf", config, "");
        CHECK(bundle.printers.get_selected_preset_name() == "Installed printer");
        CHECK_FALSE(bundle.printers.get_edited_preset().is_dirty);
        CHECK(bundle.printers.get_edited_preset().config.opt_serialize("host_type") == "octoprint");
    }
    SECTION("a different NORMAL value modifies the profile without changing its hostname kind")
    {
        Slic3r::DynamicPrintConfig config = project_like_config("Installed printer");
        config.set_deserialize_strict("printer_notes", "changed notes");
        config.set_deserialize_strict("printer_model", "Model X");
        bundle.load_config_model("project.3mf", config, "");
        CHECK(bundle.printers.get_selected_preset_name() == "Installed printer");
        CHECK(bundle.printers.get_edited_preset().is_dirty);
        CHECK(bundle.printers.get_edited_preset().config.opt_string("printer_notes") == "changed notes");
        // The model used to be left out of the copy; it is NORMAL, so it is copied like any other value.
        CHECK(bundle.printers.get_edited_preset().config.opt_string("printer_model") == "Model X");
        CHECK(bundle.printers.get_edited_preset().config.opt_string("printer_settings_id") == "Installed sentinel");
        CHECK(bundle.printers.get_edited_preset().config.opt_serialize("host_type") == "octoprint");
        CHECK(bundle.printers.get_edited_preset().inherits() == "Parent printer");
    }
    SECTION("a section with a different parent does not recognize the profile")
    {
        Slic3r::DynamicPrintConfig config = project_like_config("Installed printer");
        bundle.load_config_model("project.3mf", config, "[printer:Installed printer]\ninherits = Other parent\n\n");
        CHECK(bundle.printers.get_selected_preset_name() == "Installed printer");
        CHECK(bundle.printers.get_edited_preset().is_dirty);
        CHECK(bundle.printers.get_edited_preset().inherits() == "Other parent");
        // The section does not hold the default print profile: the installed value stays.
        CHECK(bundle.printers.get_edited_preset().config.opt_string("default_print_profile") == "Default print");
        CHECK(bundle.printers.get_edited_preset().config.opt_serialize("host_type") == "octoprint");
    }
    SECTION("a section with a different default print profile is copied into the profile")
    {
        Slic3r::DynamicPrintConfig config = project_like_config("Installed printer");
        bundle.load_config_model("project.3mf", config, "[printer:Installed printer]\ninherits = Parent printer\ndefault_print_profile = Other print\n\n");
        CHECK(bundle.printers.get_edited_preset().is_dirty);
        CHECK(bundle.printers.get_edited_preset().config.opt_string("default_print_profile") == "Other print");
        CHECK(bundle.printers.get_edited_preset().inherits() == "Parent printer");
    }
    SECTION("a section equal to the installed metadata keeps the profile unmodified")
    {
        Slic3r::DynamicPrintConfig config = project_like_config("Installed printer");
        bundle.load_config_model("project.3mf", config, "[printer:Installed printer]\ninherits = Parent printer\ndefault_print_profile = Default print\n\n");
        CHECK_FALSE(bundle.printers.get_edited_preset().is_dirty);
        CHECK(bundle.printers.get_edited_preset().config.opt_serialize("host_type") == "octoprint");
    }
}

// Two filaments: the first and the second load differently (OnlyIfModified, then Never once one was modified). Each one reads
// the section of its own profile and nothing of the other one.
TEST_CASE("Each filament of a project is recognized from its own section",
          "[plugins][config][compare][project]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Slic3r::PresetBundle bundle;
    Slic3r::DynamicPrintConfig config = project_like_config("Project printer");
    config.set_deserialize_strict({{"nozzle_diameter", "0.4,0.4"},
                                   {"filament_diameter", "1.75,1.75"},
                                   {"filament_settings_id", "Filament A;Filament B"}});
    Slic3r::Preset::normalize(config);

    SECTION("the second filament differs")
    {
        add_installed_filament(bundle, "Filament A", "Parent A", "nozzle_diameter[0]==0.6", false);
        add_installed_filament(bundle, "Filament B", "Parent B", "nozzle_diameter[0]==0.8", false);
        bundle.load_config_model("project.3mf", config,
                                 "[filament:Filament A]\ninherits = Parent A\n\n[filament:Filament B]\ninherits = Other B\n\n");
        REQUIRE(bundle.extruders_filaments.size() == 2);
        CHECK(bundle.extruders_filaments[0].get_selected_preset_name() == "Filament A");
        CHECK(bundle.extruders_filaments[1].get_selected_preset_name() == "Filament B");
        // The second filament is the edited one: it takes the parent of its section and keeps its own condition.
        const Slic3r::Preset &edited = bundle.filaments.get_edited_preset();
        CHECK(edited.name == "Filament B");
        CHECK(edited.is_dirty);
        CHECK(edited.inherits() == "Other B");
        CHECK(edited.compatible_printers_condition() == "nozzle_diameter[0]==0.8");
        // The stored presets keep their own metadata.
        const Slic3r::Preset *first = bundle.filaments.find_preset("Filament A", false, false);
        const Slic3r::Preset *second = bundle.filaments.find_preset("Filament B", false, false);
        REQUIRE(first != nullptr);
        REQUIRE(second != nullptr);
        CHECK_FALSE(first->is_dirty);
        CHECK(first->inherits() == "Parent A");
        CHECK(first->compatible_printers_condition() == "nozzle_diameter[0]==0.6");
        CHECK(second->inherits() == "Parent B");
    }
    SECTION("the first filament differs")
    {
        add_installed_filament(bundle, "Filament A", "Parent A", "nozzle_diameter[0]==0.6", true);
        add_installed_filament(bundle, "Filament B", "Parent B", "nozzle_diameter[0]==0.8", true);
        bundle.load_config_model("project.3mf", config,
                                 "[filament:Filament A]\ninherits = Other A\n\n[filament:Filament B]\ninherits = Parent B\n\n");
        REQUIRE(bundle.extruders_filaments.size() == 2);
        CHECK(bundle.extruders_filaments[0].get_selected_preset_name() == "Filament A");
        CHECK(bundle.extruders_filaments[1].get_selected_preset_name() == "Filament B");
        const Slic3r::Preset &edited = bundle.filaments.get_edited_preset();
        CHECK(edited.name == "Filament A");
        CHECK(edited.is_dirty);
        CHECK(edited.inherits() == "Other A");
        CHECK(edited.compatible_printers_condition() == "nozzle_diameter[0]==0.6");
        const Slic3r::Preset *second = bundle.filaments.find_preset("Filament B", false, false);
        REQUIRE(second != nullptr);
        CHECK(second->inherits() == "Parent B");
        CHECK(second->compatible_printers_condition() == "nozzle_diameter[0]==0.8");
        CHECK_FALSE(second->is_dirty);
    }
}

// A configuration file has no section: its profile is recognized from the values of the G-code destination.
TEST_CASE("A loaded configuration file recognizes the installed profile",
          "[plugins][config][compare][project]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedNatureDirectory directory;
    Slic3r::PresetBundle bundle;
    add_installed_printer(bundle, "Installed printer");

    Slic3r::DynamicPrintConfig config = project_like_config("Installed printer");
    config.set_deserialize_strict("host_type", "klipper");
    const boost::filesystem::path file = directory.path() / "profile.ini";
    config.save(file.string(), Slic3r::ConfigWriteTarget::GCode);
    REQUIRE_FALSE(Slic3r::Test::GCode::read_text_file(file).find("host_type") != std::string::npos);

    bundle.load_config_file(file.string(), Slic3r::ForwardCompatibilitySubstitutionRule::Enable);
    CHECK(bundle.printers.get_selected_preset_name() == "Installed printer");
    CHECK_FALSE(bundle.printers.get_edited_preset().is_dirty);
    CHECK(bundle.printers.get_edited_preset().config.opt_serialize("host_type") == "octoprint");
}

// The compared keys follow the nature, not the contents of the file: a LOCAL key that the file holds is ignored, and a NORMAL
// key that it lacks is compared with its default value.
TEST_CASE("A minimal configuration file is compared on the keys of the G-code destination",
          "[plugins][config][compare][project]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedNatureDirectory directory;
    Slic3r::PresetBundle bundle;
    Slic3r::Preset &installed = add_installed_printer(bundle, "Installed printer");
    // The file lacks printer_notes, so the loaded value is the default one, an empty text.
    installed.config.set_deserialize_strict("printer_notes", "installed notes");

    const boost::filesystem::path file = directory.path() / "minimal.ini";
    {
        std::ofstream stream(file.string(), std::ios::binary);
        stream << "printer_technology = FFF\nnozzle_diameter = 0.4\nfilament_diameter = 1.75\n"
                  "print_settings_id = PrintX\nfilament_settings_id = FilX\nprinter_settings_id = Installed printer\n"
                  "host_type = klipper\n";
    }
    bundle.load_config_file(file.string(), Slic3r::ForwardCompatibilitySubstitutionRule::Enable);

    const Slic3r::Preset &edited = bundle.printers.get_edited_preset();
    CHECK(bundle.printers.get_selected_preset_name() == "Installed printer");
    // The missing NORMAL key makes the profile modified, and the default value is copied.
    CHECK(edited.is_dirty);
    CHECK(edited.config.opt_string("printer_notes").empty());
    // The LOCAL key of the file is neither compared nor copied.
    CHECK(edited.config.opt_serialize("host_type") == "octoprint");
    const std::map<Slic3r::OptionKeyIdx, uint16_t> options =
        Slic3r::PresetCollection::dirty_options(&edited, &bundle.printers.get_selected_preset());
    CHECK(options.count(Slic3r::OptionKeyIdx::scalar("printer_notes")) == 1);
    CHECK(options.count(Slic3r::OptionKeyIdx::scalar("host_type")) == 0);
}
