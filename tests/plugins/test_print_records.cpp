///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Tests for variable record channels owned by Print.

The suite checks the core store, the C ABI and the C++ borrowed view together.
It deliberately mutates channels only from the test thread; the concurrent case
is limited to identifier allocation, which is the only thread-safe write
operation promised by the public contract.

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- Test::init_print()
        |-- Steps::StepPipeline::run_slice()
        |-- Steps::StepPipeline::run_gcode()
        |-- print.apply()
        |-- Test::GCode::read_text_file()
        `-- print.prepare_checkpoint_resume()
*/

#include <catch2/catch.hpp>

#include <algorithm>
#include <cstdint>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "plugin_test_helpers.hpp"
#include "gcode_test_helpers.hpp"
#include "perimeter_test_helpers.hpp"

#include "libslic3r/Api/host/ApiHostUtils.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/c/slic3r_config.h"
#include "libslic3r/Api/plugin/c/slic3r_data_tree.h"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Config/ConfigDef.hpp"
#include "libslic3r/Config/ConfigOption.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/CustomGCode.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintRecords.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepExtrusionOrdering.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"

#include "test_data.hpp"

namespace {

static const print_handle *print_handle_for(const Slic3r::Print &print);

// Convert the host object into the same opaque borrowed handle supplied to a plugin step.
static const print_handle *print_handle_for(const Slic3r::Print &print)
{
    return reinterpret_cast<const print_handle *>(&print);
}

} // namespace

TEST_CASE("Print record store owns free-form Config channels", "[plugins][print-records][core]")
{
    Slic3r::PrintRecordStore records;
    CHECK(records.channels().empty());
    CHECK(records.find("") == nullptr);

    Slic3r::DynamicConfig &generic = records.get_or_add("");
    generic.set_key_value("host.value", new Slic3r::ConfigOptionInt(17));

    Slic3r::DynamicConfig &table = records.get_or_add("example.events");
    table.set_key_value("count", new Slic3r::ConfigOptionInt(2));
    table.set_key_value("record_id", new Slic3r::ConfigOptionInts({1, 2}));
    table.set_key_value("height", new Slic3r::ConfigOptionFloats({0.2, 12.5}));
    table.set_key_value("label", new Slic3r::ConfigOptionStrings({"first", "second"}));

    REQUIRE(records.find("") != nullptr);
    CHECK(records.find("")->option<Slic3r::ConfigOptionInt>("host.value")->value == 17);
    REQUIRE(records.find("example.events") != nullptr);
    CHECK(records.find("example.events")->option<Slic3r::ConfigOptionInt>("count")->value == 2);
    CHECK(records.find("example.events")->option<Slic3r::ConfigOptionInts>("record_id")->get_values() ==
          std::vector<int32_t>{1, 2});
    CHECK(records.find("example.events")->option<Slic3r::ConfigOptionFloats>("height")->get_values() ==
          std::vector<double>{0.2, 12.5});

    CHECK(records.channels() == std::vector<std::string>{"", "example.events"});
    CHECK(records.remove("missing") == false);
    CHECK(records.remove("") == true);
    CHECK(records.find("") == nullptr);
}

TEST_CASE("Print record Config addresses survive channel-map growth", "[plugins][print-records][core]")
{
    Slic3r::PrintRecordStore records;
    Slic3r::DynamicConfig *stable = &records.get_or_add("example.stable");
    stable->set_key_value("value", new Slic3r::ConfigOptionInt(42));

    // unordered_map rehashes may move buckets, but references to node values remain valid.
    for (uint32_t idx = 0; idx < 2048; ++idx)
        records.get_or_add("example.channel." + std::to_string(idx));

    CHECK(records.find_mutable("example.stable") == stable);
    CHECK(stable->option<Slic3r::ConfigOptionInt>("value")->value == 42);
}

TEST_CASE("Print record snapshots restore channels without rewinding identifiers",
          "[plugins][print-records][checkpoint]")
{
    using namespace Slic3r;
    PrintRecordStore records;
    DynamicConfig *stable = &records.get_or_add("stable");
    stable->set_key_value("value", new ConfigOptionInt(17));
    records.get_or_add("deleted").set_key_value("value", new ConfigOptionString("saved"));
    records.get_or_add("");
    REQUIRE(records.allocate_id() == 1);
    const PrintRecordStore::Snapshot snapshot = records.capture();
    const config_handle *handle = ApiHost::to_config_handle(stable);
    stable->set_key_value("value", new ConfigOptionInt(99));
    REQUIRE(records.remove("deleted"));
    REQUIRE(records.remove(""));
    records.get_or_add("added");
    REQUIRE(records.allocate_id() == 2);

    for (int attempt = 0; attempt < 2; ++attempt) {
        records.restore(snapshot);
        REQUIRE(records.find("stable") == stable);
        CHECK(ApiHost::to_config_handle(records.find("stable")) == handle);
        CHECK(stable->option<ConfigOptionInt>("value")->value == 17);
        REQUIRE(records.find("deleted") != nullptr);
        CHECK(records.find("deleted")->option<ConfigOptionString>("value")->value == "saved");
        CHECK(records.channels() == std::vector<std::string>{"", "deleted", "stable"});
        CHECK(records.allocate_id() == PrintRecordId(3 + attempt));
        stable->set_key_value("value", new ConfigOptionInt(100));
        records.get_or_add("added");
    }
    CHECK(snapshot.at("stable").option<ConfigOptionInt>("value")->value == 17);
}

TEST_CASE("Print record identifiers are globally unique under concurrency",
          "[plugins][print-records][threading]")
{
    Slic3r::PrintRecordStore records;
    constexpr uint32_t thread_count = 8;
    constexpr uint32_t ids_per_thread = 512;
    std::vector<std::vector<Slic3r::PrintRecordId>> allocated(thread_count);
    std::vector<std::thread> workers;

    for (uint32_t thread_idx = 0; thread_idx < thread_count; ++thread_idx) {
        workers.emplace_back([&records, &allocated, thread_idx]() {
            allocated[thread_idx].reserve(ids_per_thread);
            for (uint32_t idx = 0; idx < ids_per_thread; ++idx)
                allocated[thread_idx].push_back(records.allocate_id());
        });
    }
    for (std::thread &worker : workers)
        worker.join();

    std::set<Slic3r::PrintRecordId> unique;
    for (const std::vector<Slic3r::PrintRecordId> &thread_ids : allocated)
        unique.insert(thread_ids.begin(), thread_ids.end());

    REQUIRE(unique.size() == thread_count * ids_per_thread);
    CHECK(*unique.begin() == 1);
    CHECK(*unique.rbegin() == thread_count * ids_per_thread);
    CHECK(unique.count(Slic3r::INVALID_PRINT_RECORD_ID) == 0);

    records.remove("unrelated");
    CHECK(records.allocate_id() == thread_count * ids_per_thread + 1);
    records.clear();
    CHECK(records.allocate_id() == 1);
}

TEST_CASE("Print record C ABI exposes borrowed mutable Config channels",
          "[plugins][print-records][api]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Slic3r::Print print;
    const print_handle *handle = print_handle_for(print);

    CHECK(print_records_channels(nullptr).size == 0);
    CHECK(print_records_get(nullptr, "example") == nullptr);
    CHECK(print_records_get(handle, nullptr) == nullptr);
    CHECK(print_records_get_or_add(nullptr, "example") == nullptr);
    CHECK(print_records_get_or_add(handle, nullptr) == nullptr);
    CHECK(print_records_remove(nullptr, "example") == 0);
    CHECK(print_records_remove(handle, nullptr) == 0);
    CHECK(print_records_allocate_id(nullptr) == PRINT_RECORD_ID_INVALID);

    config_handle *named = print_records_get_or_add(handle, "z.example");
    config_handle *generic = print_records_get_or_add(handle, "");
    REQUIRE(named != nullptr);
    REQUIRE(generic != nullptr);
    CHECK(print_records_get(handle, "z.example") == named);
    CHECK(print_records_get(handle, "missing") == nullptr);

    config_option_handle *value = config_get_or_add_mutable(named, "value", SLIC3R_CONFIG_OPTION_STRING);
    REQUIRE(value != nullptr);
    config_option_set_string(value, "stored", 0);
    CHECK(Slic3r::ApiHost::to_config(print_records_get(handle, "z.example"))
              ->option<Slic3r::ConfigOptionString>("value")->value == "stored");

    const_strings_t channels = print_records_channels(handle);
    REQUIRE(channels.size == 2);
    CHECK(std::string(channels.items[0]) == "");
    CHECK(std::string(channels.items[1]) == "z.example");

    const print_record_id first_id = print_records_allocate_id(handle);
    CHECK(first_id == 1);
    CHECK(print_records_remove(handle, "z.example") == 1);
    CHECK(print_records_remove(handle, "z.example") == 0);
    CHECK(print_records_allocate_id(handle) == 2);

    print.clear();
    CHECK(print_records_channels(handle).size == 0);
    CHECK(print_records_allocate_id(handle) == 1);
}

TEST_CASE("PrintRecords C++ view follows channel lifetime", "[plugins][print-records][cpp]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Slic3r::Print print;
    slic3r_api::Print print_view(print_handle_for(print));
    slic3r_api::PrintRecords records = print_view.records();

    CHECK(records.find("example.values").has_value() == false);
    slic3r_api::MutableConfig config = records.get_or_add("example.values");
    config.get_or_add("count", SLIC3R_CONFIG_OPTION_INT).set_int(2);
    config.get_or_add("values", SLIC3R_CONFIG_OPTION_INTS).resize(2);
    config.get_mutable("values").set_int(11, 0);
    config.get_mutable("values").set_int(22, 1);

    std::optional<slic3r_api::Config> found = records.find("example.values");
    REQUIRE(found.has_value());
    CHECK(found->int_or_default("count", 0) == 2);
    CHECK(found->vector_int_or_default("values", 0, 0) == 11);
    CHECK(found->vector_int_or_default("values", 1, 0) == 22);
    CHECK(records.allocate_id() == 1);
    CHECK(records.channels() == std::vector<std::string>{"example.values"});
    CHECK(records.remove("example.values"));
    CHECK_FALSE(records.find("example.values").has_value());
}

TEST_CASE("Print publishes custom G-code markers as a stable record table",
          "[plugins][print-records][custom-gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Slic3r::Print print;
    Slic3r::Model model;
    Slic3r::DynamicPrintConfig config = Slic3r::DynamicPrintConfig::full_print_config();
    Slic3r::Test::init_print({Slic3r::Test::TestMesh::cube_20x20x20}, print, model, config);

    CHECK(print.records().find(Slic3r::CustomGCode::PrintRecordChannel) == nullptr);
    Slic3r::Steps::StepExtrusionOrdering::prepare(print);
    const Slic3r::DynamicConfig *record = print.records().find(Slic3r::CustomGCode::PrintRecordChannel);
    REQUIRE(record != nullptr);
    REQUIRE(record->option<Slic3r::ConfigOptionInt>(Slic3r::CustomGCode::PrintRecordSizeKey) != nullptr);
    REQUIRE(record->option<Slic3r::ConfigOptionInt>(Slic3r::CustomGCode::PrintRecordModeKey) != nullptr);
    CHECK(record->option<Slic3r::ConfigOptionInt>(Slic3r::CustomGCode::PrintRecordSizeKey)->value == 0);
    CHECK(record->option<Slic3r::ConfigOptionInt>(Slic3r::CustomGCode::PrintRecordModeKey)->value ==
          static_cast<int32_t>(Slic3r::CustomGCode::Undef));
    CHECK(record->option<Slic3r::ConfigOptionFloats>(Slic3r::CustomGCode::PrintRecordPrintZKey)->empty());
    CHECK(record->option<Slic3r::ConfigOptionInts>(Slic3r::CustomGCode::PrintRecordTypeKey)->empty());
    CHECK(record->option<Slic3r::ConfigOptionInts>(Slic3r::CustomGCode::PrintRecordExtruderKey)->empty());
    CHECK(record->option<Slic3r::ConfigOptionStrings>(Slic3r::CustomGCode::PrintRecordColorKey)->empty());
    CHECK(record->option<Slic3r::ConfigOptionStrings>(Slic3r::CustomGCode::PrintRecordExtraKey)->empty());

    Slic3r::CustomGCode::Info source;
    source.mode = Slic3r::CustomGCode::MultiExtruder;
    source.gcodes = {
        {Slic3r::scale_to_layer_coord(0.20), Slic3r::CustomGCode::ColorChange, 1, "#ff0000", "color"},
        {Slic3r::scale_to_layer_coord(1.25), Slic3r::CustomGCode::PausePrint, 2, "#00ff00", "pause"},
        {Slic3r::scale_to_layer_coord(2.50), Slic3r::CustomGCode::ToolChange, 3, "#0000ff", "tool"},
        {Slic3r::scale_to_layer_coord(3.75), Slic3r::CustomGCode::Template, 4, "template", "template-extra"},
        {Slic3r::scale_to_layer_coord(5.00), Slic3r::CustomGCode::Custom, 5, "custom", "G4 P100"}
    };
    model.custom_gcode_per_print_z = source;

    const Slic3r::Print::ApplyStatus changed = print.apply(model, config);
    CHECK(changed != Slic3r::Print::APPLY_STATUS_UNCHANGED);
    CHECK(record->option<Slic3r::ConfigOptionInt>(Slic3r::CustomGCode::PrintRecordSizeKey)->value == 0);
    Slic3r::Steps::StepExtrusionOrdering::prepare(print);

    // Updating the table swaps its contents without invalidating an already borrowed Config handle.
    const Slic3r::DynamicConfig *updated = print.records().find(Slic3r::CustomGCode::PrintRecordChannel);
    REQUIRE(updated == record);
    REQUIRE(updated->option<Slic3r::ConfigOptionInt>(Slic3r::CustomGCode::PrintRecordSizeKey) != nullptr);
    CHECK(updated->option<Slic3r::ConfigOptionInt>(Slic3r::CustomGCode::PrintRecordSizeKey)->value == 5);
    CHECK(updated->option<Slic3r::ConfigOptionInt>(Slic3r::CustomGCode::PrintRecordModeKey)->value ==
          static_cast<int32_t>(Slic3r::CustomGCode::MultiExtruder));

    const std::vector<double> &print_z =
        updated->option<Slic3r::ConfigOptionFloats>(Slic3r::CustomGCode::PrintRecordPrintZKey)->get_values();
    const std::vector<int32_t> &types =
        updated->option<Slic3r::ConfigOptionInts>(Slic3r::CustomGCode::PrintRecordTypeKey)->get_values();
    const std::vector<int32_t> &extruders =
        updated->option<Slic3r::ConfigOptionInts>(Slic3r::CustomGCode::PrintRecordExtruderKey)->get_values();
    const std::vector<std::string> &colors =
        updated->option<Slic3r::ConfigOptionStrings>(Slic3r::CustomGCode::PrintRecordColorKey)->get_values();
    const std::vector<std::string> &extras =
        updated->option<Slic3r::ConfigOptionStrings>(Slic3r::CustomGCode::PrintRecordExtraKey)->get_values();

    REQUIRE(print_z.size() == 5);
    CHECK(print_z[0] == Approx(0.20));
    CHECK(print_z[1] == Approx(1.25));
    CHECK(print_z[2] == Approx(2.50));
    CHECK(print_z[3] == Approx(3.75));
    CHECK(print_z[4] == Approx(5.00));
    CHECK(types == std::vector<int32_t>{0, 1, 2, 3, 4});
    CHECK(extruders == std::vector<int32_t>{1, 2, 3, 4, 5});
    CHECK(colors == std::vector<std::string>{"#ff0000", "#00ff00", "#0000ff", "template", "custom"});
    CHECK(extras == std::vector<std::string>{"color", "pause", "tool", "template-extra", "G4 P100"});
    CHECK(print.model().custom_gcode_per_print_z == source);
    CHECK(model.custom_gcode_per_print_z == source);

    // An unchanged apply must leave even the option objects untouched, proving that no swap occurred.
    const Slic3r::ConfigOption *size_option =
        updated->option(Slic3r::CustomGCode::PrintRecordSizeKey);
    print.apply(model, config);
    CHECK(print.records().find(Slic3r::CustomGCode::PrintRecordChannel)->option(
              Slic3r::CustomGCode::PrintRecordSizeKey) == size_option);

    // The legacy Info equality ignores an empty table's mode, but the published schema must retain it.
    model.custom_gcode_per_print_z.gcodes.clear();
    model.custom_gcode_per_print_z.mode = Slic3r::CustomGCode::SingleExtruder;
    print.apply(model, config);
    Slic3r::Steps::StepExtrusionOrdering::prepare(print);
    const Slic3r::DynamicConfig *empty_updated =
        print.records().find(Slic3r::CustomGCode::PrintRecordChannel);
    REQUIRE(empty_updated == record);
    CHECK(empty_updated->option<Slic3r::ConfigOptionInt>(Slic3r::CustomGCode::PrintRecordSizeKey)->value == 0);
    CHECK(empty_updated->option<Slic3r::ConfigOptionInt>(Slic3r::CustomGCode::PrintRecordModeKey)->value ==
          static_cast<int32_t>(Slic3r::CustomGCode::SingleExtruder));
    CHECK(print.model().custom_gcode_per_print_z.mode == Slic3r::CustomGCode::SingleExtruder);

    // Publishing host data does not consume the ID sequence reserved for record producers.
    CHECK(print.records().allocate_id() == 1);
}

TEST_CASE("Custom G-code model updates replay Ordering without reslicing and preserve Plan export",
          "[plugins][print-records][checkpoint][custom-gcode]")
{
    using namespace Slic3r;
    using Steps::Checkpoint;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    DynamicPrintConfig config = Test::PerimeterPluginTests::perimeter_config({
        {"gcode_flavor", "marlin2"}
    });
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepPipeline::run_slice(orchestrator, print);
    REQUIRE(print.current_checkpoint() == Checkpoint::Print);
    REQUIRE(print.records().find(CustomGCode::PrintRecordChannel) == nullptr);
    const Layer *layer = &print.object(0).layer(0);
    const std::vector<coord_t> profile = print.object(0).layer_profile();
    const boost::filesystem::path path = Test::GCode::temporary_gcode_path();
    struct OutputCleanup {
        boost::filesystem::path path;
        ~OutputCleanup() { Test::GCode::remove_output_pair(path); }
    } cleanup{path};
    Steps::StepPipeline::run_gcode(orchestrator, print, path.string());
    const DynamicConfig *record = print.records().find(CustomGCode::PrintRecordChannel);
    REQUIRE(record != nullptr);

    model.custom_gcode_per_print_z.mode = CustomGCode::SingleExtruder;
    model.custom_gcode_per_print_z.gcodes = {
        {scale_to_layer_coord(0.2), CustomGCode::Custom, 1, "", "G4 P54321"}
    };
    CHECK(print.apply(model, config) == Print::APPLY_STATUS_INVALIDATED);
    CHECK(print.current_checkpoint() == Checkpoint::Print);
    CHECK_FALSE(print.should_execute_step(STEP_POST_SLICING));
    CHECK(record->option<ConfigOptionInt>(CustomGCode::PrintRecordSizeKey)->value == 0);
    Steps::StepPipeline::run_gcode(orchestrator, print, path.string());
    CHECK(print.records().find(CustomGCode::PrintRecordChannel) == record);
    CHECK(&print.object(0).layer(0) == layer);
    CHECK(print.object(0).layer_profile() == profile);
    const std::string output = Test::GCode::read_text_file(path);
    CHECK(output.find("G4 P54321") != std::string::npos);

    print.request_replay_from_worker(Checkpoint::Plan);
    REQUIRE(print.current_checkpoint() == Checkpoint::Plan);
    Steps::StepPipeline::run_gcode(orchestrator, print, path.string());
    CHECK(Test::GCode::read_text_file(path).find("G4 P54321") != std::string::npos);
    CHECK(print.records().find(CustomGCode::PrintRecordChannel) == record);
    CHECK(&print.object(0).layer(0) == layer);

    print.request_replay_from_worker(Checkpoint::Slicing);
    print.prepare_checkpoint_resume();
    CHECK(print.records().find(CustomGCode::PrintRecordChannel) == nullptr);
    CHECK(print.model().custom_gcode_per_print_z == model.custom_gcode_per_print_z);
    Steps::StepPipeline::run_gcode(orchestrator, print, path.string());
    CHECK(Test::GCode::read_text_file(path).find("G4 P54321") != std::string::npos);
}
