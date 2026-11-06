/*
 * Integration tests for session setting reads and writes.
 *
 * The tests exercise the serialized single-setting route, typed batch values,
 * vector cardinality, authorization, and atomic rejection of invalid batches.
 * The fixture's session registry drives the slicing state that no route can
 * produce yet, so busy refusal and result invalidation are observable.
 */

#include <catch_main.hpp>
#include <nlohmann/json.hpp>

#include "engine.hpp"
#include "rest_server_fixture.hpp"
#include "session.hpp"

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using namespace RestServerTest;

namespace {

void ensure_engine_initialized()
{
    Slic3r::Rest::Engine &engine = Slic3r::Rest::Engine::instance();
    if (engine.initialized())
        return;

    static TempDirectory engine_root;
    Slic3r::Rest::EnginePaths paths;
    paths.resource_directory = SLIC3R_REST_RESOURCE_DIRECTORY;
    paths.data_directory = engine_root.path() / "engine_data";
    engine.initialize(paths);
}

const nlohmann::json *find_setting(
    const nlohmann::json &catalog,
    const std::string &category,
    const std::string &setting_id)
{
    for (const nlohmann::json &definition : catalog.at(category))
        if (definition.at("id") == setting_id)
            return &definition;
    return nullptr;
}

const nlohmann::json *find_setting_any(
    const nlohmann::json &catalog,
    const std::string &setting_id)
{
    for (const char *category : {"preference", "print", "filament", "extruder", "printer"}) {
        const nlohmann::json *definition = find_setting(catalog, category, setting_id);
        if (definition != nullptr)
            return definition;
    }
    return nullptr;
}

// Check the status, stable error code, and a message fragment so a rejection
// caused by an unrelated validation step cannot satisfy the assertion.
void require_error(
    const HttpResponseMessage &response,
    http::status status,
    const std::string &code,
    const std::string &message_fragment)
{
    REQUIRE(response.result() == status);
    const nlohmann::json error = nlohmann::json::parse(response.body()).at("error");
    REQUIRE(error.at("code") == code);
    REQUIRE(error.at("message").get<std::string>().find(message_fragment) != std::string::npos);
}

const nlohmann::json *find_read_only_setting(const nlohmann::json &catalog)
{
    for (const char *category : {"preference", "print", "filament", "extruder", "printer"})
        for (const nlohmann::json &definition : catalog.at(category))
            if (definition.at("read_only").get<bool>())
                return &definition;
    return nullptr;
}

} // namespace

TEST_CASE("REST sessions read and update typed settings atomically", "[rest_settings]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);

    const HttpResponseMessage created = fixture.request(http::verb::post, "/create_session");
    REQUIRE(created.result() == http::status::created);
    const std::string session_id = nlohmann::json::parse(created.body()).at("session_id");
    const std::string route = "/session/" + session_id;

    const HttpResponseMessage listed = fixture.request(http::verb::get, "/list_settings");
    REQUIRE(listed.result() == http::status::ok);
    const nlohmann::json catalog = nlohmann::json::parse(listed.body());
    REQUIRE(find_setting(catalog, "print", "layer_height") != nullptr);
    REQUIRE(find_setting(catalog, "extruder", "nozzle_diameter") != nullptr);

    const HttpResponseMessage unauthorized = fixture.request_without_auth(
        http::verb::get,
        route + "/get_setting/layer_height");
    REQUIRE(unauthorized.result() == http::status::unauthorized);

    const HttpResponseMessage body_rejected = fixture.request(
        http::verb::get,
        route + "/get_setting/layer_height",
        "{}",
        "application/json");
    REQUIRE(body_rejected.result() == http::status::bad_request);

    for (const std::string &setting_id : {"not_a_setting", "display_width"}) {
        const HttpResponseMessage invalid_setting = fixture.request(
            http::verb::get,
            route + "/get_setting/" + setting_id);
        REQUIRE(invalid_setting.result() == http::status::unprocessable_entity);
        const nlohmann::json error = nlohmann::json::parse(invalid_setting.body()).at("error");
        REQUIRE(error.at("code") == "invalid_setting");
        REQUIRE(error.at("message").get<std::string>().find(setting_id) != std::string::npos);
    }

    const HttpResponseMessage serialized = fixture.request(
        http::verb::get,
        route + "/get_setting/layer_height");
    REQUIRE(serialized.result() == http::status::ok);
    const nlohmann::json serialized_body = nlohmann::json::parse(serialized.body());
    REQUIRE(serialized_body.at("setting_id") == "layer_height");
    REQUIRE(serialized_body.at("serialized_value") == "0.2");

    const HttpResponseMessage serialized_array = fixture.request(
        http::verb::get,
        route + "/get_setting/nozzle_diameter");
    REQUIRE(serialized_array.result() == http::status::ok);
    REQUIRE(nlohmann::json::parse(serialized_array.body()).at("serialized_value") == "0.4");

    const HttpResponseMessage set_body_rejected = fixture.request(
        http::verb::post,
        route + "/set_setting/layer_height/0.31",
        "{}",
        "application/json");
    REQUIRE(set_body_rejected.result() == http::status::bad_request);

    const HttpResponseMessage serialized_cardinality = fixture.request(
        http::verb::post,
        route + "/set_setting/nozzle_diameter/0.4%2C0.5");
    REQUIRE(serialized_cardinality.result() == http::status::unprocessable_entity);
    const nlohmann::json serialized_cardinality_error = nlohmann::json::parse(
        serialized_cardinality.body()).at("error");
    REQUIRE(serialized_cardinality_error.at("code") == "invalid_setting");
    REQUIRE(serialized_cardinality_error.at("message").get<std::string>().find("count") !=
        std::string::npos);
    const HttpResponseMessage cardinality_value = fixture.request(
        http::verb::post,
        route + "/get_settings",
        R"({"setting_ids":["nozzle_diameter"]})",
        "application/json");
    REQUIRE(cardinality_value.result() == http::status::ok);
    REQUIRE(nlohmann::json::parse(cardinality_value.body()).at("nozzle_diameter")
        .at("value").size() == 1);

    const HttpResponseMessage invalid_serialized = fixture.request(
        http::verb::post,
        route + "/set_setting/layer_height/abc");
    REQUIRE(invalid_serialized.result() == http::status::unprocessable_entity);
    const nlohmann::json invalid_serialized_error = nlohmann::json::parse(
        invalid_serialized.body()).at("error");
    REQUIRE(invalid_serialized_error.at("code") == "invalid_setting");
    const HttpResponseMessage after_invalid_serialized = fixture.request(
        http::verb::get,
        route + "/get_setting/layer_height");
    REQUIRE(after_invalid_serialized.result() == http::status::ok);
    REQUIRE(nlohmann::json::parse(after_invalid_serialized.body()).at("serialized_value") == "0.2");

    const HttpResponseMessage values = fixture.request(
        http::verb::post,
        route + "/get_settings",
        R"({"setting_ids":["layer_height","nozzle_diameter"]})",
        "application/json");
    REQUIRE(values.result() == http::status::ok);
    const nlohmann::json current_values = nlohmann::json::parse(values.body());
    REQUIRE(current_values.size() == 2);
    REQUIRE(current_values.at("layer_height").at("value").is_number());
    REQUIRE(current_values.at("nozzle_diameter").at("value").is_array());

    const HttpResponseMessage invalid_combination = fixture.request(
        http::verb::post,
        route + "/set_settings",
        R"({"use_firmware_retraction":{"value":true},"wipe":{"value":[true]}})",
        "application/json");
    REQUIRE(invalid_combination.result() == http::status::unprocessable_entity);
    const nlohmann::json invalid_combination_error = nlohmann::json::parse(
        invalid_combination.body()).at("error");
    REQUIRE(invalid_combination_error.at("code") == "invalid_setting");
    REQUIRE(invalid_combination_error.at("message").get<std::string>().find("not compatible") !=
        std::string::npos);

    const HttpResponseMessage unchanged_combination = fixture.request(
        http::verb::post,
        route + "/get_settings",
        R"({"setting_ids":["use_firmware_retraction","wipe"]})",
        "application/json");
    REQUIRE(unchanged_combination.result() == http::status::ok);
    const nlohmann::json unchanged_combination_values = nlohmann::json::parse(
        unchanged_combination.body());
    REQUIRE(unchanged_combination_values.at("use_firmware_retraction").at("value") == false);
    REQUIRE(unchanged_combination_values.at("wipe").at("value") ==
        nlohmann::json::array({false}));

    const HttpResponseMessage valid_combination = fixture.request(
        http::verb::post,
        route + "/set_settings",
        R"({"use_firmware_retraction":{"value":true},"wipe":{"value":[false]}})",
        "application/json");
    REQUIRE(valid_combination.result() == http::status::ok);
    const nlohmann::json valid_combination_values = nlohmann::json::parse(
        valid_combination.body()).at("updated");
    REQUIRE(valid_combination_values.at("use_firmware_retraction").at("value") == true);
    REQUIRE(valid_combination_values.at("wipe").at("value") == nlohmann::json::array({false}));

    const HttpResponseMessage firmware_retraction = fixture.request(
        http::verb::post,
        route + "/set_setting/use_firmware_retraction/1");
    REQUIRE(firmware_retraction.result() == http::status::ok);

    const HttpResponseMessage invalid_single_combination = fixture.request(
        http::verb::post,
        route + "/set_setting/wipe/1");
    REQUIRE(invalid_single_combination.result() == http::status::unprocessable_entity);
    const nlohmann::json invalid_single_combination_error = nlohmann::json::parse(
        invalid_single_combination.body()).at("error");
    REQUIRE(invalid_single_combination_error.at("code") == "invalid_setting");
    REQUIRE(invalid_single_combination_error.at("message").get<std::string>().find("not compatible") !=
        std::string::npos);
    const HttpResponseMessage unchanged_wipe = fixture.request(
        http::verb::get,
        route + "/get_setting/wipe");
    REQUIRE(unchanged_wipe.result() == http::status::ok);
    REQUIRE(nlohmann::json::parse(unchanged_wipe.body()).at("serialized_value") == "0");

    const HttpResponseMessage set_serialized = fixture.request(
        http::verb::post,
        route + "/set_setting/layer_height/0.310");
    REQUIRE(set_serialized.result() == http::status::ok);
    REQUIRE(nlohmann::json::parse(set_serialized.body()).at("serialized_value") == "0.31");

    nlohmann::json nozzle_update = {
        {"value", current_values.at("nozzle_diameter").at("value")}
    };
    if (current_values.at("nozzle_diameter").contains("enabled") &&
        current_values.at("nozzle_diameter").at("enabled").is_array())
        nozzle_update["enabled"] = current_values.at("nozzle_diameter").at("enabled");
    const nlohmann::json batch = {
        {"layer_height", { {"value", 0.32} }},
        {"nozzle_diameter", nozzle_update}
    };
    const HttpResponseMessage set_batch = fixture.request(
        http::verb::post,
        route + "/set_settings",
        batch.dump(),
        "application/json");
    REQUIRE(set_batch.result() == http::status::ok);
    const nlohmann::json updated = nlohmann::json::parse(set_batch.body()).at("updated");
    REQUIRE(updated.at("layer_height").at("value") == 0.32);
    REQUIRE(updated.at("nozzle_diameter").at("value") == nozzle_update.at("value"));

    // use_firmware_retraction was set by an earlier request and omitted from
    // the batch, so it must keep its value.
    const HttpResponseMessage modified_values = fixture.request(
        http::verb::post,
        route + "/get_settings",
        R"({"setting_ids":["layer_height","use_firmware_retraction"]})",
        "application/json");
    REQUIRE(modified_values.result() == http::status::ok);
    const nlohmann::json modified_body = nlohmann::json::parse(modified_values.body());
    REQUIRE(modified_body.at("layer_height").at("value") == 0.32);
    REQUIRE(modified_body.at("use_firmware_retraction").at("value") == true);

    const HttpResponseMessage empty_ids = fixture.request(
        http::verb::post,
        route + "/get_settings",
        R"({"setting_ids":[]})",
        "application/json");
    REQUIRE(empty_ids.result() == http::status::bad_request);

    const HttpResponseMessage non_string_id = fixture.request(
        http::verb::post,
        route + "/get_settings",
        R"({"setting_ids":["layer_height",7]})",
        "application/json");
    REQUIRE(non_string_id.result() == http::status::bad_request);
    const nlohmann::json non_string_id_body = nlohmann::json::parse(non_string_id.body());
    REQUIRE(non_string_id_body.at("error").at("code") == "invalid_request");

    const HttpResponseMessage invalid_ids = fixture.request(
        http::verb::post,
        route + "/get_settings",
        R"({"setting_ids":["layer_height","not_a_setting"]})",
        "application/json");
    REQUIRE(invalid_ids.result() == http::status::unprocessable_entity);
    const nlohmann::json invalid_ids_body = nlohmann::json::parse(invalid_ids.body());
    REQUIRE(invalid_ids_body.at("error").at("code") == "invalid_setting");
    REQUIRE_FALSE(invalid_ids_body.contains("layer_height"));

    const HttpResponseMessage invalid_batch = fixture.request(
        http::verb::post,
        route + "/set_settings",
        R"({"layer_height":{"value":0.33},"not_a_setting":{"value":1}})",
        "application/json");
    require_error(invalid_batch, http::status::unprocessable_entity, "invalid_setting", "not_a_setting");

    // Every malformed typed entry is rejected by the codec before the batch is
    // committed; the final read below proves layer_height kept its value.
    const std::vector<std::pair<std::string, std::string>> rejected_entries = {
        {R"({"layer_height":{}})", "must be an object containing value"},
        {R"({"layer_height":{"value":0.33,"enabled":true}})", "enabled is not applicable"},
        {R"({"layer_height":{"value":0.33,"percent":true}})", "percent is not applicable"},
        {R"({"layer_height":{"value":0.33,"unit":"mm"}})", "Unknown field in setting value: unit"},
        {R"({"filament_travel_pa":{"value":[5],"enabled":false}})", "enabled must match the setting value cardinality"}
    };
    for (const std::pair<std::string, std::string> &entry : rejected_entries) {
        const HttpResponseMessage rejected = fixture.request(
            http::verb::post,
            route + "/set_settings",
            entry.first,
            "application/json");
        require_error(rejected, http::status::unprocessable_entity, "invalid_setting", entry.second);
    }

    const HttpResponseMessage array_body = fixture.request(
        http::verb::post,
        route + "/set_settings",
        R"([{"layer_height":{"value":0.33}}])",
        "application/json");
    require_error(array_body, http::status::bad_request, "invalid_request", "must be a JSON object");

    const HttpResponseMessage invalid_range = fixture.request(
        http::verb::post,
        route + "/set_setting/layer_height/-1");
    REQUIRE(invalid_range.result() == http::status::unprocessable_entity);

    const HttpResponseMessage unchanged = fixture.request(
        http::verb::get,
        route + "/get_setting/layer_height");
    REQUIRE(unchanged.result() == http::status::ok);
    REQUIRE(nlohmann::json::parse(unchanged.body()).at("serialized_value") == "0.32");
    REQUIRE(fixture.running());
}

TEST_CASE("REST settings preserve indicators, structures, encoding, and ownership", "[rest_settings]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);

    const HttpResponseMessage created = fixture.request(http::verb::post, "/create_session");
    REQUIRE(created.result() == http::status::created);
    const std::string session_id = nlohmann::json::parse(created.body()).at("session_id");
    const std::string route = "/session/" + session_id;
    const nlohmann::json catalog = nlohmann::json::parse(
        fixture.request(http::verb::get, "/list_settings").body());

    const nlohmann::json *float_or_percent = find_setting_any(catalog, "overhangs_width_speed");
    const nlohmann::json *point = find_setting_any(catalog, "priming_position");
    const nlohmann::json *graph = find_setting_any(catalog, "overhangs_dynamic_flow");
    const nlohmann::json *vector_indicators = find_setting_any(catalog, "filament_travel_pa");
    REQUIRE(float_or_percent != nullptr);
    REQUIRE(point != nullptr);
    REQUIRE(graph != nullptr);
    REQUIRE(vector_indicators != nullptr);
    REQUIRE(float_or_percent->at("type") == "float_or_percent");
    REQUIRE(float_or_percent->at("can_be_disabled") == true);
    REQUIRE(point->at("type") == "point2");
    REQUIRE(graph->at("type") == "graph");
    REQUIRE(vector_indicators->at("type") == "float_or_percent");
    REQUIRE(vector_indicators->at("array") == "extruder");

    const HttpResponseMessage typed_indicators = fixture.request(
        http::verb::post,
        route + "/set_settings",
        R"({"overhangs_width_speed":{"value":30,"enabled":false,"percent":true}})",
        "application/json");
    REQUIRE(typed_indicators.result() == http::status::ok);
    const nlohmann::json typed_value = nlohmann::json::parse(typed_indicators.body())
        .at("updated").at("overhangs_width_speed");
    REQUIRE(typed_value.at("value") == 30);
    REQUIRE(typed_value.at("enabled") == false);
    REQUIRE(typed_value.at("percent") == true);

    const HttpResponseMessage typed_vector_indicators = fixture.request(
        http::verb::post,
        route + "/set_settings",
        R"({"filament_travel_pa":{"value":[12],"enabled":[false],"percent":[false]}})",
        "application/json");
    REQUIRE(typed_vector_indicators.result() == http::status::ok);
    const nlohmann::json typed_vector_value = nlohmann::json::parse(typed_vector_indicators.body())
        .at("updated").at("filament_travel_pa");
    REQUIRE(typed_vector_value.at("value") == nlohmann::json::array({12}));
    REQUIRE(typed_vector_value.at("enabled") == nlohmann::json::array({false}));
    REQUIRE(typed_vector_value.at("percent") == nlohmann::json::array({false}));

    const HttpResponseMessage vector_indicator_cardinality = fixture.request(
        http::verb::post,
        route + "/set_settings",
        R"({"filament_travel_pa":{"value":[13],"enabled":[false,false],"percent":[false]}})",
        "application/json");
    require_error(
        vector_indicator_cardinality,
        http::status::unprocessable_entity,
        "invalid_setting",
        "enabled must match the setting value cardinality");

    // The same effective value written through the JSON route and through the
    // INI route must read back identically in both representations.
    const std::string indicator_ids = R"({"setting_ids":["overhangs_width_speed"]})";
    const HttpResponseMessage typed_equivalent = fixture.request(
        http::verb::post,
        route + "/set_settings",
        R"({"overhangs_width_speed":{"value":35,"enabled":false,"percent":true}})",
        "application/json");
    REQUIRE(typed_equivalent.result() == http::status::ok);
    const nlohmann::json typed_read = nlohmann::json::parse(fixture.request(
        http::verb::post, route + "/get_settings", indicator_ids, "application/json").body());
    const nlohmann::json typed_serialized_read = nlohmann::json::parse(fixture.request(
        http::verb::get, route + "/get_setting/overhangs_width_speed").body());
    REQUIRE(typed_serialized_read.at("serialized_value") == "!35%");

    const HttpResponseMessage serialized_indicators = fixture.request(
        http::verb::post,
        route + "/set_setting/overhangs_width_speed/%2135%25");
    REQUIRE(serialized_indicators.result() == http::status::ok);
    REQUIRE(nlohmann::json::parse(serialized_indicators.body()).at("serialized_value") == "!35%");

    const HttpResponseMessage get_serialized_indicators = fixture.request(
        http::verb::get,
        route + "/get_setting/overhangs_width_speed");
    REQUIRE(get_serialized_indicators.result() == http::status::ok);
    REQUIRE(nlohmann::json::parse(get_serialized_indicators.body()).at("serialized_value") == "!35%");
    const nlohmann::json serialized_read = nlohmann::json::parse(fixture.request(
        http::verb::post, route + "/get_settings", indicator_ids, "application/json").body());
    REQUIRE(serialized_read == typed_read);

    const nlohmann::json point_value = nlohmann::json::array({12.0, 34.0});
    const nlohmann::json graph_value = {
        {"type", "square"},
        {"points", nlohmann::json::array({
            nlohmann::json::array({0.0, 1.0}),
            nlohmann::json::array({50.0, 25.0}),
            nlohmann::json::array({100.0, 90.0})
        })}
    };
    const nlohmann::json structures = {
        {"priming_position", {{"value", point_value}}},
        {"overhangs_dynamic_flow", {{"value", graph_value}, {"enabled", true}}}
    };
    const HttpResponseMessage structured = fixture.request(
        http::verb::post,
        route + "/set_settings",
        structures.dump(),
        "application/json");
    REQUIRE(structured.result() == http::status::ok);
    const nlohmann::json structured_values = nlohmann::json::parse(structured.body()).at("updated");
    REQUIRE(structured_values.at("priming_position").at("value") == point_value);
    REQUIRE(structured_values.at("overhangs_dynamic_flow").at("value") == graph_value);

    const HttpResponseMessage serialized_point = fixture.request(
        http::verb::get,
        route + "/get_setting/priming_position");
    REQUIRE(serialized_point.result() == http::status::ok);
    REQUIRE(nlohmann::json::parse(serialized_point.body()).at("serialized_value") == "12x34");

    const HttpResponseMessage serialized_graph = fixture.request(
        http::verb::get,
        route + "/get_setting/overhangs_dynamic_flow");
    REQUIRE(serialized_graph.result() == http::status::ok);
    REQUIRE(nlohmann::json::parse(serialized_graph.body()).at("serialized_value") ==
        "0:3:0:0x1:50x25:100x90");

    const HttpResponseMessage structured_round_trip = fixture.request(
        http::verb::post,
        route + "/get_settings",
        R"({"setting_ids":["overhangs_width_speed","priming_position","overhangs_dynamic_flow","filament_travel_pa"]})",
        "application/json");
    REQUIRE(structured_round_trip.result() == http::status::ok);
    const nlohmann::json round_trip_values = nlohmann::json::parse(structured_round_trip.body());
    REQUIRE(round_trip_values.size() == 4);
    REQUIRE(round_trip_values.at("overhangs_width_speed").at("enabled") == false);
    REQUIRE(round_trip_values.at("overhangs_width_speed").at("percent") == true);
    REQUIRE(round_trip_values.at("priming_position").at("value") == point_value);
    REQUIRE(round_trip_values.at("overhangs_dynamic_flow").at("value") == graph_value);
    const nlohmann::json vector_indicator_values = round_trip_values.at("filament_travel_pa");
    REQUIRE(vector_indicator_values.at("value").is_array());
    REQUIRE(vector_indicator_values.at("enabled").is_array());
    REQUIRE(vector_indicator_values.at("percent").is_array());
    REQUIRE(vector_indicator_values.at("value").size() == 1);
    REQUIRE(vector_indicator_values.at("enabled").size() == 1);
    REQUIRE(vector_indicator_values.at("percent").size() == 1);
    REQUIRE(vector_indicator_values.at("value").at(0) == 12);
    REQUIRE(vector_indicator_values.at("enabled").at(0) == false);
    REQUIRE(vector_indicator_values.at("percent").at(0) == false);

    const HttpResponseMessage encoded_string = fixture.request(
        http::verb::post,
        route + "/set_setting/output_filename_format/%2F%25");
    REQUIRE(encoded_string.result() == http::status::ok);
    REQUIRE(nlohmann::json::parse(encoded_string.body()).at("serialized_value") == "/%");

    const HttpResponseMessage get_encoded_string = fixture.request(
        http::verb::get,
        route + "/get_setting/output_filename_format");
    REQUIRE(get_encoded_string.result() == http::status::ok);
    REQUIRE(nlohmann::json::parse(get_encoded_string.body()).at("serialized_value") == "/%");

    const HttpResponseMessage encoded_text = fixture.request(
        http::verb::post,
        route + "/set_setting/output_filename_format/%5Bfoo%20caf%C3%A9%5D");
    REQUIRE(encoded_text.result() == http::status::ok);
    REQUIRE(nlohmann::json::parse(encoded_text.body()).at("serialized_value") == "[foo café]");

    const HttpResponseMessage get_encoded_text = fixture.request(
        http::verb::get,
        route + "/get_setting/output_filename_format");
    REQUIRE(get_encoded_text.result() == http::status::ok);
    REQUIRE(nlohmann::json::parse(get_encoded_text.body()).at("serialized_value") == "[foo café]");

    const HttpResponseMessage empty_string = fixture.request(
        http::verb::post,
        route + "/set_setting/output_filename_format/");
    REQUIRE(empty_string.result() == http::status::ok);
    REQUIRE(nlohmann::json::parse(empty_string.body()).at("serialized_value") == "");

    const HttpResponseMessage get_empty_string = fixture.request(
        http::verb::get,
        route + "/get_setting/output_filename_format");
    REQUIRE(get_empty_string.result() == http::status::ok);
    REQUIRE(nlohmann::json::parse(get_empty_string.body()).at("serialized_value") == "");

    const HttpResponseMessage disabled_marker_rejected = fixture.request(
        http::verb::post,
        route + "/set_setting/layer_height/%210.33");
    REQUIRE(disabled_marker_rejected.result() == http::status::unprocessable_entity);
    const nlohmann::json disabled_error = nlohmann::json::parse(disabled_marker_rejected.body());
    REQUIRE(disabled_error.at("error").at("message").get<std::string>().find("cannot be disabled") !=
        std::string::npos);

    const HttpResponseMessage type_error = fixture.request(
        http::verb::post,
        route + "/set_settings",
        R"({"layer_height":{"value":"not-a-number"}})",
        "application/json");
    REQUIRE(type_error.result() == http::status::unprocessable_entity);
    REQUIRE(nlohmann::json::parse(type_error.body()).at("error").at("message").get<std::string>().find("value") !=
        std::string::npos);

    const HttpResponseMessage empty_batch = fixture.request(
        http::verb::post,
        route + "/set_settings",
        "{}",
        "application/json");
    REQUIRE(empty_batch.result() == http::status::ok);
    REQUIRE(nlohmann::json::parse(empty_batch.body()).at("updated").empty());

    const HttpResponseMessage current_values_response = fixture.request(
        http::verb::post,
        route + "/get_settings",
        R"({"setting_ids":["nozzle_diameter"]})",
        "application/json");
    REQUIRE(current_values_response.result() == http::status::ok);
    const nlohmann::json current_nozzle = nlohmann::json::parse(current_values_response.body())
        .at("nozzle_diameter");
    nlohmann::json wrong_nozzle_values = current_nozzle.at("value");
    wrong_nozzle_values.push_back(wrong_nozzle_values.at(0));
    nlohmann::json wrong_nozzle = {{"value", wrong_nozzle_values}};
    if (current_nozzle.contains("enabled")) {
        nlohmann::json wrong_enabled = current_nozzle.at("enabled");
        if (wrong_enabled.is_array())
            wrong_enabled.push_back(wrong_enabled.at(0));
        wrong_nozzle["enabled"] = wrong_enabled;
    }
    const HttpResponseMessage cardinality = fixture.request(
        http::verb::post,
        route + "/set_settings",
        nlohmann::json{{"layer_height", {{"value", 0.34}}}, {"nozzle_diameter", wrong_nozzle}}.dump(),
        "application/json");
    REQUIRE(cardinality.result() == http::status::unprocessable_entity);

    const std::string bob_token = fixture.login_token("Bob", "secret");
    const std::vector<std::pair<std::string, std::string>> bob_auth = {
        {"Authorization", "Bearer " + bob_token}
    };
    const std::string missing_route = "/session/session_missing";
    for (const std::string &session_route : {route, missing_route}) {
        REQUIRE(fixture.request(
            http::verb::get,
            session_route + "/get_setting/layer_height",
            {},
            {},
            session_route == route ? bob_auth : std::vector<std::pair<std::string, std::string>>{}).result() ==
            http::status::not_found);
        REQUIRE(fixture.request(
            http::verb::post,
            session_route + "/get_settings",
            R"({"setting_ids":["layer_height"]})",
            "application/json",
            session_route == route ? bob_auth : std::vector<std::pair<std::string, std::string>>{}).result() ==
            http::status::not_found);
        REQUIRE(fixture.request(
            http::verb::post,
            session_route + "/set_setting/layer_height/0.31",
            {},
            {},
            session_route == route ? bob_auth : std::vector<std::pair<std::string, std::string>>{}).result() ==
            http::status::not_found);
        REQUIRE(fixture.request(
            http::verb::post,
            session_route + "/set_settings",
            R"({"layer_height":{"value":0.31}})",
            "application/json",
            session_route == route ? bob_auth : std::vector<std::pair<std::string, std::string>>{}).result() ==
            http::status::not_found);
    }

    const nlohmann::json *read_only = find_read_only_setting(catalog);
    if (read_only != nullptr) {
        const std::string read_only_id = read_only->at("id");
        const HttpResponseMessage read_only_serialized = fixture.request(
            http::verb::post,
            route + "/set_setting/" + read_only_id + "/0");
        REQUIRE(read_only_serialized.result() == http::status::unprocessable_entity);
        REQUIRE(nlohmann::json::parse(read_only_serialized.body()).at("error").at("message")
            .get<std::string>().find("read-only") != std::string::npos);
    }
    if (read_only != nullptr && !read_only->at("default").is_null()) {
        const std::string read_only_id = read_only->at("id");
        const HttpResponseMessage read_only_response = fixture.request(
            http::verb::post,
            route + "/set_settings",
            nlohmann::json{{read_only_id, read_only->at("default")}}.dump(),
            "application/json");
        REQUIRE(read_only_response.result() == http::status::unprocessable_entity);
    }

    REQUIRE(fixture.running());
}

// A slicing job is started directly on the registry because the slice route
// does not exist yet; the setting routes must refuse to mutate during that job
// and invalidate the finished result afterwards.
TEST_CASE("REST setting writes respect the slicing state", "[rest_settings]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);

    const HttpResponseMessage created = fixture.request(http::verb::post, "/create_session");
    REQUIRE(created.result() == http::status::created);
    const std::string session_id = nlohmann::json::parse(created.body()).at("session_id");
    const std::string route = "/session/" + session_id;
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = fixture.sessions().find(
        Slic3r::Rest::normalize_username("Alice"),
        session_id,
        Slic3r::Rest::SessionRegistry::Clock::now());
    REQUIRE(session != nullptr);

    const std::optional<Slic3r::Rest::SessionContext::WorkToken> work =
        session->begin_work(Slic3r::Rest::SessionRegistry::Clock::now());
    REQUIRE(work.has_value());
    const HttpResponseMessage busy_serialized = fixture.request(
        http::verb::post,
        route + "/set_setting/layer_height/0.25");
    REQUIRE(busy_serialized.result() == http::status::conflict);
    REQUIRE(nlohmann::json::parse(busy_serialized.body()).at("error").at("code") == "session_busy");
    const HttpResponseMessage busy_typed = fixture.request(
        http::verb::post,
        route + "/set_settings",
        R"({"layer_height":{"value":0.25}})",
        "application/json");
    REQUIRE(busy_typed.result() == http::status::conflict);
    REQUIRE(nlohmann::json::parse(busy_typed.body()).at("error").at("code") == "session_busy");
    const HttpResponseMessage unchanged = fixture.request(
        http::verb::get,
        route + "/get_setting/layer_height");
    REQUIRE(unchanged.result() == http::status::ok);
    REQUIRE(nlohmann::json::parse(unchanged.body()).at("serialized_value") == "0.2");

    REQUIRE(session->finish_work(
        work.value(),
        "result.gcode",
        Slic3r::GCodeProcessorResult(),
        Slic3r::Rest::SessionRegistry::Clock::now()));
    REQUIRE(session->snapshot().status == Slic3r::Rest::SessionStatus::sliced);
    REQUIRE_FALSE(session->viewer_snapshot()->outdated);

    const HttpResponseMessage invalidating = fixture.request(
        http::verb::post,
        route + "/set_setting/layer_height/0.25");
    REQUIRE(invalidating.result() == http::status::ok);
    const Slic3r::Rest::SessionContext::Snapshot after = session->snapshot();
    REQUIRE(after.status == Slic3r::Rest::SessionStatus::ready);
    REQUIRE_FALSE(after.current_gcode);
    REQUIRE(session->viewer_snapshot()->outdated);
}

// A configuration can already be invalid when it arrives from a profile or a
// project. The setting routes then refuse only mutations that introduce a new
// engine validation error, so unrelated edits remain possible.
TEST_CASE("REST setting writes tolerate a pre-existing invalid combination", "[rest_settings]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);

    const HttpResponseMessage created = fixture.request(http::verb::post, "/create_session");
    REQUIRE(created.result() == http::status::created);
    const std::string session_id = nlohmann::json::parse(created.body()).at("session_id");
    const std::string route = "/session/" + session_id;
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = fixture.sessions().find(
        Slic3r::Rest::normalize_username("Alice"),
        session_id,
        Slic3r::Rest::SessionRegistry::Clock::now());
    REQUIRE(session != nullptr);

    // Firmware retraction combined with wipe is rejected by the engine validator.
    const std::optional<Slic3r::Rest::SessionContext::MutationToken> mutation = session->begin_mutation();
    REQUIRE(mutation.has_value());
    Slic3r::Rest::SessionData invalid = session->data_snapshot().value();
    invalid.configuration.set_deserialize_strict("use_firmware_retraction", "1");
    invalid.configuration.set_deserialize_strict("wipe", "1");
    REQUIRE_FALSE(Slic3r::DynamicPrintConfig(invalid.configuration).validate().empty());
    REQUIRE(session->commit_mutation(
        mutation.value(),
        std::move(invalid),
        true,
        Slic3r::Rest::SessionRegistry::Clock::now()));

    const HttpResponseMessage unrelated_serialized = fixture.request(
        http::verb::post,
        route + "/set_setting/layer_height/0.25");
    REQUIRE(unrelated_serialized.result() == http::status::ok);
    const HttpResponseMessage unrelated_typed = fixture.request(
        http::verb::post,
        route + "/set_settings",
        R"({"layer_height":{"value":0.26}})",
        "application/json");
    REQUIRE(unrelated_typed.result() == http::status::ok);

    // Removing the conflict is accepted, and recreating it is refused again.
    const HttpResponseMessage fixed = fixture.request(
        http::verb::post,
        route + "/set_setting/wipe/0");
    REQUIRE(fixed.result() == http::status::ok);
    const HttpResponseMessage reintroduced = fixture.request(
        http::verb::post,
        route + "/set_setting/wipe/1");
    require_error(reintroduced, http::status::unprocessable_entity, "invalid_setting", "not compatible");
}
