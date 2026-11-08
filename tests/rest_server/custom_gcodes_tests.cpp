// Direct tests cover the engine mode rule, request validation and sorting, and the JSON warnings.
#include <catch2/catch.hpp>

#include "custom_gcodes.hpp"
#include "paint.hpp"
#include "rest_models_test_helpers.hpp"

#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

using namespace RestServerTest;
using namespace Slic3r;
using namespace Slic3r::Rest;

namespace {

// Build a scene with one 20 mm cube lying on the bed.
Model make_cube_scene()
{
    Model scene;
    ModelObject *object = scene.add_object("cube", "cube.stl", make_cube(20., 20., 20.));
    object->add_instance();
    object->ensure_on_bed();
    return scene;
}

// Build a complete session configuration with the given extruder count.
DynamicPrintConfig make_session(std::size_t extruders)
{
    DynamicPrintConfig session = DynamicPrintConfig::full_print_config();
    session.set_num_extruders(static_cast<unsigned int>(extruders));
    return session;
}

// Build one request element.
CustomGCodeEntry make_entry(double z, CustomGCode::Type type, std::optional<int> extruder = {},
    std::optional<std::string> color = {}, std::string text = {})
{
    CustomGCodeEntry entry;
    entry.z = z;
    entry.type = type;
    entry.extruder = extruder;
    entry.color = std::move(color);
    entry.text = std::move(text);
    return entry;
}

// Return the index and reason of the item error thrown for the entries.
std::pair<std::size_t, std::string> item_error(const std::vector<CustomGCodeEntry> &entries, const Model &scene,
    const DynamicPrintConfig &session)
{
    try {
        make_custom_gcodes(entries, scene, session);
    } catch (const CustomGCodeItemError &error) {
        CHECK(error.status() == 422);
        CHECK(error.code() == "invalid_custom_gcode");
        return {error.item(), error.reason()};
    }
    FAIL("make_custom_gcodes did not throw an item error.");
    return {};
}

// Return true when the warnings array contains the given item (or no item) and reason.
bool has_warning(const nlohmann::json &json, const std::string &reason, std::optional<std::size_t> item = {})
{
    for (const nlohmann::json &warning : json.at("warnings"))
        if (warning.at("reason") == reason && warning.contains("item") == item.has_value() &&
            (!item || warning.at("item") == item.value()))
            return true;
    return false;
}

} // namespace

// Verify the mode follows the engine rule for extruder counts, parts, modifiers, ranges and MMU painting.
TEST_CASE("Custom G-code mode follows the engine rule", "[custom_gcode]")
{
    ensure_engine_initialized();
    Model scene = make_cube_scene();
    ModelObject *object = scene.object_ptrs().front();
    ModelVolume *volume = object->volumes.front();
    const DynamicPrintConfig triple = make_session(3);

    SECTION("single extruder and empty scene") {
        const CustomGCodeMode single = current_custom_gcode_mode(scene, make_session(1));
        CHECK(single.mode == CustomGCode::SingleExtruder);
        CHECK(single.extruder == 1);
        CHECK(current_custom_gcode_mode(Model(), triple).mode == CustomGCode::SingleExtruder);
    }

    SECTION("everything on the default extruder") {
        const CustomGCodeMode mode = current_custom_gcode_mode(scene, triple);
        CHECK(mode.mode == CustomGCode::MultiAsSingle);
        CHECK(mode.extruder == 1);
    }

    // The object's fallback print region (region zero) always keeps the default extruder, so a
    // part on another extruder makes two extruders print.
    SECTION("a part on another extruder under an object without extruder") {
        volume->config.set_key_value("extruder", new ConfigOptionInt(2));
        const CustomGCodeMode mode = current_custom_gcode_mode(scene, triple);
        CHECK(mode.mode == CustomGCode::MultiExtruder);
        CHECK(mode.extruder == 1);
    }

    // Same fallback region zero: the object-level extruder does not reach it either.
    SECTION("an object and its part on the same other extruder") {
        object->config.set_key_value("extruder", new ConfigOptionInt(2));
        volume->config.set_key_value("extruder", new ConfigOptionInt(2));
        CHECK(current_custom_gcode_mode(scene, triple).mode == CustomGCode::MultiExtruder);
    }

    SECTION("a layer range reaching the rounding limit") {
        object->layer_config_ranges[{0., 10000.}].set_key_value("layer_height", new ConfigOptionFloat(0.2));
        object->layer_config_ranges[{20000., 30000.}].set_key_value("layer_height", new ConfigOptionFloat(0.2));
        CHECK(current_custom_gcode_mode(scene, triple).mode == CustomGCode::MultiAsSingle);
    }

    SECTION("a layer range just below the rounding limit") {
        object->instances.front()->set_offset(Vec3d(0., 0., 9999.8 - 20.));
        ModelConfig &range = object->layer_config_ranges[{9999.2, 9999.8}];
        range.set_key_value("layer_height", new ConfigOptionFloat(0.2));
        range.set_key_value("extruder", new ConfigOptionInt(2));
        CHECK(current_custom_gcode_mode(scene, triple).mode == CustomGCode::MultiExtruder);
    }

    SECTION("a modifier on another extruder") {
        ModelVolume *modifier = object->add_volume(make_cube(10., 10., 10.), ModelVolumeType::PARAMETER_MODIFIER);
        modifier->config.set_key_value("extruder", new ConfigOptionInt(3));
        CHECK(current_custom_gcode_mode(scene, triple).mode == CustomGCode::MultiExtruder);
    }

    SECTION("a layer range on another extruder") {
        ModelConfig &range = object->layer_config_ranges[{0., 5.}];
        range.set_key_value("layer_height", new ConfigOptionFloat(0.2));
        range.set_key_value("extruder", new ConfigOptionInt(3));
        CHECK(current_custom_gcode_mode(scene, triple).mode == CustomGCode::MultiExtruder);
    }

    SECTION("MMU painting on another extruder") {
        PaintRequest request;
        request.kind = "mmu";
        request.state = static_cast<EnforcerBlockerType>(2);
        request.strokes = {{PaintPoint{false, Vec3d(10., 10., 20.), Vec3d::Zero()}}};
        REQUIRE_FALSE(paint(*object, volume, 0, request).changed.empty());
        CHECK(current_custom_gcode_mode(scene, triple).mode == CustomGCode::MultiExtruder);
    }
}

// Verify sorting, layer rounding, duplicates, bounds, per-mode types, colors, texts and sequential printing.
TEST_CASE("Custom G-code entries are validated and sorted", "[custom_gcode]")
{
    ensure_engine_initialized();
    Model scene = make_cube_scene();
    const DynamicPrintConfig single = make_session(1);
    DynamicPrintConfig triple = make_session(3);

    SECTION("sorting and rounding") {
        const CustomGCode::Info info = make_custom_gcodes({
            make_entry(10., CustomGCode::PausePrint, {}, {}, "wait"),
            make_entry(5., CustomGCode::Template),
            make_entry(20., CustomGCode::Custom, {}, {}, "M117 top")}, scene, single);
        REQUIRE(info.gcodes.size() == 3);
        CHECK(info.mode == CustomGCode::SingleExtruder);
        CHECK(info.gcodes[0].print_z_ == scale_to_layer_coord(5.));
        CHECK(info.gcodes[0].type == CustomGCode::Template);
        CHECK(info.gcodes[1].type == CustomGCode::PausePrint);
        CHECK(info.gcodes[1].extra == "wait");
        CHECK(info.gcodes[1].extruder == 1);
        CHECK(info.gcodes[2].extra == "M117 top");
        CHECK(unscaled(make_custom_gcodes({make_entry(5.00006, CustomGCode::Template)}, scene, single)
            .gcodes[0].print_z_) == Approx(5.0001).margin(1e-9));
        CHECK(make_custom_gcodes({}, scene, triple).mode == CustomGCode::MultiAsSingle);
    }

    SECTION("duplicates after rounding") {
        const std::vector<CustomGCodeEntry> entries = {
            make_entry(5., CustomGCode::Template), make_entry(6., CustomGCode::Template),
            make_entry(5.00004, CustomGCode::Template)};
        REQUIRE(scale_to_layer_coord(5.) == scale_to_layer_coord(5.00004));
        CHECK(item_error(entries, scene, single) == std::make_pair(std::size_t(2), std::string("duplicate_z")));
    }

    SECTION("z bounds") {
        CHECK(item_error({make_entry(0., CustomGCode::Template)}, scene, single).second == "invalid_z");
        CHECK(item_error({make_entry(1., CustomGCode::Template), make_entry(-1., CustomGCode::Template)}, scene,
            single) == std::make_pair(std::size_t(1), std::string("invalid_z")));
        CHECK(item_error({make_entry(std::numeric_limits<double>::quiet_NaN(), CustomGCode::Template)}, scene,
            single).second == "invalid_z");
        CHECK(item_error({make_entry(20.5, CustomGCode::Template)}, scene, single).second == "above_objects");
        CHECK_NOTHROW(make_custom_gcodes({make_entry(20., CustomGCode::Template)}, scene, single));
        CHECK(item_error({make_entry(1., CustomGCode::Template)}, Model(), single).second == "empty_scene");
        CHECK(make_custom_gcodes({}, Model(), single).gcodes.empty());
        CHECK(item_error({make_entry(0.00004, CustomGCode::Template)}, scene, single).second == "invalid_z");
        CHECK(item_error({make_entry(10000., CustomGCode::Template)}, scene, single).second == "above_objects");
    }

    SECTION("a scene below the bed or at the rounding limit") {
        Model low = make_cube_scene();
        low.object_ptrs().front()->instances.front()->set_offset(Vec3d(0., 0., -30.));
        REQUIRE(low.object_ptrs().front()->instance_bounding_box(0).max.z() < 0.);
        CHECK(item_error({make_entry(1., CustomGCode::Template)}, low, single).second == "above_objects");
        CustomGCode::Info stored;
        stored.mode = CustomGCode::SingleExtruder;
        stored.gcodes = {{scale_to_layer_coord(5.), CustomGCode::Template, 1, "", ""}};
        CHECK(has_warning(custom_gcodes_json(stored, low, single), "above_objects", 0));

        // A top that rounds up to 10000 mm cannot be addressed again, so it is refused.
        Model tall = make_cube_scene();
        tall.object_ptrs().front()->instances.front()->set_offset(Vec3d(0., 0., 9999.99996 - 20.));
        REQUIRE(tall.object_ptrs().front()->instance_bounding_box(0).max.z() < 10000.);
        CHECK(item_error({make_entry(9999.99996, CustomGCode::Template)}, tall, single).second == "above_objects");
        CHECK_NOTHROW(make_custom_gcodes({make_entry(9990., CustomGCode::Template)}, tall, single));
    }

    SECTION("a top off the layer grid") {
        Model high;
        ModelObject *object = high.add_object("cube", "cube.stl", make_cube(20., 20., 20.00006));
        object->add_instance();
        object->ensure_on_bed();
        const double top = object->instance_bounding_box(0).max.z();
        REQUIRE(top == Approx(20.00006).margin(1e-5));
        REQUIRE(scale_to_layer_coord(top) != scale_i(top)); // the top is off the layer grid
        const CustomGCode::Info info = make_custom_gcodes({make_entry(top, CustomGCode::Template),
            make_entry(5., CustomGCode::PausePrint, {}, {}, "wait")}, high, single);
        REQUIRE(info.gcodes.size() == 2);
        const nlohmann::json json = custom_gcodes_json(info, high, single);
        CHECK_FALSE(has_warning(json, "above_objects", 1));
        std::vector<CustomGCodeEntry> again;
        for (const nlohmann::json &item : json.at("items")) {
            if (item.at("type") == "template")
                again.push_back(make_entry(item.at("z"), CustomGCode::Template));
            else
                again.push_back(make_entry(item.at("z"), CustomGCode::PausePrint, {}, {}, item.at("message")));
        }
        CHECK(make_custom_gcodes(again, high, single).gcodes == info.gcodes);
        CHECK_FALSE(has_warning(json, "above_objects", 0));
        CHECK(item_error({make_entry(top + 0.0002, CustomGCode::Template)}, high, single).second == "above_objects");
    }

    SECTION("types and extruders") {
        CHECK(item_error({make_entry(5., CustomGCode::ToolChange, 1)}, scene, single).second == "mode_tool_change");
        CHECK(item_error({make_entry(5., CustomGCode::ColorChange, 2)}, scene, single).second == "extruder_range");
        CHECK(make_custom_gcodes({make_entry(5., CustomGCode::ColorChange, 1)}, scene, single).gcodes[0].extruder == 1);
        CHECK(item_error({make_entry(5., CustomGCode::ToolChange)}, scene, triple).second == "extruder_required");
        CHECK(item_error({make_entry(5., CustomGCode::ToolChange, 4)}, scene, triple).second == "extruder_range");
        CHECK(item_error({make_entry(5., CustomGCode::ColorChange, 0)}, scene, triple).second == "extruder_range");
        const CustomGCode::Info info = make_custom_gcodes({make_entry(5., CustomGCode::ToolChange, 2)}, scene, triple);
        CHECK(info.mode == CustomGCode::MultiAsSingle);
        CHECK(info.gcodes[0].type == CustomGCode::ToolChange);
        CHECK(info.gcodes[0].extruder == 2);

        scene.object_ptrs().front()->volumes.front()->config.set_key_value("extruder", new ConfigOptionInt(3));
        scene.object_ptrs().front()->config.set_key_value("extruder", new ConfigOptionInt(1));
        REQUIRE(current_custom_gcode_mode(scene, triple).mode == CustomGCode::MultiExtruder);
        CHECK(item_error({make_entry(5., CustomGCode::ToolChange, 2)}, scene, triple).second == "mode_tool_change");
        const CustomGCode::Info multi = make_custom_gcodes({make_entry(5., CustomGCode::ColorChange, 3)}, scene, triple);
        CHECK(multi.mode == CustomGCode::MultiExtruder);
        CHECK(multi.gcodes[0].extruder == 3);
    }

    SECTION("colors") {
        CHECK(item_error({make_entry(5., CustomGCode::ColorChange, {}, std::string("red"))}, scene, single).second ==
            "invalid_color");
        CHECK(item_error({make_entry(5., CustomGCode::ColorChange, {}, std::string("#12345"))}, scene, single).second ==
            "invalid_color");
        CHECK(item_error({make_entry(5., CustomGCode::ColorChange, {}, std::string("#GG0000"))}, scene, single).second ==
            "invalid_color");
        CHECK(make_custom_gcodes({make_entry(5., CustomGCode::ColorChange, {}, std::string("#ff00aa"))}, scene, single)
            .gcodes[0].color == "#FF00AA");

        triple.set_deserialize_strict("extruder_colour", "\"\";\"\";\"\"");
        triple.set_deserialize_strict("filament_colour", "\"\";\"\";\"\"");
        CHECK(make_custom_gcodes({make_entry(5., CustomGCode::ColorChange)}, scene, triple).gcodes[0].color.empty());
        triple.set_deserialize_strict("extruder_colour", "\"#112233\";\"\";\"#44556a\"");
        triple.set_deserialize_strict("filament_colour", "\"#AAAAAA\";\"#bbbbbb\";\"#CCCCCC\"");
        const auto default_color = [&](int extruder) {
            return make_custom_gcodes({make_entry(5., CustomGCode::ColorChange, extruder)}, scene, triple)
                .gcodes[0].color;
        };
        CHECK(default_color(1) == "#112233");
        CHECK(default_color(2) == "#BBBBBB");
        CHECK(default_color(3) == "#44556A");
    }

    SECTION("texts") {
        CHECK(item_error({make_entry(5., CustomGCode::PausePrint)}, scene, single).second == "empty_message");
        CHECK(item_error({make_entry(5., CustomGCode::Custom)}, scene, single).second == "empty_gcode");
        CHECK(make_custom_gcodes({make_entry(5., CustomGCode::PausePrint, {}, {}, "Insert nuts")}, scene, single)
            .gcodes[0].extra == "Insert nuts");
    }

    SECTION("sequential printing") {
        DynamicPrintConfig sequential = make_session(1);
        sequential.set_deserialize_strict("complete_objects", "1");
        CHECK(make_custom_gcodes({}, scene, sequential).gcodes.empty());
        try {
            make_custom_gcodes({make_entry(5., CustomGCode::Template)}, scene, sequential);
            FAIL("Sequential printing accepted a non-empty list.");
        } catch (const CustomGCodeItemError &) {
            FAIL("The sequential refusal must not carry an item.");
        } catch (const SceneEditError &error) {
            CHECK(error.status() == 422);
            CHECK(error.code() == "invalid_custom_gcode");
        }
    }
}

// Verify the JSON item shapes, both modes and every warning reason.
TEST_CASE("Custom G-code JSON reports modes and warnings", "[custom_gcode]")
{
    ensure_engine_initialized();
    const Model scene = make_cube_scene();
    DynamicPrintConfig single = make_session(1);
    // mach3 has no default color change or pause G-code, so empty options really mean "nothing".
    single.set_deserialize_strict("gcode_flavor", "mach3");
    single.set_deserialize_strict("color_change_gcode", "");
    single.set_deserialize_strict("pause_print_gcode", "M601");
    single.set_deserialize_strict("template_custom_gcode", "");

    CustomGCode::Info info;
    info.mode = CustomGCode::SingleExtruder;
    info.gcodes = {
        {scale_to_layer_coord(5.), CustomGCode::ColorChange, 1, "#FF0000", ""},
        {scale_to_layer_coord(6.), CustomGCode::PausePrint, 1, "", "Insert nuts"},
        {scale_to_layer_coord(7.), CustomGCode::Template, 1, "", ""},
        {scale_to_layer_coord(8.), CustomGCode::Custom, 1, "", "M117 Hello"}};

    SECTION("shapes") {
        const nlohmann::json json = custom_gcodes_json(info, scene, single);
        CHECK(json.at("mode") == "single_extruder");
        CHECK(json.at("current_mode") == "single_extruder");
        REQUIRE(json.at("items").size() == 4);
        CHECK(json.at("items").at(0) == nlohmann::json({{"z", 5.}, {"type", "color_change"}, {"extruder", 1},
            {"color", "#FF0000"}}));
        CHECK(json.at("items").at(1) == nlohmann::json({{"z", 6.}, {"type", "pause"}, {"message", "Insert nuts"}}));
        CHECK(json.at("items").at(2) == nlohmann::json({{"z", 7.}, {"type", "template"}}));
        CHECK(json.at("items").at(3) == nlohmann::json({{"z", 8.}, {"type", "custom"}, {"gcode", "M117 Hello"}}));
        CHECK(custom_gcodes_json(CustomGCode::Info(), scene, single).at("mode") == "undefined");
        info.gcodes[0].color.clear();
        CHECK(custom_gcodes_json(info, scene, single).at("items").at(0) ==
            nlohmann::json({{"z", 5.}, {"type", "color_change"}, {"extruder", 1}}));
    }

    SECTION("empty printer G-code") {
        const nlohmann::json json = custom_gcodes_json(info, scene, single);
        CHECK(has_warning(json, "empty_printer_gcode", 0));
        CHECK_FALSE(has_warning(json, "empty_printer_gcode", 1));
        CHECK(has_warning(json, "empty_printer_gcode", 2));
        CHECK_FALSE(has_warning(json, "empty_printer_gcode", 3));
        CHECK(json.at("warnings").size() == 2);
        single.set_deserialize_strict("color_change_gcode", "M600");
        CHECK_FALSE(has_warning(custom_gcodes_json(info, scene, single), "empty_printer_gcode", 0));
    }

    SECTION("empty printer G-code follows the flavor defaults") {
        single.set_deserialize_strict("pause_print_gcode", "");
        single.set_deserialize_strict("gcode_flavor", "marlin");
        nlohmann::json json = custom_gcodes_json(info, scene, single);
        CHECK_FALSE(has_warning(json, "empty_printer_gcode", 0));
        CHECK_FALSE(has_warning(json, "empty_printer_gcode", 1));
        CHECK(has_warning(json, "empty_printer_gcode", 2));
        single.set_deserialize_strict("gcode_flavor", "klipper");
        json = custom_gcodes_json(info, scene, single);
        CHECK(has_warning(json, "empty_printer_gcode", 0));
        CHECK_FALSE(has_warning(json, "empty_printer_gcode", 1));
        single.set_deserialize_strict("gcode_flavor", "mach3");
        CHECK(has_warning(custom_gcodes_json(info, scene, single), "empty_printer_gcode", 1));
    }

    SECTION("color changes are checked in every mode") {
        DynamicPrintConfig dual = make_session(2);
        dual.set_deserialize_strict("gcode_flavor", "mach3");
        dual.set_deserialize_strict("color_change_gcode", "");
        info.mode = CustomGCode::MultiAsSingle;
        CHECK(has_warning(custom_gcodes_json(info, scene, dual), "empty_printer_gcode", 0));
    }

    SECTION("above objects") {
        info.gcodes.push_back({scale_to_layer_coord(25.), CustomGCode::Custom, 1, "", "M117"});
        CHECK(has_warning(custom_gcodes_json(info, scene, single), "above_objects", 4));
        CHECK(has_warning(custom_gcodes_json(info, Model(), single), "above_objects", 0));
    }

    SECTION("sequential printing") {
        CHECK_FALSE(has_warning(custom_gcodes_json(info, scene, single), "sequential_print"));
        single.set_deserialize_strict("complete_objects", "1");
        CHECK(has_warning(custom_gcodes_json(info, scene, single), "sequential_print"));
        CHECK_FALSE(has_warning(custom_gcodes_json(CustomGCode::Info(), scene, single), "sequential_print"));
    }

    SECTION("mode mismatch") {
        CHECK_FALSE(has_warning(custom_gcodes_json(info, scene, single), "mode_mismatch"));
        info.mode = CustomGCode::MultiExtruder;
        const nlohmann::json json = custom_gcodes_json(info, scene, single);
        CHECK(json.at("mode") == "multi_extruder");
        CHECK(has_warning(json, "mode_mismatch"));
        CustomGCode::Info texts_only;
        texts_only.mode = CustomGCode::MultiExtruder;
        texts_only.gcodes = {{scale_to_layer_coord(8.), CustomGCode::Custom, 1, "", "M117 Hello"}};
        CHECK_FALSE(has_warning(custom_gcodes_json(texts_only, scene, single), "mode_mismatch"));
        info.mode = CustomGCode::MultiAsSingle;
        CHECK_FALSE(has_warning(custom_gcodes_json(info, scene, make_session(2)), "mode_mismatch"));
        CHECK_FALSE(has_warning(custom_gcodes_json(info, scene, single), "mode_mismatch"));
    }
}
