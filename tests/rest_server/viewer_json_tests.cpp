// Pure tests of the viewer JSON: number formatting, stable names, layer
// mapping and the sparse move encoding, on synthetic processor results.
#include <catch2/catch.hpp>

#include <cstdint>
#include <cmath>
#include <cstring>
#include <limits>
#include <locale>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "viewer_json.hpp"

using namespace Slic3r;
using namespace Slic3r::Rest;

namespace {

// Compatibility oracle for the established stream-based decimal projection.
// Keep it in tests only: production must not pay stream/locale costs per field.
double stream_projection(float value);

double stream_projection(float value)
{
    for (int precision = 6; precision <= 9; ++precision) {
        std::ostringstream output;
        output.imbue(std::locale::classic());
        output.precision(precision);
        output << value;
        std::istringstream input(output.str());
        input.imbue(std::locale::classic());
        float restored = 0;
        input >> restored;
        if (restored == value || precision == 9) {
            std::istringstream widened(output.str());
            widened.imbue(std::locale::classic());
            double projected = 0;
            widened >> projected;
            return projected;
        }
    }
    throw std::logic_error("Stream projection did not terminate.");
}

GCodeProcessorResult::MoveVertex vertex(std::uint32_t id, EMoveType type, std::uint16_t layer, float z,
    std::uint16_t object = GCodeProcessorResult::OBJECT_INSTANCE_ID_NONE, float width = 0.45f)
{
    return GCodeProcessorResult::MoveVertex(id, type,
        type == EMoveType::Extrude ? GCodeExtrusionRole::Perimeter : GCodeExtrusionRole::None, 0, 0, object,
        Vec3f(float(id), 0.f, z), 0.f, 30.f, width, 0.2f, 0.1f, 0.f, 210.f, float(id), layer, false);
}

// Reverse of the sparse encoding: carry omitted fields from the previous move.
std::vector<nlohmann::json> decode(const nlohmann::json &moves)
{
    std::vector<nlohmann::json> decoded;
    for (const nlohmann::json &move : moves) {
        nlohmann::json full = decoded.empty() ? nlohmann::json::object() : decoded.back();
        for (auto item = move.begin(); item != move.end(); ++item)
            full[item.key()] = item.value();
        decoded.push_back(std::move(full));
    }
    return decoded;
}

} // namespace

TEST_CASE("Viewer numbers use the shortest decimal of the engine float", "[rest_viewer]")
{
    CHECK(nlohmann::json(shortest_float(0.2f)).dump() == "0.2");
    CHECK(nlohmann::json(shortest_float(1.75f)).dump() == "1.75");
    CHECK(nlohmann::json(shortest_float(210.f)).dump() == "210.0");
    CHECK(float(shortest_float(1e-7f)) == 1e-7f);
    std::mt19937 random(42);
    std::uniform_real_distribution<float> values(-1000.f, 1000.f);
    for (int index = 0; index < 10000; ++index) {
        const float value = values(random);
        CAPTURE(value);
        CHECK(float(shortest_float(value)) == value);
    }
}

TEST_CASE("Viewer charconv projection preserves stream JSON and float bits", "[rest_viewer]")
{
    const std::vector<float> boundaries = {0.f, -0.f, 0.2f, -0.2f, 210.f,
        std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::min(), std::numeric_limits<float>::denorm_min()};
    std::mt19937 random(93);
    for (std::size_t index = 0; index < 10000 + boundaries.size(); ++index) {
        float value;
        if (index < boundaries.size()) {
            value = boundaries[index];
        } else {
            const std::uint32_t bits = random();
            std::memcpy(&value, &bits, sizeof(value));
        }
        if (!std::isfinite(value))
            continue;
        CAPTURE(value);
        const double projected = shortest_float(value);
        CHECK(nlohmann::json(projected).dump() == nlohmann::json(stream_projection(value)).dump());
        CHECK(float(projected) == value);
        if (value == 0.f)
            CHECK(std::signbit(projected) == std::signbit(value));
    }
    CHECK(nlohmann::json(shortest_float(std::numeric_limits<float>::infinity())).is_number_float());
    CHECK(nlohmann::json(shortest_float(std::numeric_limits<float>::infinity())).dump() == "null");
}

TEST_CASE("Viewer move types and roles have stable unique names", "[rest_viewer]")
{
    std::set<std::string> types;
    for (int type = 0; type < int(EMoveType::Count); ++type)
        types.insert(viewer_move_type_name(EMoveType(type)));
    CHECK(types == std::set<std::string>{"noop", "retract", "unretract", "seam", "tool_change", "color_change",
        "pause_print", "custom_gcode", "travel", "wipe", "extrude"});
    CHECK_THROWS_AS(viewer_move_type_name(EMoveType::Count), std::logic_error);

    std::set<std::string> roles;
    CHECK_FALSE(viewer_role_name(GCodeExtrusionRole::None).has_value());
    for (int role = 1; role < int(GCodeExtrusionRole::Count); ++role)
        roles.insert(viewer_role_name(GCodeExtrusionRole(role)).value());
    CHECK(roles == std::set<std::string>{"perimeter", "external_perimeter", "overhang_perimeter", "internal_infill",
        "internal_bridge_infill", "solid_infill", "top_solid_infill", "ironing", "bridge_infill", "thin_wall",
        "gap_fill", "skirt", "support_material", "support_material_interface", "wipe_tower", "milling", "custom",
        "travel"});
}

// The processor always resets its result; a default-constructed one leaves
// scalar fields such as extruders_count uninitialized.
GCodeProcessorResult empty_result()
{
    GCodeProcessorResult result;
    result.reset();
    return result;
}

TEST_CASE("Viewer header counts extruded layers like the GUI", "[rest_viewer]")
{
    GCodeProcessorResult result = empty_result();
    result.filename = "C:/server/work/results/rest-result-x/output.gcode";
    result.extruders_count = 2;
    result.extruder_colors = {"#FFFFFF"};
    result.filament_diameters = {1.75f, 2.85f};
    result.object_names = {"cube.stl"};
    // Start G-code in processor layer 0, first layer at 2, layer 3 without
    // extrusion, then layer 4.
    result.moves.push_back(vertex(1, EMoveType::Travel, 0, 5.f));
    result.moves.push_back(vertex(2, EMoveType::Extrude, 2, 0.2f, 0));
    result.moves.push_back(vertex(3, EMoveType::Extrude, 2, 0.2f, 0));
    result.moves.push_back(vertex(4, EMoveType::Travel, 3, 0.4f));
    result.moves.push_back(vertex(5, EMoveType::Extrude, 4, 0.4f, 0));
    result.moves.push_back(vertex(6, EMoveType::Travel, 5, 10.f));

    const std::shared_ptr<ViewerHeader> header = make_viewer_header(result);
    CHECK(header->layers == nlohmann::json::parse(R"([{"id":0,"z_mm":0.2},{"id":1,"z_mm":0.4}])"));
    CHECK(header->layer_index == std::vector<std::uint32_t>{0, 0, 0, 1, 1, 1});
    CHECK(header->metadata.at("filename") == "output.gcode");
    CHECK(header->metadata.at("objects") == nlohmann::json::parse(R"([{"id":0,"name":"cube.stl"}])"));
    const nlohmann::json &extruders = header->metadata.at("extruders");
    REQUIRE(extruders.size() == 2);
    CHECK(extruders[0].at("extruder_color") == "#FFFFFF");
    CHECK(extruders[1].at("extruder_color").is_null());
    CHECK(extruders[1].at("filament_diameter_mm") == 2.85);
    CHECK(header->metadata.at("statistics").at("estimated_time_s").at("stealth").is_null());
    CHECK(header->metadata.at("statistics").at("filament_volume_mm3_by_extruder").size() == 2);

    GCodeProcessorResult travel_only = empty_result();
    travel_only.moves.push_back(vertex(1, EMoveType::Travel, 0, 1.f));
    const std::shared_ptr<ViewerHeader> empty = make_viewer_header(travel_only);
    CHECK(empty->layers.empty());
    CHECK(empty->layer_index.empty());
    const ViewerPage page = make_viewer_page(travel_only, 0, 1);
    CHECK(viewer_moves_json(page, *empty)[0].at("layer_id").is_null());
}

TEST_CASE("Viewer moves are delta-encoded within a page", "[rest_viewer]")
{
    GCodeProcessorResult result = empty_result();
    result.moves.push_back(vertex(1, EMoveType::Extrude, 0, 0.2f, 0));
    result.moves.push_back(vertex(2, EMoveType::Extrude, 0, 0.2f, 0));
    result.moves.push_back(vertex(3, EMoveType::Travel, 0, 0.2f));
    result.moves.push_back(vertex(4, EMoveType::Retract, 0, 0.2f, GCodeProcessorResult::OBJECT_INSTANCE_ID_NONE, 0.f));
    const std::shared_ptr<ViewerHeader> header = make_viewer_header(result);
    const ViewerPage page = make_viewer_page(result, 0, 4);
    const nlohmann::json moves = viewer_moves_json(page, *header);
    REQUIRE(moves.size() == 4);

    // First move complete; a same-role extrusion only repeats what changed.
    CHECK(moves[0].size() == 19);
    CHECK(moves[0].at("role") == "perimeter");
    CHECK(moves[0].at("object_id") == 0);
    CHECK_FALSE(moves[1].contains("role"));
    CHECK_FALSE(moves[1].contains("width_mm"));
    CHECK(moves[1].contains("gcode_id"));
    CHECK(moves[1].contains("points"));
    // Changes to null are explicit.
    CHECK(moves[2].at("role").is_null());
    CHECK(moves[2].at("object_id").is_null());
    // A non-spatial move keeps an explicit empty path.
    CHECK(moves[3].at("points") == nlohmann::json::array());

    // Each page decodes on its own: a page starting at move 2 is complete.
    const nlohmann::json tail = viewer_moves_json(make_viewer_page(result, 2, 2), *header);
    CHECK(tail[0].size() == 19);
    const std::vector<nlohmann::json> all = decode(moves);
    CHECK(decode(tail)[0] == all[2]);
    CHECK(decode(tail)[1] == all[3]);
    CHECK(all[3].at("position_mm") == nlohmann::json::array({4.0, 0.0, 0.2}));
}
