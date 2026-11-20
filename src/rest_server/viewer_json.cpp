// Build the viewer JSON of §5.15 from an immutable GCodeProcessorResult.
#include "viewer_json.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#ifdef __APPLE__
#include <iomanip>
#include <locale>
#include <sstream>
#endif

#include <boost/filesystem/path.hpp>
#include <fast_float/fast_float.h>

namespace Slic3r::Rest {

namespace {

nlohmann::json number(float value)
{
    return shortest_float(value);
}

nlohmann::json point_json(const Vec3f &point)
{
    return nlohmann::json::array({number(point.x()), number(point.y()), number(point.z())});
}

// Strings are optional in the result vectors: a missing entry is null.
template <typename Value>
nlohmann::json entry_or_null(const std::vector<Value> &values, std::size_t index)
{
    if (index >= values.size())
        return nullptr;
    if constexpr (std::is_same<Value, float>::value)
        return number(values[index]);
    else
        return values[index];
}

nlohmann::json layer_id_json(const ViewerHeader &header, std::uint16_t processor_layer)
{
    if (header.layer_index.empty())
        return nullptr;
    const std::size_t index = std::min<std::size_t>(processor_layer, header.layer_index.size() - 1);
    return header.layer_index[index];
}

// Complete JSON of one move, before delta encoding.
nlohmann::json full_move_json(const ViewerMove &move, const ViewerHeader &header)
{
    const GCodeProcessorResult::MoveVertex &source = move.source;
    nlohmann::json points = nlohmann::json::array();
    for (const Vec3f &point : move.points)
        points.push_back(point_json(point));
    const std::optional<const char *> role = viewer_role_name(source.extrusion_role);
    return nlohmann::json{
        {"gcode_id", source.gcode_id},
        {"type", viewer_move_type_name(source.type)},
        {"role", role ? nlohmann::json(*role) : nlohmann::json(nullptr)},
        {"extruder_id", source.extruder_id},
        {"color_change_id", source.cp_color_id},
        {"object_id", source.object_id == GCodeProcessorResult::OBJECT_INSTANCE_ID_NONE ?
            nlohmann::json(nullptr) : nlohmann::json(source.object_id)},
        {"position_mm", point_json(source.position)},
        {"extrusion_delta_mm", number(source.delta_extruder)},
        {"feedrate_start_mm_s", number(move.feedrate_start_mm_s)},
        {"feedrate_end_mm_s", number(move.feedrate_end_mm_s)},
        {"width_mm", number(source.width)},
        {"height_mm", number(source.height)},
        {"mm3_per_mm", number(source.mm3_per_mm)},
        {"fan_speed_percent", number(source.fan_speed)},
        {"temperature_c", number(source.temperature)},
        {"time_s", number(move.time_s)},
        {"layer_id", layer_id_json(header, source.layer_id)},
        {"internal_only", source.internal_only},
        {"points", std::move(points)}
    };
}

} // namespace

double shortest_float(float value)
{
    if (!std::isfinite(value))
        return value;
    // Preserve the 6..9 significant-digit projection, without constructing
    // locale-aware streams for every coordinate/property of every move.
    // charconv and fast_float are locale-independent; fast_float parses
    // because macOS libc++ has std::from_chars only for integers.
    char text[64];
#ifdef __APPLE__
    // Floating-point to_chars requires a newer macOS than the deployment target.
    std::ostringstream stream;
    stream.imbue(std::locale::classic());
#endif
    for (int precision = 6; precision <= 9; ++precision) {
#ifdef __APPLE__
        stream.str(std::string{});
        stream << std::setprecision(precision) << value;
        const std::string formatted_text = stream.str();
        if (formatted_text.size() >= sizeof(text))
            throw std::logic_error("Cannot format viewer float.");
        std::copy(formatted_text.begin(), formatted_text.end(), text);
        const char *formatted_end = text + formatted_text.size();
#else
        const std::to_chars_result formatted = std::to_chars(text, text + sizeof(text), value,
            std::chars_format::general, precision);
        if (formatted.ec != std::errc{})
            throw std::logic_error("Cannot format viewer float.");
        const char *formatted_end = formatted.ptr;
#endif
        float restored = 0;
        const fast_float::from_chars_result parsed = fast_float::from_chars(text, formatted_end, restored);
        if ((parsed.ec == std::errc{} && restored == value) || precision == 9) {
            double projected = 0;
            if (fast_float::from_chars(text, formatted_end, projected).ec != std::errc{})
                throw std::logic_error("Cannot parse viewer decimal.");
            return projected;
        }
    }
    throw std::logic_error("Cannot project viewer float.");
}

// No default branch: a new enum value must be given a stable name here.
const char *viewer_move_type_name(EMoveType type)
{
    switch (type) {
    case EMoveType::Noop: return "noop";
    case EMoveType::Retract: return "retract";
    case EMoveType::Unretract: return "unretract";
    case EMoveType::Seam: return "seam";
    case EMoveType::Tool_change: return "tool_change";
    case EMoveType::Color_change: return "color_change";
    case EMoveType::Pause_Print: return "pause_print";
    case EMoveType::Custom_GCode: return "custom_gcode";
    case EMoveType::Travel: return "travel";
    case EMoveType::Wipe: return "wipe";
    case EMoveType::Extrude: return "extrude";
    case EMoveType::Count: break;
    }
    throw std::logic_error("Unknown G-code move type.");
}

std::optional<const char *> viewer_role_name(GCodeExtrusionRole role)
{
    switch (role) {
    case GCodeExtrusionRole::None: return std::nullopt;
    case GCodeExtrusionRole::Perimeter: return "perimeter";
    case GCodeExtrusionRole::ExternalPerimeter: return "external_perimeter";
    case GCodeExtrusionRole::OverhangPerimeter: return "overhang_perimeter";
    case GCodeExtrusionRole::InternalInfill: return "internal_infill";
    case GCodeExtrusionRole::InternalBridgeInfill: return "internal_bridge_infill";
    case GCodeExtrusionRole::SolidInfill: return "solid_infill";
    case GCodeExtrusionRole::TopSolidInfill: return "top_solid_infill";
    case GCodeExtrusionRole::Ironing: return "ironing";
    case GCodeExtrusionRole::BridgeInfill: return "bridge_infill";
    case GCodeExtrusionRole::ThinWall: return "thin_wall";
    case GCodeExtrusionRole::GapFill: return "gap_fill";
    case GCodeExtrusionRole::Skirt: return "skirt";
    case GCodeExtrusionRole::SupportMaterial: return "support_material";
    case GCodeExtrusionRole::SupportMaterialInterface: return "support_material_interface";
    case GCodeExtrusionRole::WipeTower: return "wipe_tower";
    case GCodeExtrusionRole::Milling: return "milling";
    case GCodeExtrusionRole::Custom: return "custom";
    case GCodeExtrusionRole::Travel: return "travel";
    case GCodeExtrusionRole::Count: break;
    }
    throw std::logic_error("Unknown G-code extrusion role.");
}

std::shared_ptr<ViewerHeader> make_viewer_header(const GCodeProcessorResult &result)
{
    std::shared_ptr<ViewerHeader> header = std::make_shared<ViewerHeader>();

    nlohmann::json bed = nlohmann::json::array();
    for (const Vec2d &point : result.bed_shape)
        bed.push_back(nlohmann::json::array({point.x(), point.y()}));
    nlohmann::json extruders = nlohmann::json::array();
    for (std::size_t index = 0; index < result.extruders_count; ++index)
        extruders.push_back({
            {"id", index},
            {"extruder_color", entry_or_null(result.extruder_colors, index)},
            {"filament_color", entry_or_null(result.filament_colors, index)},
            {"filament_diameter_mm", entry_or_null(result.filament_diameters, index)},
            {"filament_density_g_cm3", entry_or_null(result.filament_densities, index)}});
    nlohmann::json objects = nlohmann::json::array();
    for (std::size_t index = 0; index < result.object_names.size(); ++index)
        objects.push_back({{"id", index}, {"name", result.object_names[index]}});
    nlohmann::json volumes = nlohmann::json::array();
    for (std::size_t index = 0; index < result.extruders_count; ++index) {
        const auto found = result.print_statistics.volumes_per_extruder.find(static_cast<extruder_index_t>(index));
        volumes.push_back(found == result.print_statistics.volumes_per_extruder.end() ? 0.0 : found->second);
    }
    using TimeMode = PrintEstimatedStatistics::ETimeMode;
    const float normal_time = result.print_statistics.modes[static_cast<std::size_t>(TimeMode::Normal)].time;
    const float stealth_time = result.print_statistics.modes[static_cast<std::size_t>(TimeMode::Stealth)].time;
    header->metadata = {
        // The processor stores the server output path: expose its file name only.
        {"filename", boost::filesystem::path(result.filename).filename().string()},
        {"is_binary", result.is_binary_file},
        {"bed_shape_mm", std::move(bed)},
        {"max_print_height_mm", number(result.max_print_height)},
        {"z_offset_mm", number(result.z_offset)},
        {"extruder_count", result.extruders_count},
        {"extruders", std::move(extruders)},
        {"objects", std::move(objects)},
        {"statistics", {
            // A disabled stealth estimator leaves its mode reset to zero.
            {"estimated_time_s", {{"normal", number(normal_time)},
                {"stealth", stealth_time > 0.f ? number(stealth_time) : nlohmann::json(nullptr)}}},
            {"filament_volume_mm3_by_extruder", std::move(volumes)}}}
    };

    // Counted layers: a processor layer with an extrusion, in order, at the
    // height of its first extrusion (GCodeViewer layer detection).
    std::vector<std::uint16_t> counted_layers;
    std::uint16_t max_layer = 0;
    header->layers = nlohmann::json::array();
    for (const GCodeProcessorResult::MoveVertex &move : result.moves) {
        max_layer = std::max(max_layer, move.layer_id);
        if (move.type != EMoveType::Extrude || (!counted_layers.empty() && move.layer_id <= counted_layers.back()))
            continue;
        header->layers.push_back({{"id", counted_layers.size()}, {"z_mm", number(move.position.z())}});
        counted_layers.push_back(move.layer_id);
    }
    if (!counted_layers.empty()) {
        header->layer_index.resize(std::size_t(max_layer) + 1);
        for (std::size_t processor_layer = 0; processor_layer < header->layer_index.size(); ++processor_layer) {
            const auto next = std::lower_bound(counted_layers.begin(), counted_layers.end(), processor_layer);
            header->layer_index[processor_layer] = static_cast<std::uint32_t>(
                next == counted_layers.end() ? counted_layers.size() - 1 : next - counted_layers.begin());
        }
    }
    return header;
}

nlohmann::json viewer_moves_json(const ViewerPage &page, const ViewerHeader &header)
{
    nlohmann::json moves = nlohmann::json::array();
    nlohmann::json previous;
    for (const ViewerMove &move : page.moves) {
        nlohmann::json full = full_move_json(move, header);
        if (previous.is_null()) {
            moves.push_back(full);
        } else {
            nlohmann::json sparse = nlohmann::json::object();
            for (auto item = full.begin(); item != full.end(); ++item)
                if (item.key() == "points" || previous.at(item.key()) != item.value())
                    sparse[item.key()] = item.value();
            moves.push_back(std::move(sparse));
        }
        previous = std::move(full);
    }
    return moves;
}

} // namespace Slic3r::Rest
