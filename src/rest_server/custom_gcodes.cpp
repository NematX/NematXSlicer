/*
 * Headless custom G-code per height implementation.
 *
 * The engine applies each element to the first layer whose print_z is at or above
 * the element's z, so the stored list must be sorted by the layer-rounded print_z.
 * The mode (single extruder, one extruder of several, or several extruders) decides
 * which element types the engine honors; it is computed with the engine rule
 * (ToolOrdering::assign_custom_gcodes) by applying a Print on a scene copy, once per
 * request when the caller passes the mode on, and stored with the list.
 * Validation never touches the scene: it either returns the complete engine list
 * or throws before anything is written.
 */

#include "custom_gcodes.hpp"

#include "entity_settings.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/GCode/GCodeWriter.hpp"
#include "libslic3r/Print.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <set>
#include <unordered_set>
#include <utility>

namespace Slic3r::Rest {

namespace {

// The engine asserts z < 10000 mm when rounding to the layer grid; larger values are refused first.
constexpr double max_layer_z = 10000.;

// Return the top of the tallest instance of solid parts, or nullopt for an empty scene.
std::optional<double> scene_height(const Model &scene);

// Return the lowercase wire name of a mode.
std::string mode_name(CustomGCode::Mode mode);

// Return the wire name of an element type.
std::string type_name(CustomGCode::Type type);

// Return true and store the uppercase form when text is #RRGGBB.
bool normalize_color(const std::string &text, std::string &result);

// Return the color of the extruder from the session, or an empty string when unusable.
std::string default_extruder_color(const DynamicPrintConfig &session, int extruder);

// Validate one element and build its engine item; throws CustomGCodeItemError.
CustomGCode::Item make_item(std::size_t index, const CustomGCodeEntry &entry, const CustomGCodeMode &mode,
    std::size_t extruder_count, const std::optional<double> &height, const DynamicPrintConfig &session);

// Return true when z (already known to be finite) lies above the scene top on the layer grid.
bool above_scene(coord_t print_z, const std::optional<double> &height);

// Return true when the G-code the engine would emit for the key is empty.
bool effective_gcode_empty(const DynamicPrintConfig &session, const char *key);

// Return true when the session string option is missing or empty.
bool printer_gcode_empty(const DynamicPrintConfig &session, const char *key);

// Return true when the session enables sequential printing.
bool sequential_printing(const DynamicPrintConfig &session);

std::optional<double> scene_height(const Model &scene)
{
    std::optional<double> height;
    for (const ModelObject *object : scene.object_ptrs())
        for (std::size_t index = 0; index < object->instances.size(); ++index) {
            const BoundingBoxf3 box = object->instance_bounding_box(index);
            if (box.defined && (!height || box.max.z() > height.value()))
                height = box.max.z();
        }
    return height;
}

std::string mode_name(CustomGCode::Mode mode)
{
    switch (mode) {
    case CustomGCode::SingleExtruder: return "single_extruder";
    case CustomGCode::MultiAsSingle: return "multi_as_single";
    case CustomGCode::MultiExtruder: return "multi_extruder";
    default: return "undefined";
    }
}

std::string type_name(CustomGCode::Type type)
{
    switch (type) {
    case CustomGCode::ColorChange: return "color_change";
    case CustomGCode::PausePrint: return "pause";
    case CustomGCode::ToolChange: return "tool_change";
    case CustomGCode::Template: return "template";
    default: return "custom";
    }
}

bool normalize_color(const std::string &text, std::string &result)
{
    if (text.size() != 7 || text.front() != '#')
        return false;
    result = "#";
    for (std::size_t index = 1; index < text.size(); ++index) {
        if (!std::isxdigit(static_cast<unsigned char>(text[index])))
            return false;
        result += static_cast<char>(std::toupper(static_cast<unsigned char>(text[index])));
    }
    return true;
}

// Prefer extruder_colour, else filament_colour; a malformed or missing entry yields "".
std::string default_extruder_color(const DynamicPrintConfig &session, int extruder)
{
    const std::size_t index = static_cast<std::size_t>(extruder - 1);
    std::string candidate;
    for (const char *key : {"extruder_colour", "filament_colour"}) {
        const ConfigOptionStrings *colors = session.option<ConfigOptionStrings>(key);
        if (colors != nullptr && index < colors->size() && !colors->get_values()[index].empty()) {
            candidate = colors->get_values()[index];
            break;
        }
    }
    std::string normalized;
    return normalize_color(candidate, normalized) ? normalized : std::string();
}

// Checks run in a fixed order so the reported reason is deterministic: z, type and
// extruder, color, then the text payload.  Heights are compared on the layer grid, as
// the engine does, so a stored z is always accepted again on the same scene.  The stored
// extruder of text-only types is informative, as in the GUI.
CustomGCode::Item make_item(std::size_t index, const CustomGCodeEntry &entry, const CustomGCodeMode &mode,
    std::size_t extruder_count, const std::optional<double> &height, const DynamicPrintConfig &session)
{
    if (!std::isfinite(entry.z) || entry.z <= 0.)
        throw CustomGCodeItemError(index, "invalid_z");
    if (!height)
        throw CustomGCodeItemError(index, "empty_scene");
    if (entry.z >= max_layer_z)
        throw CustomGCodeItemError(index, "above_objects");
    const coord_t print_z = scale_to_layer_coord(entry.z);
    if (print_z <= 0)
        throw CustomGCodeItemError(index, "invalid_z");
    if (above_scene(print_z, height))
        throw CustomGCodeItemError(index, "above_objects");

    CustomGCode::Item item{print_z, entry.type, mode.extruder, std::string(), std::string()};
    const bool colored = entry.type == CustomGCode::ColorChange || entry.type == CustomGCode::ToolChange;
    if (entry.type == CustomGCode::ToolChange && mode.mode != CustomGCode::MultiAsSingle)
        throw CustomGCodeItemError(index, "mode_tool_change");
    if (entry.type == CustomGCode::ToolChange && !entry.extruder)
        throw CustomGCodeItemError(index, "extruder_required");
    if (colored) {
        item.extruder = entry.extruder.value_or(mode.extruder);
        if (item.extruder < 1 || static_cast<std::size_t>(item.extruder) > extruder_count)
            throw CustomGCodeItemError(index, "extruder_range");
        if (!entry.color)
            item.color = default_extruder_color(session, item.extruder);
        else if (!normalize_color(entry.color.value(), item.color))
            throw CustomGCodeItemError(index, "invalid_color");
    }
    if (entry.type == CustomGCode::PausePrint || entry.type == CustomGCode::Custom) {
        if (entry.text.empty())
            throw CustomGCodeItemError(index, entry.type == CustomGCode::PausePrint ? "empty_message" : "empty_gcode");
        item.extra = entry.text;
    }
    return item;
}

// A negative top lies below every positive height, and a top or height reaching the rounding
// limit is refused, so scale_to_layer_coord (which asserts 0 <= z < 10000) is only called in range.
bool above_scene(coord_t print_z, const std::optional<double> &height)
{
    return !height || height.value() < 0. || height.value() >= max_layer_z || print_z >= scale_i(max_layer_z) ||
        print_z > scale_to_layer_coord(height.value());
}

// Use the option when set, else the flavor default of the writer (GCodeConfig holds neither
// plugin-defined key, so the writer helpers fall back to the default of the session's flavor).
bool effective_gcode_empty(const DynamicPrintConfig &session, const char *key)
{
    if (!printer_gcode_empty(session, key))
        return false;
    GCodeConfig config;
    config.apply_only(session, std::set<t_config_option_key>{"gcode_flavor"}, true);
    return std::string(key) == "color_change_gcode" ? GCodeWriter::get_default_color_change_gcode(config).empty() :
        GCodeWriter::get_default_pause_gcode(config).empty();
}

bool printer_gcode_empty(const DynamicPrintConfig &session, const char *key)
{
    const ConfigOptionString *option = session.option<ConfigOptionString>(key);
    return option == nullptr || option->value.empty();
}

bool sequential_printing(const DynamicPrintConfig &session)
{
    const ConfigOptionBool *option = session.option<ConfigOptionBool>("complete_objects");
    return option != nullptr && option->value;
}

} // namespace

// Store a 422 element error with its zero-based index and reason.
CustomGCodeItemError::CustomGCodeItemError(std::size_t item, std::string reason)
    : SceneEditError(422, "invalid_custom_gcode", "Custom G-code item " + std::to_string(item) +
        " is invalid: " + reason),
      item_(item), reason_(std::move(reason))
{
}

// Return the stored zero-based element index.
std::size_t CustomGCodeItemError::item() const noexcept
{
    return item_;
}

// Return the stored reason.
const std::string &CustomGCodeItemError::reason() const noexcept
{
    return reason_;
}

// Same rule as ToolOrdering::assign_custom_gcodes: one extruder in total is
// single_extruder, one used extruder (modifiers, ranges and MMU painting included, as
// Print::apply resolves them) is multi_as_single.  The Print is only applied, never sliced.
CustomGCodeMode current_custom_gcode_mode(const Model &scene, const DynamicPrintConfig &session)
{
    if (session_extruder_count(session) <= 1 || scene.object_ptrs().empty())
        return {CustomGCode::SingleExtruder, 1};
    Model print_scene(scene);
    Print print;
    Orchestrator::RunContext run_context;
    Orchestrator::RunScope run_scope = Orchestrator::instance().enter_run(run_context, &print);
    for (ModelObject *object : print_scene.object_ptrs()) {
        // The mode depends on region configs only: keep every range bound below the largest value
        // scale_to_layer_coord accepts (10000 mm, asserted), dropping ranges that start beyond it.
        const double cap = max_layer_z - EPSILON;
        t_layer_config_ranges ranges;
        for (const std::pair<const t_layer_height_range, ModelConfig> &range : object->layer_config_ranges)
            if (range.first.first < cap)
                ranges.emplace(t_layer_height_range(range.first.first, std::min(range.first.second, cap)), range.second);
        object->layer_config_ranges = std::move(ranges);
        print.auto_assign_extruders(object);
    }
    print.apply(print_scene, session);
    const std::set<extruder_index_t> used = print.object_extruders();
    if (used.size() == 1)
        return {CustomGCode::MultiAsSingle, static_cast<int>(*used.begin()) + 1};
    return {CustomGCode::MultiExtruder, 1};
}

// An empty list is always accepted; otherwise every element is validated against one mode
// and one scene height, and the result is stably sorted by the layer-rounded print_z.
CustomGCode::Info make_custom_gcodes(const std::vector<CustomGCodeEntry> &entries,
    const Model &scene, const DynamicPrintConfig &session)
{
    CustomGCode::Info result;
    if (sequential_printing(session) && !entries.empty())
        throw SceneEditError(422, "invalid_custom_gcode",
            "Custom G-codes cannot be used with sequential printing.");
    const CustomGCodeMode mode = current_custom_gcode_mode(scene, session);
    result.mode = mode.mode;
    if (entries.empty())
        return result;

    const std::size_t extruder_count = session_extruder_count(session);
    const std::optional<double> height = scene_height(scene);
    for (std::size_t index = 0; index < entries.size(); ++index)
        result.gcodes.push_back(make_item(index, entries[index], mode, extruder_count, height, session));
    std::unordered_set<coord_t> seen;
    for (std::size_t index = 0; index < result.gcodes.size(); ++index)
        if (!seen.insert(result.gcodes[index].print_z_).second)
            throw CustomGCodeItemError(index, "duplicate_z");
    std::stable_sort(result.gcodes.begin(), result.gcodes.end());
    return result;
}

// Compute the current mode, then serialize.
nlohmann::json custom_gcodes_json(const CustomGCode::Info &info, const Model &scene,
    const DynamicPrintConfig &session)
{
    return custom_gcodes_json(info, scene, session, current_custom_gcode_mode(scene, session).mode);
}

// Warnings come after the items: per-item ones in list order, then the global ones.  The
// scene top is only measured when there is an item to compare it with, and each effective
// printer G-code is checked at most once.
nlohmann::json custom_gcodes_json(const CustomGCode::Info &info, const Model &scene,
    const DynamicPrintConfig &session, CustomGCode::Mode current)
{
    const std::optional<double> height = info.gcodes.empty() ? std::nullopt : scene_height(scene);
    nlohmann::json items = nlohmann::json::array();
    nlohmann::json warnings = nlohmann::json::array();
    const auto warn = [&warnings](std::size_t item, const char *reason) {
        warnings.push_back({{"item", item}, {"reason", reason}});
    };
    std::optional<bool> color_empty;
    std::optional<bool> pause_empty;
    const auto printer_empty = [&](std::optional<bool> &cache, const char *key) {
        if (!cache)
            cache = effective_gcode_empty(session, key);
        return cache.value();
    };
    bool has_colors = false;
    for (std::size_t index = 0; index < info.gcodes.size(); ++index) {
        const CustomGCode::Item &item = info.gcodes[index];
        nlohmann::json entry = {{"z", unscaled(item.print_z_)}, {"type", type_name(item.type)}};
        if (item.type == CustomGCode::ColorChange || item.type == CustomGCode::ToolChange) {
            entry["extruder"] = item.extruder;
            if (!item.color.empty())
                entry["color"] = item.color;
            has_colors = true;
        } else if (item.type == CustomGCode::PausePrint) {
            entry["message"] = item.extra;
        } else if (item.type == CustomGCode::Custom) {
            entry["gcode"] = item.extra;
        }
        items.push_back(std::move(entry));

        if ((item.type == CustomGCode::ColorChange && printer_empty(color_empty, "color_change_gcode")) ||
            (item.type == CustomGCode::PausePrint && printer_empty(pause_empty, "pause_print_gcode")) ||
            (item.type == CustomGCode::Template && printer_gcode_empty(session, "template_custom_gcode")))
            warn(index, "empty_printer_gcode");
        if (above_scene(item.print_z_, height))
            warn(index, "above_objects");
    }
    if (sequential_printing(session) && !info.gcodes.empty())
        warnings.push_back({{"reason", "sequential_print"}});
    if (has_colors && (info.mode == CustomGCode::MultiExtruder) != (current == CustomGCode::MultiExtruder))
        warnings.push_back({{"reason", "mode_mismatch"}});
    return {{"mode", mode_name(info.mode)}, {"current_mode", mode_name(current)}, {"items", std::move(items)},
        {"warnings", std::move(warnings)}};
}

} // namespace Slic3r::Rest
