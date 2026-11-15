///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "GCodeReaderResultBuilder.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "libslic3r/Api/plugin/c/slic3r_extrusions.h"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/ExtrusionRole.hpp"
#include "libslic3r/I18N.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"

/*
Host Reader timeline implementation
===================================

The implementation intentionally contains no command strings. Reader plugins
own syntax; the host owns presentation and aggregate calculations. A movement
listener compares the state before and after an event, classifies the move,
then updates the viewer vertex and statistics from the same normalized delta.
Consumption is accumulated by extrusion role and logical tool, independently
of object labels. Single-extruder multi-material (SEMM) shares one retraction
debt in filament mm, while E coordinates remain per logical tool.

Some viewer metadata is textual or structurally richer than the numeric Reader
event vocabulary. Listeners below carry focused TODO comments for missing
payloads; syntax parsing remains the Reader plugin's responsibility.
*/

namespace Slic3r {

namespace {

constexpr float DEFAULT_VIEWER_FILAMENT_DIAMETER = 1.75f;
constexpr float DEFAULT_VIEWER_FILAMENT_DENSITY = 1.245f;
constexpr float DEFAULT_VIEWER_FILAMENT_COST = 0.0f;
constexpr float DEFAULT_VIEWER_TOOLPATH_HEIGHT = 0.2f;
constexpr double MM_PER_ARC_SEGMENT = 1.0;

using RegionIslandLookup = std::unordered_map<const LayerRegionIsland *,
                                              const LayerSliceIsland *>;

// Reader state carries tool ids as uint32_t. Convert only after enforcing the
// signed public identity range used by the historical viewer result.
extruder_index_t checked_extruder_index(const uint32_t tool)
{
    if (tool > static_cast<uint32_t>(std::numeric_limits<extruder_index_t>::max()))
        throw std::out_of_range(
            "A Reader extruder id cannot be represented by extruder_index_t.");
    return static_cast<extruder_index_t>(tool);
}

// Record the owning geometric island for every region bucket of one trusted
// source layer. The lookup lets the plan recover boundaries without storing a
// redundant parent pointer in each PrintingExtrusion.
void index_source_layer_islands(const Layer &layer, RegionIslandLookup &lookup);

// Replace the public per-layer area series from the final plan. Every logical
// layer gets one entry, and a source island is counted only once even when
// several region buckets or extrusion roles refer to it.
void initialize_layer_area_stats(Print &print,
                                 const Printing::PrintingPlan &plan);

// Give independently produced results distinct cache identities.
std::atomic<unsigned int> s_reader_result_id{0};

// Read one scalar without converting an unknown value into a physical zero.
std::optional<double> event_value(const raw_gcode_reader_event &event,
                                  uint32_t type,
                                  uint32_t target = 0)
{
    for (uint32_t index = 0; index < event.item_count; ++index) {
        const raw_gcode_reader_event_item &item = event.items[index];
        if (item.type == type && item.target == target)
            return item.value;
    }
    return std::nullopt;
}

// Return a known coordinate or the supplied fallback. Movement events are
// skipped when their position is unknowable, so this is used only after that
// check or for non-moving markers.
double state_value_or(const slic3r_api::GCodeStateTracker &state,
                      uint32_t type,
                      double fallback,
                      uint32_t target = 0)
{
    const std::optional<double> value = state.value(type, target);
    return value ? *value : fallback;
}

// Reader results use the same zero-based object identity as PrintingPlan. The
// compact viewer field cannot carry UINT32_MAX, so it uses its own all-bits-one
// value for movements outside every object. The legacy GCodeProcessor now uses
// the same viewer convention, so both input paths produce interchangeable IDs.
uint16_t viewer_object_id(const slic3r_api::GCodeStateTracker &state)
{
    const std::optional<double> raw = state.value(RAW_GCODE_READER_ITEM_OBJECT);
    if (!raw || static_cast<uint32_t>(*raw) == UINT32_MAX)
        return GCodeProcessorResult::OBJECT_INSTANCE_ID_NONE;
    const uint32_t object_id = static_cast<uint32_t>(*raw);
    if (object_id >= GCodeProcessorResult::OBJECT_INSTANCE_ID_NONE)
        throw std::out_of_range(
            "A Reader object instance id cannot be represented by MoveVertex.");
    return static_cast<uint16_t>(object_id);
}

// Read the validated uint32 LAYER item, default missing state to 0, and clamp it
// to MoveVertex::layer_id's uint16 range. This borrows state and does not mutate it.
uint32_t viewer_layer_id(const slic3r_api::GCodeStateTracker &state)
{
    return std::min<uint32_t>(static_cast<uint32_t>(
        state.value(RAW_GCODE_READER_ITEM_LAYER).value_or(0.0)), UINT16_MAX);
}

// Convert the public extrusion-role bit mask to the compact viewer category.
// Invalid or mixed masks stay visible as None instead of asserting in host UI.
GCodeExtrusionRole viewer_role(const slic3r_api::GCodeStateTracker &state)
{
    const std::optional<double> value = state.value(RAW_GCODE_READER_ITEM_EXTRUSION_ROLE);
    if (!value)
        return GCodeExtrusionRole::None;
    const uint32_t role = static_cast<uint32_t>(*value);
    const bool perimeter = (role & RAW_EXTRUSION_ROLE_PERIMETER) != 0;
    const bool infill = (role & RAW_EXTRUSION_ROLE_INFILL) != 0;
    const bool support = (role & RAW_EXTRUSION_ROLE_SUPPORT) != 0;
    if ((role & RAW_EXTRUSION_ROLE_TRAVEL) != 0)
        return GCodeExtrusionRole::Travel;
    if ((role & RAW_EXTRUSION_ROLE_WIPE_TOWER) != 0)
        return GCodeExtrusionRole::WipeTower;
    if ((role & RAW_EXTRUSION_ROLE_MILL) != 0)
        return GCodeExtrusionRole::Milling;
    if ((role & RAW_EXTRUSION_ROLE_SKIRT) != 0)
        return GCodeExtrusionRole::Skirt;
    if (support)
        return (role & RAW_EXTRUSION_ROLE_EXTERNAL) != 0 ?
            GCodeExtrusionRole::SupportMaterialInterface :
            GCodeExtrusionRole::SupportMaterial;
    if (perimeter) {
        if ((role & RAW_EXTRUSION_ROLE_BRIDGE) != 0)
            return GCodeExtrusionRole::OverhangPerimeter;
        return (role & RAW_EXTRUSION_ROLE_EXTERNAL) != 0 ?
            GCodeExtrusionRole::ExternalPerimeter : GCodeExtrusionRole::Perimeter;
    }
    if (infill) {
        if ((role & RAW_EXTRUSION_ROLE_IRONING) != 0)
            return GCodeExtrusionRole::Ironing;
        if ((role & RAW_EXTRUSION_ROLE_BRIDGE) != 0)
            return (role & RAW_EXTRUSION_ROLE_EXTERNAL) != 0 ?
                GCodeExtrusionRole::BridgeInfill : GCodeExtrusionRole::InternalBridgeInfill;
        if ((role & RAW_EXTRUSION_ROLE_SOLID) != 0)
            return (role & RAW_EXTRUSION_ROLE_EXTERNAL) != 0 ?
                GCodeExtrusionRole::TopSolidInfill : GCodeExtrusionRole::SolidInfill;
        return GCodeExtrusionRole::InternalInfill;
    }
    if ((role & RAW_EXTRUSION_ROLE_THIN) != 0)
        return (role & RAW_EXTRUSION_ROLE_EXTERNAL) != 0 ?
            GCodeExtrusionRole::ThinWall : GCodeExtrusionRole::GapFill;
    return GCodeExtrusionRole::None;
}

// Keep vector-valued settings aligned with the number of logical tools.
template<class Value>
Value vector_value_or(const std::vector<Value> &values, size_t index, Value fallback)
{
    if (values.empty())
        return fallback;
    return index < values.size() ? values[index] : values.front();
}

void index_source_layer_islands(const Layer &layer, RegionIslandLookup &lookup)
{
    for (const LayerSliceIsland &island : layer.islands())
        for (const LayerRegionIsland &region_island : island.regions_islands()) {
            const std::pair<RegionIslandLookup::iterator, bool> inserted =
                lookup.emplace(&region_island, &island);
            if (!inserted.second && inserted.first->second != &island)
                throw std::logic_error(
                    "A LayerRegionIsland belongs to more than one LayerSliceIsland.");
        }
}

void initialize_layer_area_stats(Print &print,
                                 const Printing::PrintingPlan &plan)
{
    /*
    Build the parent relation from Print-owned layers instead of trusting plan
    pointers. A plan may also contain synthetic or print-level auxiliary work;
    such region-island pointers are intentionally absent from this lookup and
    therefore contribute no model cross-section.
    */
    RegionIslandLookup lookup;
    for (const PrintObject &object : print.objects()) {
        for (const Layer &layer : object.layers())
            index_source_layer_islands(layer, lookup);
        for (const Layer &layer : object.auxiliary_layers())
            index_source_layer_islands(layer, lookup);
    }

    std::vector<std::pair<coord_t, float>> layer_areas;
    size_t layer_group_count = 0;
    for (const Printing::PrintingGroup &group : plan.groups)
        layer_group_count += group.layers.size();
    layer_areas.reserve(layer_group_count);

    for (const Printing::PrintingGroup &group : plan.groups)
        for (const Printing::PrintingLayerGroup &layer_group : group.layers) {
            std::unordered_set<const LayerSliceIsland *> seen_islands;
            std::vector<const LayerSliceIsland *> ordered_islands;
            for (const Printing::PrintingToolGroup &tool_group : layer_group.tool_groups)
                for (const Printing::PrintingExtrusion &extrusion : tool_group.extrusions) {
                    if (extrusion.region_island == nullptr)
                        continue;
                    const RegionIslandLookup::const_iterator found =
                        lookup.find(extrusion.region_island);
                    if (found != lookup.end() && seen_islands.insert(found->second).second)
                        ordered_islands.push_back(found->second);
                }

            // Note: we disregard the intersection(island.slice(), regions.slices()) 
            //  because in a pritning plan, an island is always entirely printed at the same layer or collsions occurs.
            double layer_area = 0.0;
            for (const LayerSliceIsland *island : ordered_islands)
                layer_area += island->get_slice().area();
            layer_area = unscaled(unscaled(layer_area));
            layer_areas.emplace_back(layer_group.print_z,
                                     static_cast<float>(layer_area));
        }

    print.print_statistics()._layer_area_stats = std::move(layer_areas);
}

} // namespace

GCodeReaderResultBuilder::GCodeReaderResultBuilder(
    Print *print,
    const Printing::PrintingPlan *plan,
    std::string reader_plugin_id,
    const slic3r_api::GCodeStateTracker &initial_state)
    : m_previous_state(initial_state)
{
    initialize_result(print, plan, reader_plugin_id);
    if (const std::optional<double> object =
            m_previous_state.value(RAW_GCODE_READER_ITEM_OBJECT))
        process_object_change(static_cast<uint32_t>(*object));
}

// Borrow a matching Print/plan pair or two null pointers, copy viewer metadata
// and SEMM mode, and replace local source-island area statistics. No pointers
// are retained; mismatched inputs/catalogs throw and allocation failures propagate.
void GCodeReaderResultBuilder::initialize_result(
    Print *print,
    const Printing::PrintingPlan *plan,
    const std::string &reader_plugin_id)
{
    if ((print == nullptr) != (plan == nullptr))
        throw std::invalid_argument(
            "G-code viewer metadata requires both a Print and its PrintingPlan.");
    m_result.reset();
    m_result.id = ++s_reader_result_id;
    m_result.reader_plugin_id = reader_plugin_id;
    m_result.filament_diameters.assign(1, DEFAULT_VIEWER_FILAMENT_DIAMETER);
    m_result.filament_densities.assign(1, DEFAULT_VIEWER_FILAMENT_DENSITY);
    m_result.filament_cost.assign(1, DEFAULT_VIEWER_FILAMENT_COST);
    if (plan == nullptr) {
        m_result.extruders_count = 1;
        // TODO: external-file metadata needs Reader events for producer,
        // extruder/filament RGB colors, object names, profile ids, filament
        // density/cost and print metadata. Numeric movement events cannot
        // reconstruct those text and indexed-array values yet.
        return;
    }

    const PrintConfig &config = print->config();
    m_single_extruder_multi_material = slic3r_api::ConstPrintingPlan(
        reinterpret_cast<const printing_plan_handle *>(plan))
        .uses_single_extruder_multi_material(config.single_extruder_multi_material.value);
    const size_t tool_count = std::max<size_t>(config.nozzle_diameter.size(), 1);
    m_result.extruders_count = tool_count;
    m_result.max_print_height = static_cast<float>(config.max_print_height.value);
    m_result.z_offset = static_cast<float>(config.z_offset.value);
    m_result.bed_shape = config.bed_shape.get_values();
    m_result.extruder_colors = config.extruder_colour.get_values();
    m_result.filament_colors = config.filament_colour.get_values();
    m_result.filament_diameters.resize(tool_count);
    m_result.filament_densities.resize(tool_count);
    m_result.filament_cost.resize(tool_count);
    for (size_t tool = 0; tool < tool_count; ++tool) {
        m_result.filament_diameters[tool] = config.filament_diameter.size() == 0 ?
            DEFAULT_VIEWER_FILAMENT_DIAMETER : static_cast<float>(
                config.filament_diameter.get_at(
                    std::min(tool, config.filament_diameter.size() - 1)));
        m_result.filament_densities[tool] = config.filament_density.size() == 0 ?
            DEFAULT_VIEWER_FILAMENT_DENSITY : static_cast<float>(
                config.filament_density.get_at(
                    std::min(tool, config.filament_density.size() - 1)));
        m_result.filament_cost[tool] = config.filament_cost.size() == 0 ?
            DEFAULT_VIEWER_FILAMENT_COST : static_cast<float>(
                config.filament_cost.get_at(
                    std::min(tool, config.filament_cost.size() - 1)));
    }

    const DynamicPrintConfig &full_config = print->full_print_config();
    const ConfigOptionString *print_id =
        full_config.option<ConfigOptionString>("print_settings_id");
    const ConfigOptionStrings *filament_ids =
        full_config.option<ConfigOptionStrings>("filament_settings_id");
    const ConfigOptionString *printer_id =
        full_config.option<ConfigOptionString>("printer_settings_id");
    if (print_id != nullptr)
        m_result.settings_ids.print = print_id->value;
    if (filament_ids != nullptr)
        m_result.settings_ids.filament = filament_ids->get_values();
    if (printer_id != nullptr)
        m_result.settings_ids.printer = printer_id->value;

    for (size_t index = 0; index < plan->object_instances.size(); ++index) {
        const Printing::PrintingObjectInstance &instance =
            plan->object_instances[index];
        // The vector position is the zero-based Reader identity. Movements
        // outside objects use MoveVertex::OBJECT_INSTANCE_ID_NONE instead of
        // occupying an artificial entry in this vector.
        if (instance.name.empty())
            throw std::invalid_argument(
                "The PrintingPlan contains an incomplete object-instance catalog.");
        m_result.object_names.push_back(instance.name);
    }
    initialize_layer_area_stats(*print, *plan);
    m_has_object_instance_catalog = true;
}

void GCodeReaderResultBuilder::consume_chunk(std::string_view chunk)
{
    if (m_finished)
        throw std::logic_error("Cannot append G-code bytes after finalizing viewer data.");
    if (chunk.size() > std::numeric_limits<uint64_t>::max() - m_source_size)
        throw std::overflow_error("G-code source exceeds the viewer byte index range.");
    if (m_result.lines_ends.empty())
        m_result.lines_ends.emplace_back();
    for (size_t index = 0; index < chunk.size(); ++index)
        if (chunk[index] == '\n')
            m_result.lines_ends.front().push_back(
                static_cast<size_t>(m_source_size + index + 1));
    m_source_size += static_cast<uint64_t>(chunk.size());
    m_source_ends_with_newline = chunk.empty() ? m_source_ends_with_newline : chunk.back() == '\n';
}

void GCodeReaderResultBuilder::consume_event(
    const raw_gcode_reader_event &event,
    const slic3r_api::GCodeStateTracker &post_state)
{
    if (m_finished)
        throw std::logic_error("Cannot append a Reader event after finalizing viewer data.");
    invalidate_derived_state(event, post_state);
    const std::optional<double> previous_object_value = m_previous_state.value(
        RAW_GCODE_READER_ITEM_OBJECT);
    const std::optional<double> current_object_value = post_state.value(
        RAW_GCODE_READER_ITEM_OBJECT);
    const std::optional<uint32_t> previous_object = previous_object_value ?
        std::optional<uint32_t>(static_cast<uint32_t>(*previous_object_value)) :
        std::optional<uint32_t>();
    const std::optional<uint32_t> current_object = current_object_value ?
        std::optional<uint32_t>(static_cast<uint32_t>(*current_object_value)) :
        std::optional<uint32_t>();
    if (current_object && current_object != previous_object)
        process_object_change(*current_object);
    switch (event.kind) {
    case RAW_GCODE_READER_EVENT_STATE_UPDATE:
        process_state_update(event, post_state);
        break;
    case RAW_GCODE_READER_EVENT_LINEAR_MOVE:
        process_linear_move(event, post_state);
        break;
    case RAW_GCODE_READER_EVENT_ARC_MOVE:
        process_arc_move(event, post_state);
        break;
    case RAW_GCODE_READER_EVENT_SET_POSITION:
    case RAW_GCODE_READER_EVENT_SET_ORIGIN:
        process_position_change(event, post_state);
        break;
    case RAW_GCODE_READER_EVENT_SET_TEMPERATURE:
    case RAW_GCODE_READER_EVENT_WAIT_TEMPERATURE:
        process_temperature(event, post_state);
        break;
    case RAW_GCODE_READER_EVENT_MARKER:
        process_marker(event, post_state);
        break;
    case RAW_GCODE_READER_EVENT_DIAGNOSTIC:
        process_diagnostic(event, post_state);
        break;
    case RAW_GCODE_READER_EVENT_INVALIDATE:
        process_invalidation(event, post_state);
        break;
    default:
        throw std::invalid_argument("The viewer received an unknown Reader event kind.");
    }
    m_previous_state = post_state;
}

// Store any MAX_LAYER_HEIGHT delta for its current layer, then append a tool transition
// when the borrowed complete states differ. Result vectors own the copies; allocation
// failures and invalid tool IDs propagate to the synchronous Reader caller.
void GCodeReaderResultBuilder::process_state_update(
    const raw_gcode_reader_event &event,
    const slic3r_api::GCodeStateTracker &state)
{
    process_max_layer_height(event, state);
    const std::optional<uint32_t> previous_tool = m_previous_state.active_tool();
    const std::optional<uint32_t> current_tool = state.active_tool();
    if (!previous_tool || !current_tool || *previous_tool == *current_tool)
        return;
    process_tool_change(event, state, *current_tool);
}

void GCodeReaderResultBuilder::process_tool_change(
    const raw_gcode_reader_event &event,
    const slic3r_api::GCodeStateTracker &state,
    uint32_t new_tool)
{
    ensure_tool(new_tool);
    const extruder_index_t extruder_id = checked_extruder_index(new_tool);

    const Vec3d position(
        state_value_or(state, RAW_GCODE_READER_ITEM_X, 0.0),
        state_value_or(state, RAW_GCODE_READER_ITEM_Y, 0.0),
        state_value_or(state, RAW_GCODE_READER_ITEM_Z, 0.0));
    const uint32_t line = static_cast<uint32_t>(
        event_value(event, RAW_GCODE_READER_ITEM_LINE_NUMBER).value_or(0.0));
    const double offset_x = state_value_or(
        state, RAW_GCODE_READER_ITEM_TOOL_OFFSET_X, 0.0, new_tool);
    const double offset_y = state_value_or(
        state, RAW_GCODE_READER_ITEM_TOOL_OFFSET_Y, 0.0, new_tool);
    const double offset_z = state_value_or(
        state, RAW_GCODE_READER_ITEM_TOOL_OFFSET_Z, 0.0, new_tool);
    const uint32_t fan_id = static_cast<uint32_t>(state_value_or(
        state, RAW_GCODE_READER_ITEM_TOOL_FAN, 0.0, new_tool));
    m_result.moves.emplace_back(
        line + 1u, EMoveType::Tool_change, viewer_role(state),
        extruder_id,
        static_cast<uint8_t>(std::min<uint32_t>(m_current_color, UINT8_MAX)),
        viewer_object_id(state),
        Vec3f(static_cast<float>(position.x() + offset_x),
              static_cast<float>(position.y() + offset_y),
              static_cast<float>(position.z() + offset_z - m_result.z_offset)),
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        static_cast<float>(state.value(
            RAW_GCODE_READER_ITEM_FAN_SPEED, fan_id).value_or(0.0)),
        static_cast<float>(state.value(
            RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE, new_tool).value_or(0.0)),
        m_total_time, 0, false);
}

void GCodeReaderResultBuilder::process_object_change(
    const uint32_t object_instance_id)
{
    if (object_instance_id != UINT32_MAX && object_instance_id >= UINT16_MAX)
        throw std::out_of_range(
            "A Reader object instance id cannot be represented by MoveVertex.");
    if (object_instance_id == UINT32_MAX)
        return;

    const uint32_t viewer_object_id = object_instance_id;

    if (m_has_object_instance_catalog) {
        if (viewer_object_id >= m_result.object_names.size())
            throw std::out_of_range(
                "A Reader object instance id is absent from the Print catalog.");
        return;
    }

    while (m_result.object_names.size() <= viewer_object_id) {
        const size_t id = m_result.object_names.size();
        m_result.object_names.push_back("Object " + std::to_string(id));
    }
}

void GCodeReaderResultBuilder::process_linear_move(
    const raw_gcode_reader_event &event,
    const slic3r_api::GCodeStateTracker &state)
{
    if (!m_previous_state.x() || !m_previous_state.y() || !m_previous_state.z() ||
        !state.x() || !state.y() || !state.z()) {
        // TODO: add a Reader discontinuity event carrying a trusted restart
        // position. After an unknown command, partial coordinates cannot be
        // turned into an honest viewer segment.
        return;
    }
    const std::optional<uint32_t> active_tool = state.active_tool();
    const uint32_t tool = active_tool.value_or(0);
    const Vec3d start(*m_previous_state.x(), *m_previous_state.y(), *m_previous_state.z());
    const Vec3d end(*state.x(), *state.y(), *state.z());
    const double start_e = state_value_or(
        m_previous_state, RAW_GCODE_READER_ITEM_E, 0.0, tool);
    const double end_e = state_value_or(state, RAW_GCODE_READER_ITEM_E, start_e, tool);
    append_move(event, state, start, end, start_e, end_e, false);
}

void GCodeReaderResultBuilder::process_arc_move(
    const raw_gcode_reader_event &event,
    const slic3r_api::GCodeStateTracker &state)
{
    if (!m_previous_state.x() || !m_previous_state.y() || !m_previous_state.z() ||
        !state.x() || !state.y() || !state.z())
        return;

    const Vec3d start(*m_previous_state.x(), *m_previous_state.y(), *m_previous_state.z());
    const Vec3d end(*state.x(), *state.y(), *state.z());
    const std::optional<double> center_x =
        event_value(event, RAW_GCODE_READER_ITEM_ARC_CENTER_X);
    const std::optional<double> center_y =
        event_value(event, RAW_GCODE_READER_ITEM_ARC_CENTER_Y);
    const std::optional<double> plane =
        event_value(event, RAW_GCODE_READER_ITEM_ARC_PLANE);
    const uint32_t tool = state.active_tool().value_or(0);
    const double start_e = state_value_or(
        m_previous_state, RAW_GCODE_READER_ITEM_E, 0.0, tool);
    const double end_e = state_value_or(state, RAW_GCODE_READER_ITEM_E, start_e, tool);

    if (!center_x || !center_y || (plane && *plane != 0.0)) {
        // TODO: add an ARC_RADIUS item and documented plane values. R arcs and
        // non-XY planes cannot be reconstructed from the current event, so the
        // endpoint is retained as one conservative straight viewer segment.
        append_move(event, state, start, end, start_e, end_e, false);
        return;
    }

    const Vec2d center(start.x() + *center_x, start.y() + *center_y);
    const Vec2d start_radius = start.head<2>() - center;
    const Vec2d end_radius = end.head<2>() - center;
    const double radius = start_radius.norm();
    if (!(radius > 0.0) || !std::isfinite(radius)) {
        append_move(event, state, start, end, start_e, end_e, false);
        return;
    }
    double angle = std::atan2(start_radius.x() * end_radius.y() - start_radius.y() * end_radius.x(),
                              start_radius.dot(end_radius));
    const bool full_circle = (start.head<2>() - end.head<2>()).norm() < EPSILON;
    const bool clockwise = event_value(event, RAW_GCODE_READER_ITEM_ARC_CLOCKWISE).value_or(0.0) != 0.0;
    const uint32_t turns = std::max<uint32_t>(1u, static_cast<uint32_t>(
        event_value(event, RAW_GCODE_READER_ITEM_ARC_TURNS).value_or(1.0)));
    if (full_circle)
        angle = (clockwise ? -2.0 : 2.0) * PI * turns;
    else if (clockwise && angle >= 0.0)
        angle -= 2.0 * PI;
    else if (!clockwise && angle <= 0.0)
        angle += 2.0 * PI;
    if (!full_circle && turns > 1)
        angle += (clockwise ? -2.0 : 2.0) * PI * (turns - 1);
    const size_t segments = std::max<size_t>(
        static_cast<size_t>(std::ceil(std::abs(angle) * radius / MM_PER_ARC_SEGMENT)), 1);
    Vec3d previous = start;
    double previous_e = start_e;
    for (size_t segment = 1; segment <= segments; ++segment) {
        const double ratio = double(segment) / double(segments);
        const double segment_angle = angle * ratio;
        const double cosine = std::cos(segment_angle);
        const double sine = std::sin(segment_angle);
        Vec3d point;
        point.x() = center.x() + start_radius.x() * cosine - start_radius.y() * sine;
        point.y() = center.y() + start_radius.x() * sine + start_radius.y() * cosine;
        point.z() = start.z() + (end.z() - start.z()) * ratio;
        if (segment == segments)
            point = end;
        const double next_e = start_e + (end_e - start_e) * ratio;
        append_move(event, state, previous, point, previous_e, next_e,
                    segment != segments);
        previous = point;
        previous_e = next_e;
    }
}

void GCodeReaderResultBuilder::process_position_change(
    const raw_gcode_reader_event &,
    const slic3r_api::GCodeStateTracker &)
{
    // G92 and origin changes alter the coordinate model but do not move the
    // nozzle. m_previous_state is replaced by consume_event() after return.
}

void GCodeReaderResultBuilder::process_temperature(
    const raw_gcode_reader_event &,
    const slic3r_api::GCodeStateTracker &)
{
    // Temperatures are sampled from the complete state when a movement vertex
    // is stored. A target-only command has no geometry of its own.
}

// Convert a validated marker event into one owned viewer move, using its layer tag
// before the last HEIGHT. Event and state are borrowed; unknown markers or invalid
// tools throw, and result-vector allocation failures propagate.
void GCodeReaderResultBuilder::process_marker(
    const raw_gcode_reader_event &event,
    const slic3r_api::GCodeStateTracker &state)
{
    const uint32_t marker = static_cast<uint32_t>(
        event_value(event, RAW_GCODE_READER_ITEM_MARKER).value_or(0.0));
    EMoveType type = EMoveType::Noop;
    switch (marker) {
    case RAW_GCODE_READER_MARKER_SEAM:         type = EMoveType::Seam; break;
    case RAW_GCODE_READER_MARKER_COLOR_CHANGE: type = EMoveType::Color_change; break;
    case RAW_GCODE_READER_MARKER_PAUSE_PRINT:  type = EMoveType::Pause_Print; break;
    case RAW_GCODE_READER_MARKER_CUSTOM_GCODE: type = EMoveType::Custom_GCode; break;
    default:
        throw std::invalid_argument("The viewer received an unknown Reader marker type.");
    }

    const uint32_t tool = state.active_tool().value_or(0);
    ensure_tool(tool);
    const extruder_index_t extruder_id = checked_extruder_index(tool);
    if (type == EMoveType::Color_change)
        m_current_color = std::max<uint32_t>(m_current_color + 1u,
            static_cast<uint32_t>(std::min<size_t>(m_result.extruders_count,
                                                   UINT32_MAX)));

    const uint32_t line = static_cast<uint32_t>(
        event_value(event, RAW_GCODE_READER_ITEM_LINE_NUMBER).value_or(0.0));
    const uint32_t gcode_id = type == EMoveType::Seam && !m_result.moves.empty() ?
        m_result.moves.back().gcode_id : line + 1u;
    const double offset_x = state_value_or(
        state, RAW_GCODE_READER_ITEM_TOOL_OFFSET_X, 0.0, tool);
    const double offset_y = state_value_or(
        state, RAW_GCODE_READER_ITEM_TOOL_OFFSET_Y, 0.0, tool);
    const double offset_z = state_value_or(
        state, RAW_GCODE_READER_ITEM_TOOL_OFFSET_Z, 0.0, tool);
    const uint32_t fan_id = static_cast<uint32_t>(state_value_or(
        state, RAW_GCODE_READER_ITEM_TOOL_FAN, 0.0, tool));

    //w idht & height of the marker is used (*1.5) to show the marker in the gcodeviewer.
    // The height is the MAX_LAYER_HEIGHT tag of the marker's layer when it has one,
    // else the last HEIGHT read.
    const double width = state.value(RAW_GCODE_READER_ITEM_WIDTH).value_or(0.0);
    const double height = state.value(RAW_GCODE_READER_ITEM_HEIGHT).value_or(0.0);
    const double marker_width = state.value(RAW_GCODE_READER_ITEM_WIDTH).value_or(width);
    const double marker_height = tagged_layer_height(viewer_layer_id(state)).value_or(
        state.value(RAW_GCODE_READER_ITEM_HEIGHT).value_or(height));

    // Markers have no geometric delta. They sample the current machine state
    // so the viewer can place and color the point without parsing G-code text.
    m_result.moves.emplace_back(
        gcode_id, // uint32_t gcode_id
        type, // EMoveType type
        viewer_role(state), // GCodeExtrusionRole extrusion_role
        extruder_id, // extruder_index_t extruder_id
        static_cast<uint8_t>(std::min<uint32_t>(m_current_color, UINT8_MAX)), // uint8_t cp_color_id
        viewer_object_id(state), // uint16_t object_id
        Vec3f(static_cast<float>(state_value_or(state, RAW_GCODE_READER_ITEM_X, 0.0) + offset_x),
              static_cast<float>(state_value_or(state, RAW_GCODE_READER_ITEM_Y, 0.0) + offset_y),
              static_cast<float>(state_value_or(state, RAW_GCODE_READER_ITEM_Z, 0.0) + offset_z - m_result.z_offset)), // Vec3f position
        0.0f, // float delta_extruder
        0.0f, // float feedrate
        float(marker_width), // float width
        float(marker_height), // float height
        0.0f, // float mm3_per_mm
        static_cast<float>(state.value(
            RAW_GCODE_READER_ITEM_FAN_SPEED, fan_id).value_or(0.0)), // float fan_speed
        static_cast<float>(state.value(
            RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE, tool).value_or(0.0)), // float temperature
        m_total_time, // float move_time
        static_cast<uint16_t>(viewer_layer_id(state)), // uint16_t layer_id
        false // bool internal_only
    );
}

void GCodeReaderResultBuilder::process_diagnostic(
    const raw_gcode_reader_event &event,
    const slic3r_api::GCodeStateTracker &)
{
    const std::optional<double> raw_type = event_value(
        event, RAW_GCODE_READER_ITEM_DIAGNOSTIC_TYPE);
    const std::optional<double> raw_line = event_value(
        event, RAW_GCODE_READER_ITEM_LINE_NUMBER);
    if (!raw_type || !raw_line)
        throw std::invalid_argument("The viewer received an incomplete Reader diagnostic.");

    const raw_gcode_reader_diagnostic_type type =
        static_cast<raw_gcode_reader_diagnostic_type>(uint32_t(*raw_type));
    std::string message;
    switch (type) {
    case RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_COMMAND:
        message = _u8L("The G-code Reader encountered an unknown command.");
        break;
    case RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS:
        message = _u8L("The G-code Reader encountered invalid command parameters.");
        break;
    case RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_TOOL:
        message = _u8L("The G-code Reader could not identify the requested tool.");
        break;
    case RAW_GCODE_READER_DIAGNOSTIC_AMBIGUOUS_TOOL:
        message = _u8L("The G-code Reader found several tools matching the requested tool.");
        break;
    case RAW_GCODE_READER_DIAGNOSTIC_UNSUPPORTED_STATE_EFFECT:
        message = _u8L("The G-code Reader recognized a command whose machine-state effect is not supported.");
        break;
    default:
        throw std::invalid_argument("The viewer received an unknown Reader diagnostic type.");
    }

    // Diagnostics are historical observations, not modal state. Preserve
    // every occurrence in stream order so the viewer can associate it with
    // the source line even when multiple lines report the same condition.
    m_result.reader_diagnostics.push_back({
        type, static_cast<uint32_t>(*raw_line),
        PrintStateBase::WarningLevel::NON_CRITICAL, std::move(message)});
}

void GCodeReaderResultBuilder::process_invalidation(
    const raw_gcode_reader_event &,
    const slic3r_api::GCodeStateTracker &)
{
    // Invalidation already removed untrusted values from post_state. The next
    // movement listener will resume only when enough position data is known.
}

// Inspect borrowed event/post-state without retaining either. Clear debt when E
// changes with no active tool; otherwise clear a tool's debt on unknown E, using
// the active tool's E for shared SEMM debt. Other retained debts are unchanged.
void GCodeReaderResultBuilder::invalidate_derived_state(
    const raw_gcode_reader_event &event,
    const slic3r_api::GCodeStateTracker &state)
{
    bool e_changed = false;
    for (uint32_t index = 0; index < event.item_count; ++index) {
        const raw_gcode_reader_event_item &item = event.items[index];
        if (item.type == RAW_GCODE_READER_ITEM_E &&
            m_previous_state.e(item.target) != state.e(item.target)) {
            e_changed = true;
            break;
        }
    }

    // E positions are stored per logical tool, so merely losing ACTIVE_TOOL
    // does not invalidate them. If E changes while no tool is active, however,
    // the host cannot safely associate that change with any retained debt.
    if (!state.active_tool() && e_changed) {
        std::fill(m_retracted_filament.begin(), m_retracted_filament.end(), 0.0);
        return;
    }

    // Shared physical debt follows the active logical tool's E-known state.
    // Losing ACTIVE_TOOL alone preserves debt until a tool is known again,
    // unless an E change already made attribution unreliable above.
    if (m_single_extruder_multi_material) {
        const std::optional<uint32_t> active_tool = state.active_tool();
        if (active_tool && !state.e(*active_tool))
            std::fill(m_retracted_filament.begin(), m_retracted_filament.end(), 0.0);
        return;
    }

    // A Reader may invalidate one E slot while preserving other tools. Keep
    // independent debts only where the corresponding E position remains known.
    for (size_t tool = 0; tool < m_retracted_filament.size(); ++tool)
        if (!state.e(static_cast<uint32_t>(tool)))
            m_retracted_filament[tool] = 0.0;
}

// Borrow a validated event/state and endpoints, classify their delta, append an owned
// viewer move, account time/material, and raise a layer maximum from known extrusion
// HEIGHT values. Positive E repays shared SEMM or per-tool debt before consumption;
// no-op deltas change nothing. Inputs are not retained; invalid IDs throw and allocations may fail.
void GCodeReaderResultBuilder::append_move(
    const raw_gcode_reader_event &event,
    const slic3r_api::GCodeStateTracker &state,
    const Vec3d &start,
    const Vec3d &end,
    double start_e,
    double end_e,
    bool internal_only)
{
    const std::optional<uint32_t> active_tool = state.active_tool();
    const uint32_t tool = active_tool.value_or(0);
    ensure_tool(tool);
    const size_t debt_slot = m_single_extruder_multi_material ? 0 : tool;
    const extruder_index_t extruder_id = checked_extruder_index(tool);
    const Vec3d delta = end - start;
    const double xyz_distance = delta.norm();
    const bool xyz_changed = xyz_distance > EPSILON;
    const bool volumetric = state.value(RAW_GCODE_READER_ITEM_VOLUMETRIC).value_or(0.0) != 0.0;
    const double diameter = vector_value_or(
        m_result.filament_diameters, tool, DEFAULT_VIEWER_FILAMENT_DIAMETER);
    const double filament_area = PI * std::pow(0.5 * diameter, 2.0);
    // Keep the spatial move visible when ACTIVE_TOOL is unknown, but do not
    // attribute its E delta to the fallback display tool. A later known E
    // position starts a new trustworthy accounting sequence.
    const double e_delta_raw = active_tool ? end_e - start_e : 0.0;
    const double e_delta = volumetric && filament_area > 0.0 ?
        e_delta_raw / filament_area : e_delta_raw;
    const double extrusion_factor =
        state.value(RAW_GCODE_READER_ITEM_EXTRUSION_FACTOR).value_or(100.0) * 0.01;
    const GCodeExtrusionRole role = viewer_role(state);
    const uint32_t raw_role = static_cast<uint32_t>(
        state.value(RAW_GCODE_READER_ITEM_EXTRUSION_ROLE).value_or(0.0));

    // Reader role/object markers drive classification and accounting; missing
    // geometry metadata uses the neutral/default viewer values below.

    // Positive E first restores any preceding retraction. Only the remainder
    // deposits material and may classify an XY move as an extrusion. Compute
    // it without changing the debt yet: a sub-precision no-op must not alter
    // the accounting state when no viewer movement is emitted.
    double restored_filament = 0.0;
    double consumed_filament = 0.0;
    if (e_delta > 0.0) {
        restored_filament = std::min(e_delta, m_retracted_filament[debt_slot]);
        consumed_filament = e_delta - restored_filament;
    }

    EMoveType type = EMoveType::Noop;
    if ((raw_role & RAW_EXTRUSION_ROLE_WIPE) != 0)
        type = EMoveType::Wipe;
    else if ((raw_role & RAW_EXTRUSION_ROLE_RETRACT) != 0)
        type = EMoveType::Retract;
    else if ((raw_role & RAW_EXTRUSION_ROLE_UNRETRACT) != 0)
        type = EMoveType::Unretract;
    else if (e_delta < -EPSILON)
        type = xyz_changed ? EMoveType::Travel : EMoveType::Retract;
    else if (e_delta > EPSILON)
        if (consumed_filament > 0.0)
            type = xyz_changed ? EMoveType::Extrude : EMoveType::Unretract;
        else
            type = xyz_changed ? EMoveType::Travel : EMoveType::Unretract;
    else if (xyz_changed)
        type = EMoveType::Travel;
    if (type == EMoveType::Noop)
        return;

    if (e_delta < 0.0)
        m_retracted_filament[debt_slot] += -e_delta;
    else if (e_delta > 0.0) {
        m_retracted_filament[debt_slot] -= restored_filament;
    }
    const double volume = consumed_filament * filament_area * extrusion_factor;

    const double feedrate_mm_min =
        state.value(RAW_GCODE_READER_ITEM_FEEDRATE).value_or(0.0);
    const double speed_factor =
        state.value(RAW_GCODE_READER_ITEM_SPEED_FACTOR).value_or(100.0) * 0.01;
    const float feedrate = static_cast<float>(feedrate_mm_min * speed_factor / 60.0);
    const double movement_distance = xyz_changed ? xyz_distance : std::abs(e_delta);
    const float elapsed = feedrate > 0.0f ?
        static_cast<float>(movement_distance / feedrate) : 0.0f;
    m_total_time += elapsed;

    const double mm3_per_mm = type == EMoveType::Extrude && xyz_distance > EPSILON ?
        volume / xyz_distance : 0.0;
    float height = static_cast<float>(
        state.value(RAW_GCODE_READER_ITEM_HEIGHT).value_or(DEFAULT_VIEWER_TOOLPATH_HEIGHT));
    float width = static_cast<float>(state.value(RAW_GCODE_READER_ITEM_WIDTH).value_or(0.0));
    if (type == EMoveType::Extrude && width <= 0.0f && height > 0.0f)
        width = static_cast<float>(mm3_per_mm / height + height * (1.0 - 0.25 * PI));

    const double offset_x = state_value_or(state, RAW_GCODE_READER_ITEM_TOOL_OFFSET_X, 0.0, tool);
    const double offset_y = state_value_or(state, RAW_GCODE_READER_ITEM_TOOL_OFFSET_Y, 0.0, tool);
    const double offset_z = state_value_or(state, RAW_GCODE_READER_ITEM_TOOL_OFFSET_Z, 0.0, tool);
    const uint32_t line = static_cast<uint32_t>(
        event_value(event, RAW_GCODE_READER_ITEM_LINE_NUMBER).value_or(0.0));
    const uint16_t layer = static_cast<uint16_t>(viewer_layer_id(state));
    const uint32_t fan_id = static_cast<uint32_t>(
        state.value(RAW_GCODE_READER_ITEM_TOOL_FAN, tool).value_or(0.0));
    const double fan = state.value(RAW_GCODE_READER_ITEM_FAN_SPEED, fan_id).value_or(0.0);
    const double temperature =
        state.value(RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE, tool).value_or(0.0);

    m_result.moves.emplace_back(
        line + 1u, type, role,
        extruder_id,
        static_cast<uint8_t>(std::min<uint32_t>(m_current_color, UINT8_MAX)),
        viewer_object_id(state),
        Vec3f(static_cast<float>(end.x() + offset_x),
              static_cast<float>(end.y() + offset_y),
              static_cast<float>(end.z() + offset_z - m_result.z_offset)),
        static_cast<float>(e_delta), feedrate, width, height,
        static_cast<float>(mm3_per_mm), static_cast<float>(fan),
        static_cast<float>(temperature), m_total_time,
        layer, internal_only);

    // Only extrusions whose HEIGHT is known contribute to the layer height.
    if (type == EMoveType::Extrude)
        if (const std::optional<double> extrusion_height = state.value(RAW_GCODE_READER_ITEM_HEIGHT))
            raise_layer_max_height(layer, *extrusion_height);

    m_move_times[static_cast<size_t>(type)] += elapsed;
    m_role_times[static_cast<size_t>(role)] += elapsed;
    if (layer >= m_layer_times.size())
        m_layer_times.resize(static_cast<size_t>(layer) + 1, 0.0f);
    m_layer_times[layer] += elapsed;
    if (type == EMoveType::Travel)
        m_result.print_statistics.modes[0].travel_time += elapsed;

    if (volume > 0.0) {
        m_result.print_statistics.volumes_per_extruder[extruder_id] += volume;
        m_result.print_statistics.volumes_per_role_per_extruder[role][extruder_id] += volume;
        const uint16_t object = viewer_object_id(state);
        if (object != GCodeProcessorResult::OBJECT_INSTANCE_ID_NONE)
            m_result.print_statistics.volumes_per_role_per_extruder_per_object[object][extruder_id][role] += volume;
        const double length_m = filament_area > 0.0 ? volume / filament_area * 0.001 : 0.0;
        const double density = vector_value_or(
            m_result.filament_densities, tool, DEFAULT_VIEWER_FILAMENT_DENSITY);
        const double mass_g = volume * density * 0.001;
        std::pair<double, double> &role_filament =
            m_result.print_statistics.used_filaments_per_role[role];
        role_filament.first += length_m;
        role_filament.second += mass_g;
        const double cost_per_kg = vector_value_or(
            m_result.filament_cost, tool, DEFAULT_VIEWER_FILAMENT_COST);
        m_result.print_statistics.cost_per_extruder[extruder_id] += mass_g * cost_per_kg * 0.001;
        if (m_color_volumes.size() <= m_current_color)
            m_color_volumes.resize(static_cast<size_t>(m_current_color) + 1, 0.0);
        m_color_volumes[m_current_color] += volume;
    }
}

// Read a validated event delta and store its MAX_LAYER_HEIGHT under the borrowed
// post-state's current layer. The result owns vector values; event/state are not
// retained, later layers use separate slots, and vector allocation failures propagate.
void GCodeReaderResultBuilder::process_max_layer_height(
    const raw_gcode_reader_event &event,
    const slic3r_api::GCodeStateTracker &state)
{
    const std::optional<double> tag = event_value(event, RAW_GCODE_READER_ITEM_MAX_LAYER_HEIGHT);
    if (!tag)
        return;
    const size_t layer = viewer_layer_id(state);
    if (layer >= m_result.layers_max_height.size())
        m_result.layers_max_height.resize(layer + 1, 0.0f);
    if (layer >= m_layer_max_height_tagged.size())
        m_layer_max_height_tagged.resize(layer + 1, false);
    m_result.layers_max_height[layer] = static_cast<float>(*tag);
    m_layer_max_height_tagged[layer] = true;
}

// Raise one untagged layer's stored height to a known extrusion HEIGHT; the result
// owns the vector value, no external input is retained, and vector growth may throw.
void GCodeReaderResultBuilder::raise_layer_max_height(uint32_t layer, double extrusion_height)
{
    if (layer < m_layer_max_height_tagged.size() && m_layer_max_height_tagged[layer])
        return;
    if (layer >= m_result.layers_max_height.size())
        m_result.layers_max_height.resize(static_cast<size_t>(layer) + 1, 0.0f);
    m_result.layers_max_height[layer] = std::max(
        m_result.layers_max_height[layer], static_cast<float>(extrusion_height));
}

// Return a layer's own MAX_LAYER_HEIGHT value or nullopt when no tag was consumed;
// this reads result-owned vectors without mutation or allocation.
std::optional<double> GCodeReaderResultBuilder::tagged_layer_height(uint32_t layer) const
{
    if (layer < m_layer_max_height_tagged.size() && m_layer_max_height_tagged[layer])
        return static_cast<double>(m_result.layers_max_height[layer]);
    return std::nullopt;
}

// Ensure owned logical-tool metadata covers tool, with one debt slot for SEMM
// or one per tool otherwise. Reject IDs outside the signed extruder range;
// vector allocation failures propagate, and no external ownership is changed.
void GCodeReaderResultBuilder::ensure_tool(uint32_t tool)
{
    (void)checked_extruder_index(tool);
    const size_t required = static_cast<size_t>(tool) + 1;
    const size_t debt_slots = m_single_extruder_multi_material ? 1 : required;
    if (m_retracted_filament.size() < debt_slots)
        m_retracted_filament.resize(debt_slots, 0.0);
    if (m_result.extruders_count >= required)
        return;

    const float diameter = vector_value_or(
        m_result.filament_diameters, tool, DEFAULT_VIEWER_FILAMENT_DIAMETER);
    const float density = vector_value_or(
        m_result.filament_densities, tool, DEFAULT_VIEWER_FILAMENT_DENSITY);
    const float cost = vector_value_or(
        m_result.filament_cost, tool, DEFAULT_VIEWER_FILAMENT_COST);
    m_result.filament_diameters.resize(required, diameter);
    m_result.filament_densities.resize(required, density);
    m_result.filament_cost.resize(required, cost);
    m_result.extruders_count = required;
}

void GCodeReaderResultBuilder::finalize_statistics()
{
    PrintEstimatedStatistics::Mode &normal = m_result.print_statistics.modes[0];
    normal.time = m_total_time;
    normal.layers_times = m_layer_times;
    for (size_t index = 0; index < m_move_times.size(); ++index)
        if (m_move_times[index] > 0.0f)
            normal.moves_times.emplace_back(static_cast<EMoveType>(index), m_move_times[index]);
    for (size_t index = 0; index < m_role_times.size(); ++index)
        if (m_role_times[index] > 0.0f)
            normal.roles_times.emplace_back(
                static_cast<GCodeExtrusionRole>(index), m_role_times[index]);
    m_result.print_statistics.volumes_per_color_change = m_color_volumes;

    // Keep the viewer's per-object statistics shape aligned with the Reader
    // catalog even when an object has no extrusion volume.
    for (size_t object_id = 0; object_id < m_result.object_names.size(); ++object_id)
        (void)m_result.print_statistics.volumes_per_role_per_extruder_per_object[object_id];

    // TODO: exact planner time needs portable machine-limit, jerk and command
    // delay events. Until then the host reports nominal path time and leaves
    // Stealth empty instead of presenting a false acceleration simulation.
    m_result.print_statistics.modes[
        static_cast<size_t>(PrintEstimatedStatistics::ETimeMode::Stealth)].reset();
}

GCodeProcessorResult GCodeReaderResultBuilder::finish(
    const slic3r_api::GCodeStateTracker &final_state)
{
    if (m_finished)
        throw std::logic_error("G-code viewer data was finalized more than once.");
    if (final_state.reader_id() != m_previous_state.reader_id())
        throw std::invalid_argument("The Reader final state changed schema identity.");
    if (m_result.lines_ends.empty())
        m_result.lines_ends.emplace_back();
    if (m_source_size != 0 && !m_source_ends_with_newline)
        m_result.lines_ends.front().push_back(static_cast<size_t>(m_source_size));
    finalize_statistics();
    m_finished = true;
    return std::move(m_result);
}

} // namespace Slic3r
