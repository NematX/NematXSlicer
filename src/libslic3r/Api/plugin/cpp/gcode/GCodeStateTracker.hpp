///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_gcode_GCodeStateTracker_hpp_
#define slic3r_Api_plugin_cpp_gcode_GCodeStateTracker_hpp_

#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_gcode_reader.h"

/*
G-code reader state tracker
===========================

This value type stores every currently known scalar G-code state item. Reader
providers apply normalized action events to it while parsing, and temporarily
expose its complete raw state to the host at the end of one synchronous run.

The map is authoritative. The raw view is rebuilt after every mutation and
after every copy or move, so its item pointer always targets storage owned by
the same tracker. Missing keys remain distinct from legitimate zero values.

    raw state/event
          |
          v  validate completely
    map<(type,target), value>       authoritative owned representation
          |
          v  rebuild_state()
    vector<raw item> + raw state    borrowed C view for synchronous callbacks

Mutation is transactional: validation and updates happen in temporaries before
the authoritative map changes. A rejected state or event therefore leaves the
previous tracker usable.
*/

namespace slic3r_api {

class GCodeStateTracker
{
public:
    // A standalone tracker starts on physical line zero. Reader invocations
    // normally use the state constructor so the host controls initial values.
    GCodeStateTracker()
    {
        set_line_number(0);
        set_reader_id(GCODE_READER_STATE_ID_PORTABLE);
    }

    // Copy and validate a borrowed complete state. No pointer from the source
    // is retained after this constructor returns.
    explicit GCodeStateTracker(const raw_gcode_reader_state &initial_state)
    {
        replace(initial_state);
    }

    // Copy only the authoritative values, then point the new raw view at the
    // new object's vector. Copying m_state directly would retain other's data().
    GCodeStateTracker(const GCodeStateTracker &other) : m_values(other.m_values)
    {
        rebuild_state();
    }

    GCodeStateTracker &operator=(const GCodeStateTracker &other)
    {
        if (this != &other) {
            m_values = other.m_values;
            rebuild_state();
        }
        return *this;
    }

    // Rebuild both raw views after moving because a vector move does not make a
    // copied raw pointer safe. The moved-from tracker is safe to destroy or
    // assign, but no longer represents a complete Reader state.
    GCodeStateTracker(GCodeStateTracker &&other) noexcept : m_values(std::move(other.m_values))
    {
        rebuild_state();
        other.rebuild_state();
    }

    GCodeStateTracker &operator=(GCodeStateTracker &&other) noexcept
    {
        if (this != &other) {
            m_values = std::move(other.m_values);
            rebuild_state();
            other.rebuild_state();
        }
        return *this;
    }

    // Replace every known value from a complete state. Validation finishes
    // before mutation, so a rejected input leaves the previous state intact.
    void replace(const raw_gcode_reader_state &state)
    {
        validate_items(state.items, state.item_count);
        for (uint32_t index = 0; index < state.item_count; ++index) {
            if (state.items[index].type == RAW_GCODE_READER_ITEM_MARKER)
                throw std::invalid_argument("A complete G-code reader state contains an event-local marker.");
            if (state.items[index].type == RAW_GCODE_READER_ITEM_DIAGNOSTIC_TYPE)
                throw std::invalid_argument("A complete G-code reader state contains an event-local diagnostic.");
        }
        std::map<Key, double> replacement;
        copy_items(state.items, state.item_count, replacement);
        require_line_number(replacement);
        normalize_reader_id(replacement);
        m_values = std::move(replacement);
        rebuild_state();
    }

    // Apply one action delta. INVALIDATE removes each listed value except the
    // line number, while omitted values deliberately remain unchanged.
    void apply(const raw_gcode_reader_event &event)
    {
        validate_event(event);
        std::map<Key, double> updated = m_values;
        for (uint32_t index = 0; index < event.item_count; ++index) {
            const raw_gcode_reader_event_item &item = event.items[index];
            // Markers and diagnostics annotate one event. Retaining either
            // would make later machine snapshots repeat a past notification.
            if (item.type == RAW_GCODE_READER_ITEM_MARKER ||
                item.type == RAW_GCODE_READER_ITEM_DIAGNOSTIC_TYPE)
                continue;
            const Key key{item.type, item.target};
            if (event.kind == RAW_GCODE_READER_EVENT_INVALIDATE &&
                item.type != RAW_GCODE_READER_ITEM_LINE_NUMBER)
                updated.erase(key);
            else
                updated[key] = item.value;
        }

        // A changed temperature target invalidates the earlier guarantee that
        // execution waited for the previous target. A wait re-establishes it.
        if (event.kind == RAW_GCODE_READER_EVENT_SET_TEMPERATURE) {
            for (uint32_t index = 0; index < event.item_count; ++index) {
                const raw_gcode_reader_event_item &item = event.items[index];
                if (is_temperature_item(item.type))
                    updated.erase(Key{reached_temperature_item(item.type), item.target});
            }
        }
        if (event.kind == RAW_GCODE_READER_EVENT_WAIT_TEMPERATURE) {
            for (uint32_t index = 0; index < event.item_count; ++index) {
                const raw_gcode_reader_event_item &item = event.items[index];
                if (is_temperature_item(item.type))
                    updated[Key{reached_temperature_item(item.type), item.target}] = item.value;
            }
        }
        require_line_number(updated);
        normalize_reader_id(updated);
        m_values = std::move(updated);
        rebuild_state();
    }

    // Test presence without confusing an unknown value with a known zero.
    bool known(uint32_t type, uint32_t target = 0) const
    {
        return m_values.find(Key{type, target}) != m_values.end();
    }

    // Return one known scalar. The optional is empty when no command or initial
    // state established this (type, target) pair.
    std::optional<double> value(uint32_t type, uint32_t target = 0) const
    {
        const std::map<Key, double>::const_iterator found = m_values.find(Key{type, target});
        return found == m_values.end() ? std::optional<double>() : std::optional<double>(found->second);
    }

    // The returned C view is borrowed until this tracker changes, moves or is
    // destroyed. A callback consumer must copy it before returning.
    const raw_gcode_reader_state &state() const { return m_state; }

    // Set the mandatory physical, zero-based line cursor used by every event.
    void set_line_number(uint32_t value)
    {
        m_values[Key{RAW_GCODE_READER_ITEM_LINE_NUMBER, 0}] = double(value);
        rebuild_state();
    }

    // Store the identity which scopes provider-private items. Zero permanently
    // names the portable schema and therefore promises that no private key
    // requires a provider-specific interpretation.
    void set_reader_id(uint32_t value)
    {
        m_values[Key{RAW_GCODE_READER_ITEM_READER_ID, 0}] = double(value);
        rebuild_state();
    }

    uint32_t reader_id() const
    {
        const std::optional<uint32_t> id = uint32_value(RAW_GCODE_READER_ITEM_READER_ID);
        if (!id)
            throw std::logic_error("A G-code reader state has no Reader identity.");
        return *id;
    }

    // Provider-private state remains opaque to the tracker. These mutators are
    // intentionally restricted to the custom range and keep the raw callback
    // view synchronized with the authoritative map.
    void set_custom_value(uint32_t type, uint32_t target, double value)
    {
        const raw_gcode_reader_event_item item{type, target, value};
        validate_item(item);
        if (type < RAW_GCODE_READER_ITEM_CUSTOM_BEGIN)
            throw std::invalid_argument("A private G-code reader item is outside the custom range.");
        m_values[Key{type, target}] = value;
        rebuild_state();
    }

    void erase_custom_value(uint32_t type, uint32_t target)
    {
        if (type < RAW_GCODE_READER_ITEM_CUSTOM_BEGIN)
            throw std::invalid_argument("A private G-code reader item is outside the custom range.");
        m_values.erase(Key{type, target});
        rebuild_state();
    }

    // Advance the physical line cursor after consuming one line terminator.
    // Overflow is rejected rather than wrapping source diagnostics to line 0.
    void advance_line_number()
    {
        const uint32_t line = line_number();
        if (line == std::numeric_limits<uint32_t>::max())
            throw std::overflow_error("G-code reader line number exceeds uint32_t.");
        set_line_number(line + 1);
    }

    // Frequently consumed state slots. Coordinates and E are expressed in the
    // normalized units declared by the Reader contract; targeted values require
    // the logical tool or physical fan identifier used in the event stream.
    std::optional<double> x() const { return value(RAW_GCODE_READER_ITEM_X); }
    std::optional<double> y() const { return value(RAW_GCODE_READER_ITEM_Y); }
    std::optional<double> z() const { return value(RAW_GCODE_READER_ITEM_Z); }
    std::optional<double> e(uint32_t tool) const { return value(RAW_GCODE_READER_ITEM_E, tool); }
    std::optional<double> tool_temperature(uint32_t tool) const
    { return value(RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE, tool); }
    std::optional<double> fan_speed(uint32_t fan) const
    { return value(RAW_GCODE_READER_ITEM_FAN_SPEED, fan); }

    uint32_t line_number() const
    {
        const std::optional<uint32_t> line = uint32_value(RAW_GCODE_READER_ITEM_LINE_NUMBER);
        if (!line)
            throw std::logic_error("A G-code reader state has no line number.");
        return *line;
    }

    std::optional<bool> units_are_millimetres() const
    { return boolean_value(RAW_GCODE_READER_ITEM_UNITS_MM); }
    std::optional<bool> xyz_is_absolute() const
    { return boolean_value(RAW_GCODE_READER_ITEM_XYZ_ABSOLUTE); }
    std::optional<bool> e_is_absolute() const
    { return boolean_value(RAW_GCODE_READER_ITEM_E_ABSOLUTE); }
    std::optional<uint32_t> active_tool() const
    { return uint32_value(RAW_GCODE_READER_ITEM_ACTIVE_TOOL); }

private:
    struct Key
    {
        uint32_t type;
        uint32_t target;

        bool operator<(const Key &other) const
        {
            return type < other.type || (type == other.type && target < other.target);
        }
    };

    // Identify temperature targets whose matching REACHED marker must track
    // whether the latest requested target has actually been waited for.
    static bool is_temperature_item(uint32_t type)
    {
        return type == RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE ||
               type == RAW_GCODE_READER_ITEM_BED_TEMPERATURE ||
               type == RAW_GCODE_READER_ITEM_CHAMBER_TEMPERATURE;
    }

    // Map a requested-temperature slot to its reached-temperature slot.
    // Calling this for another item is a programming error.
    static uint32_t reached_temperature_item(uint32_t type)
    {
        switch (type) {
        case RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE:
            return RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE_REACHED;
        case RAW_GCODE_READER_ITEM_BED_TEMPERATURE:
            return RAW_GCODE_READER_ITEM_BED_TEMPERATURE_REACHED;
        case RAW_GCODE_READER_ITEM_CHAMBER_TEMPERATURE:
            return RAW_GCODE_READER_ITEM_CHAMBER_TEMPERATURE_REACHED;
        default:
            throw std::invalid_argument("A non-temperature item cannot be waited for.");
        }
    }

    // Values cross the ABI as doubles. Mark slots which must nevertheless be
    // exactly representable non-negative uint32 values.
    static bool requires_integral_value(uint32_t type)
    {
        switch (type) {
        case RAW_GCODE_READER_ITEM_ACTIVE_TOOL:
        case RAW_GCODE_READER_ITEM_TOOL_NOZZLE:
        case RAW_GCODE_READER_ITEM_TOOL_FAN:
        case RAW_GCODE_READER_ITEM_TOOL_E_AXIS:
        case RAW_GCODE_READER_ITEM_UNITS_MM:
        case RAW_GCODE_READER_ITEM_XYZ_ABSOLUTE:
        case RAW_GCODE_READER_ITEM_E_ABSOLUTE:
        case RAW_GCODE_READER_ITEM_VOLUMETRIC:
        case RAW_GCODE_READER_ITEM_EXTRUSION_ROLE:
        case RAW_GCODE_READER_ITEM_LAYER:
        case RAW_GCODE_READER_ITEM_OBJECT:
        case RAW_GCODE_READER_ITEM_MARKER:
        case RAW_GCODE_READER_ITEM_ARC_PLANE:
        case RAW_GCODE_READER_ITEM_ARC_CLOCKWISE:
        case RAW_GCODE_READER_ITEM_ARC_TURNS:
        case RAW_GCODE_READER_ITEM_LINE_NUMBER:
        case RAW_GCODE_READER_ITEM_READER_ID:
        case RAW_GCODE_READER_ITEM_DIAGNOSTIC_TYPE:
            return true;
        default:
            return false;
        }
    }

    // Return whether a standard item has one target-zero slot; per-tool/device items keep their target.
    static bool requires_zero_target(uint32_t type)
    {
        switch (type) {
        case RAW_GCODE_READER_ITEM_X:
        case RAW_GCODE_READER_ITEM_Y:
        case RAW_GCODE_READER_ITEM_Z:
        case RAW_GCODE_READER_ITEM_FEEDRATE:
        case RAW_GCODE_READER_ITEM_UNITS_MM:
        case RAW_GCODE_READER_ITEM_XYZ_ABSOLUTE:
        case RAW_GCODE_READER_ITEM_E_ABSOLUTE:
        case RAW_GCODE_READER_ITEM_VOLUMETRIC:
        case RAW_GCODE_READER_ITEM_ACTIVE_TOOL:
        case RAW_GCODE_READER_ITEM_BED_TEMPERATURE:
        case RAW_GCODE_READER_ITEM_CHAMBER_TEMPERATURE:
        case RAW_GCODE_READER_ITEM_SPEED_FACTOR:
        case RAW_GCODE_READER_ITEM_EXTRUSION_FACTOR:
        case RAW_GCODE_READER_ITEM_ACCELERATION:
        case RAW_GCODE_READER_ITEM_EXTRUSION_ROLE:
        case RAW_GCODE_READER_ITEM_WIDTH:
        case RAW_GCODE_READER_ITEM_HEIGHT:
        case RAW_GCODE_READER_ITEM_LAYER:
        case RAW_GCODE_READER_ITEM_OBJECT:
        case RAW_GCODE_READER_ITEM_MARKER:
        case RAW_GCODE_READER_ITEM_ARC_CENTER_X:
        case RAW_GCODE_READER_ITEM_ARC_CENTER_Y:
        case RAW_GCODE_READER_ITEM_ARC_CENTER_Z:
        case RAW_GCODE_READER_ITEM_ARC_PLANE:
        case RAW_GCODE_READER_ITEM_ARC_CLOCKWISE:
        case RAW_GCODE_READER_ITEM_ARC_TURNS:
        case RAW_GCODE_READER_ITEM_BED_TEMPERATURE_REACHED:
        case RAW_GCODE_READER_ITEM_CHAMBER_TEMPERATURE_REACHED:
        case RAW_GCODE_READER_ITEM_LINE_NUMBER:
        case RAW_GCODE_READER_ITEM_READER_ID:
        case RAW_GCODE_READER_ITEM_DIAGNOSTIC_TYPE:
        case RAW_GCODE_READER_ITEM_MAX_LAYER_HEIGHT:
            return true;
        default:
            return false;
        }
    }

    // Validate one scalar item's vocabulary, target, finite value, and stricter integer/boolean representation.
    static void validate_item(const raw_gcode_reader_event_item &item)
    {
        if (!std::isfinite(item.value))
            throw std::invalid_argument("A G-code reader item contains a non-finite value.");
        if (item.type == 0)
            throw std::invalid_argument("A G-code reader item has an invalid type.");
        if (item.type > RAW_GCODE_READER_ITEM_MAX_LAYER_HEIGHT &&
            item.type < RAW_GCODE_READER_ITEM_CUSTOM_BEGIN)
            throw std::invalid_argument("A G-code reader item uses an unknown type.");
        if (item.type >= RAW_GCODE_READER_ITEM_CUSTOM_BEGIN)
            return;
        if (requires_zero_target(item.type) && item.target != 0)
            throw std::invalid_argument("A G-code reader global item has a nonzero target.");
        if (requires_integral_value(item.type) &&
            (item.value < 0.0 || item.value > double(UINT32_MAX) || std::trunc(item.value) != item.value))
            throw std::invalid_argument("A G-code reader integer item is not exactly representable.");
        if ((item.type == RAW_GCODE_READER_ITEM_UNITS_MM ||
             item.type == RAW_GCODE_READER_ITEM_XYZ_ABSOLUTE ||
             item.type == RAW_GCODE_READER_ITEM_E_ABSOLUTE ||
             item.type == RAW_GCODE_READER_ITEM_VOLUMETRIC ||
             item.type == RAW_GCODE_READER_ITEM_ARC_CLOCKWISE) &&
            item.value > 1.0)
            throw std::invalid_argument("A G-code reader boolean item must be zero or one.");
        if (item.type == RAW_GCODE_READER_ITEM_MARKER &&
            (item.value < RAW_GCODE_READER_MARKER_SEAM ||
             item.value > RAW_GCODE_READER_MARKER_CUSTOM_GCODE))
            throw std::invalid_argument("A G-code reader marker item has an unknown marker type.");
        if (item.type == RAW_GCODE_READER_ITEM_DIAGNOSTIC_TYPE &&
            (item.value < RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_COMMAND ||
             item.value > RAW_GCODE_READER_DIAGNOSTIC_UNSUPPORTED_STATE_EFFECT))
            throw std::invalid_argument("A G-code reader diagnostic item has an unknown diagnostic type.");
    }

    // Validate a borrowed array and reject duplicate keys. Duplicate deltas are
    // ambiguous because their ordering inside one semantic event is undefined.
    static void validate_items(const raw_gcode_reader_event_item *items,
                               uint32_t item_count)
    {
        if (item_count != 0 && items == nullptr)
            throw std::invalid_argument("A G-code reader state or event has items but no item array.");
        std::map<Key, bool> seen;
        for (uint32_t index = 0; index < item_count; ++index) {
            validate_item(items[index]);
            if (!seen.emplace(Key{items[index].type, items[index].target}, true).second)
                throw std::invalid_argument("A G-code reader state or event duplicates an item target.");
        }
    }

    // Validate event metadata in addition to its items. Every event carries a
    // physical line number so consumers can associate it with source text.
    static void validate_event(const raw_gcode_reader_event &event)
    {
        validate_items(event.items, event.item_count);
        if (event.kind < RAW_GCODE_READER_EVENT_STATE_UPDATE ||
            event.kind > RAW_GCODE_READER_EVENT_INVALIDATE)
            throw std::invalid_argument("A G-code reader event has an invalid action kind.");
        bool has_line_number = false;
        bool has_marker = false;
        bool has_diagnostic = false;
        for (uint32_t index = 0; index < event.item_count; ++index) {
            has_line_number = has_line_number ||
                event.items[index].type == RAW_GCODE_READER_ITEM_LINE_NUMBER;
            has_marker = has_marker ||
                event.items[index].type == RAW_GCODE_READER_ITEM_MARKER;
            has_diagnostic = has_diagnostic ||
                event.items[index].type == RAW_GCODE_READER_ITEM_DIAGNOSTIC_TYPE;
        }
        if (!has_line_number)
            throw std::invalid_argument("A G-code reader event has no line number.");
        if (event.kind == RAW_GCODE_READER_EVENT_MARKER && !has_marker)
            throw std::invalid_argument("A G-code reader marker event has no marker type.");
        if (event.kind != RAW_GCODE_READER_EVENT_MARKER && has_marker)
            throw std::invalid_argument("A non-marker G-code reader event contains a marker type.");
        if (event.kind == RAW_GCODE_READER_EVENT_DIAGNOSTIC && !has_diagnostic)
            throw std::invalid_argument("A G-code reader diagnostic event has no diagnostic type.");
        if (event.kind != RAW_GCODE_READER_EVENT_DIAGNOSTIC && has_diagnostic)
            throw std::invalid_argument("A non-diagnostic G-code reader event contains a diagnostic type.");
    }

    // Copy a validated borrowed array into an owning map.
    static void copy_items(const raw_gcode_reader_event_item *items,
                           uint32_t item_count,
                           std::map<Key, double> &destination)
    {
        for (uint32_t index = 0; index < item_count; ++index)
            destination[Key{items[index].type, items[index].target}] = items[index].value;
    }

    // Complete states and post-event states must always preserve the mandatory
    // line cursor, including after INVALIDATE events.
    static void require_line_number(const std::map<Key, double> &values)
    {
        if (values.find(Key{RAW_GCODE_READER_ITEM_LINE_NUMBER, 0}) == values.end())
            throw std::invalid_argument("A G-code reader state or event has no line number.");
    }

    // Reader identity was introduced after the first complete-state fixtures.
    // Normalize an omitted value to the portable schema so generic producers
    // need not synthesize a redundant item.
    static void normalize_reader_id(std::map<Key, double> &values)
    {
        values.emplace(Key{RAW_GCODE_READER_ITEM_READER_ID, 0},
                       double(GCODE_READER_STATE_ID_PORTABLE));
    }

    // Rebuild the contiguous C view after every mutation, copy or move. Its
    // pointer is borrowed and remains valid only until the next rebuild.
    void rebuild_state()
    {
        m_state_items.clear();
        m_state_items.reserve(m_values.size());
        for (const std::pair<const Key, double> &entry : m_values)
            m_state_items.push_back({entry.first.type, entry.first.target, entry.second});
        m_state.items = m_state_items.empty() ? nullptr : m_state_items.data();
        m_state.item_count = static_cast<uint32_t>(m_state_items.size());
    }

    // Convert already validated scalar slots into convenient typed optionals.
    std::optional<bool> boolean_value(uint32_t type) const
    {
        const std::optional<double> raw = value(type);
        return raw ? std::optional<bool>(*raw != 0.0) : std::optional<bool>();
    }

    std::optional<uint32_t> uint32_value(uint32_t type) const
    {
        const std::optional<double> raw = value(type);
        return raw ? std::optional<uint32_t>(uint32_t(*raw)) : std::optional<uint32_t>();
    }

    std::map<Key, double> m_values;
    std::vector<raw_gcode_reader_event_item> m_state_items;
    raw_gcode_reader_state m_state = {};
};

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_gcode_GCodeStateTracker_hpp_
