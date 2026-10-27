///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_GCode_Reader_KlipperGCodeReader_hpp_
#define slic3r_Plugins_GCode_Reader_KlipperGCodeReader_hpp_

#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

#include "libslic3r/Api/plugin/cpp/gcode/BuiltinGCodeReaderBase.hpp"

/*
Klipper G-code reader
=====================

Klipper combines standard G/M commands with named textual commands. Both
dispatchers call their parent first, then route only Klipper additions to
virtual handlers. Print-backed extruder names follow the tool count and enabled
tool_name fallback used by KlipperGCodeFirmwareSession::setup_firmware;
external files accept only conventional names. Named heater targets use the
same portable temperature events as M104/M109 and M140/M190. A temperature
wait publishes the known target only when MINIMUM or MAXIMUM proves it;
otherwise it publishes a state update without changing the target.

Klipper names objects with strings while the portable Reader state exposes a
numeric object ID. A registry scoped to one read assigns those IDs in DEFINE
order, or in first-START order when definitions are absent. The registry never
enters GCodeStateTracker: custom tracker items belong to the caller and must be
preserved exactly. The provider may serve concurrent reads, so access to the
short-lived registries is serialized while parsing itself remains concurrent.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

class KlipperGCodeReader : public BuiltinGCodeReaderBase
{
public:
    explicit KlipperGCodeReader(orchestrator_handle *orchestrator);

protected:
    // Install the exact object-name registry for this invocation, delegate the
    // complete stream to the common parser, then remove the registry on both
    // success and exception. The tracker address is only a run-local lookup
    // key and is never dereferenced after this call returns.
    void read_impl(const plugin_run_context &run_context,
                   const run_ctx_gcode_reader &context,
                   GCodeStateTracker &state) const override;
    // Chain textual dispatch through the parent before trying Klipper's named
    // commands. True is returned only after a process_*() handler runs.
    bool consume_text_command(const plugin_run_context &run_context,
                              const run_ctx_gcode_reader &context,
                              GCodeStateTracker &state,
                              const GCodeLineParser::GCodeLine &line) const override;
    // Chain portable numeric commands first, then add Klipper M204.
    bool consume_numeric_command(const plugin_run_context &run_context,
                                 const run_ctx_gcode_reader &context,
                                 GCodeStateTracker &state,
                                 const GCodeLineParser::GCodeLine &line,
                                 char letter,
                                 uint32_t number) const override;
    // Resolve EXTRUDER=name against the Print configuration or conventional
    // external-file names and publish the resulting active tool.
    virtual void process_ACTIVATE_EXTRUDER(
        const plugin_run_context &run_context,
        const run_ctx_gcode_reader &context,
        GCodeStateTracker &state,
        const GCodeLineParser::GCodeLine &line) const;
    // Preserve a valid pressure-advance command as a known state update. Its
    // value has no portable Reader item yet.
    virtual void process_SET_PRESSURE_ADVANCE(
        const plugin_run_context &run_context,
        const run_ctx_gcode_reader &context,
        GCodeStateTracker &state,
        const GCodeLineParser::GCodeLine &line) const;
    // Resolve a named Klipper heater target and publish a non-waiting temperature event.
    virtual void process_SET_HEATER_TEMPERATURE(
        const plugin_run_context &run_context,
        const run_ctx_gcode_reader &context,
        GCodeStateTracker &state,
        const GCodeLineParser::GCodeLine &line) const;
    // Resolve a named heater threshold and publish a wait with the known target only when the threshold proves it; otherwise publish an unchanged state update.
    virtual void process_TEMPERATURE_WAIT(
        const plugin_run_context &run_context,
        const run_ctx_gcode_reader &context,
        GCodeStateTracker &state,
        const GCodeLineParser::GCodeLine &line) const;
    // Publish Klipper PAUSE as a timeline marker.
    virtual void process_PAUSE(const plugin_run_context &run_context,
                               const run_ctx_gcode_reader &context,
                               GCodeStateTracker &state) const;
    // Definitions establish the canonical order used by subsequent named
    // START commands. With no definitions, START allocates IDs on first use.
    virtual void process_EXCLUDE_OBJECT_DEFINE(
        const plugin_run_context &run_context,
        const run_ctx_gcode_reader &context,
        GCodeStateTracker &state,
        const GCodeLineParser::GCodeLine &line) const;
    virtual void process_EXCLUDE_OBJECT_START(
        const plugin_run_context &run_context,
        const run_ctx_gcode_reader &context,
        GCodeStateTracker &state,
        const GCodeLineParser::GCodeLine &line) const;
    virtual void process_EXCLUDE_OBJECT_END(
        const plugin_run_context &run_context,
        const run_ctx_gcode_reader &context,
        GCodeStateTracker &state,
        const GCodeLineParser::GCodeLine &line) const;
    // Resolve Klipper's S/P/T forms to the single portable acceleration active
    // after the command.
    virtual void process_M204(const plugin_run_context &run_context,
                              const run_ctx_gcode_reader &context,
                              GCodeStateTracker &state,
                              const GCodeLineParser::GCodeLine &line) const;
    bool is_neutral_command(char letter, uint32_t number) const override;
    bool has_unsupported_state_effect(char letter,
                                      uint32_t number) const override;
    std::optional<raw_gcode_reader_marker_type> marker_type(
        char letter, uint32_t number) const override;

private:
    struct ObjectRegistry
    {
        std::unordered_map<std::string, uint32_t> ids;
        uint32_t next_id = 0;
    };

    // Resolve a named bed or tool heater and publish its target, proven wait, or state-only event.
    void publish_named_heater_event(
        const plugin_run_context &run_context,
        const run_ctx_gcode_reader &context,
        GCodeStateTracker &state,
        std::string_view name,
        raw_gcode_reader_event_kind kind,
        double value,
        bool minimum_wait) const;
    // Return the existing canonical ID for a Klipper object name, or allocate
    // the next one when DEFINE/START encounters that exact name for the first
    // time in this run.
    uint32_t find_or_add_object_id(
        const GCodeStateTracker &state, std::string_view name) const;
    void erase_object_registry(const GCodeStateTracker &state) const noexcept;

    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    uint32_t probe_impl(std::string_view prefix) const override;

    mutable std::mutex m_object_registries_mutex;
    mutable std::unordered_map<const GCodeStateTracker *, ObjectRegistry>
        m_object_registries;
};

}}} // namespace slic3r_api::GCodeGeneration::Reader

#endif // slic3r_Plugins_GCode_Reader_KlipperGCodeReader_hpp_
