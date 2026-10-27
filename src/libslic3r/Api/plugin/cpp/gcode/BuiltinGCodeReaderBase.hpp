///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_gcode_BuiltinGCodeReaderBase_hpp_
#define slic3r_Api_plugin_cpp_gcode_BuiltinGCodeReaderBase_hpp_

#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

#include "libslic3r/Api/plugin/cpp/gcode/GCodeLineParser.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeReaderPlugin.hpp"

/*
Reusable G-code reader base (also used by built-in dialects)
==========================================================

BuiltinGCodeReaderBase implements the stream, portable command dispatcher and
state machinery shared by built-in and external Readers. Link the public
slic3r::plugin_sdk_cpp target; no host C++ headers are required.
Every derived dispatcher calls
its parent first, then handles only commands introduced at its own level:

    chunks -> complete physical line -> textual dialect hook
                                     -> chained numeric dispatchers
                                          +-> common virtual process_*()
                                          `-> dialect virtual process_*()
                                     -> marker/neutral classification
                                     -> conservative state invalidation

The textual hook is for commands such as Klipper's NAME=value syntax. A numeric
dispatcher returns true only after calling a process_*() handler. To add a new
command, a subclass calls the parent dispatcher and examines the command only
after a false result. To change a command already known by a parent, it
overrides that command's virtual process_*() handler instead. The two predicates
classify commands which either produce a timeline marker or are understood but
have no portable Reader state item.

Returning true from a command hook means that the derived Reader has completely
handled the command and published the required event. Returning false delegates
to the next stage. Unknown commands ultimately invalidate portable and private
state so a later writer never relies on values an unsupported command may have
changed.

Plugin instances contain no stream state. Each read creates its own tracker and
line buffer, so immutable provider instances may be executed concurrently.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

class BuiltinGCodeReaderBase : public GCodeReaderPluginBase
{
public:
    explicit BuiltinGCodeReaderBase(orchestrator_handle *orchestrator);

protected:
    // Dispatch a textual command known by this class. A subclass first calls
    // its parent and only tries its additional commands when false is returned.
    // True therefore means that one process_*() handler was called and fully
    // published the command's synchronous event.
    virtual bool consume_text_command(const plugin_run_context &run_context,
                                      const run_ctx_gcode_reader &context,
                                      GCodeStateTracker &state,
                                      const GCodeLineParser::GCodeLine &line) const;
    // Dispatch a numeric command known by this class. The built-in base routes
    // the portable vocabulary to virtual process_*() handlers. A subclass
    // calls this implementation first, then dispatches only additional command
    // numbers when it returns false. Shared semantics are customized by
    // overriding the corresponding process_*() handler instead.
    virtual bool consume_numeric_command(const plugin_run_context &run_context,
                                         const run_ctx_gcode_reader &context,
                                         GCodeStateTracker &state,
                                         const GCodeLineParser::GCodeLine &line,
                                         char letter,
                                         uint32_t number) const;
    // Return true for a command understood by the dialect which cannot alter
    // any value represented by raw_gcode_reader_state.
    virtual bool is_neutral_command(char letter, uint32_t number) const;
    // Return true for a recognized command which changes machine state that
    // this Reader cannot represent yet. Such commands produce a warning but
    // do not invalidate unrelated portable values.
    virtual bool has_unsupported_state_effect(char letter,
                                              uint32_t number) const;
    // Return the point-in-time annotation produced by a command, or no value
    // when normal command dispatch should continue.
    virtual std::optional<raw_gcode_reader_marker_type> marker_type(
        char letter, uint32_t number) const;
    // Validate provider-private state at both boundaries of one read. The
    // generic base has no private schema; dialects use this hook to reject an
    // incomplete snapshot even when the input stream contains no command.
    virtual void validate_private_state(const GCodeStateTracker &state) const;
    // Common command handlers are virtual so a dialect can replace one
    // command family without reimplementing line parsing or dispatch. An
    // override owns the complete command and must synchronously publish every
    // event it requires. It may call the qualified base implementation when
    // the dialect only adds behavior around the portable interpretation.
    // Resolve a linear destination, invalidate unresolved relative axes, then
    // publish one LINEAR_MOVE containing the values that remain known.
    virtual void process_G0_G1(const plugin_run_context &run_context,
                               const run_ctx_gcode_reader &context,
                               GCodeStateTracker &state,
                               const GCodeLineParser::GCodeLine &line) const;
    // Apply the default homing convention: selected axes become logical zero.
    // Firmware-specific readers may override this when homing offsets differ.
    virtual void process_G28(const plugin_run_context &run_context,
                             const run_ctx_gcode_reader &context,
                             GCodeStateTracker &state,
                             const GCodeLineParser::GCodeLine &line) const;
    // Resolve an arc destination and its I/J/K offsets, recording whether the
    // command selected clockwise G2 or counter-clockwise G3 interpolation.
    virtual void process_G2_G3(const plugin_run_context &run_context,
                               const run_ctx_gcode_reader &context,
                               GCodeStateTracker &state,
                               const GCodeLineParser::GCodeLine &line,
                               uint32_t number) const;
    // Select inches for G20 or millimetres for G21 in portable modal state.
    virtual void process_G20_G21(const plugin_run_context &run_context,
                                 const run_ctx_gcode_reader &context,
                                 GCodeStateTracker &state,
                                 uint32_t number) const;
    // Select absolute G90 or relative G91 positioning for the XYZ axes.
    virtual void process_G90_G91(const plugin_run_context &run_context,
                                 const run_ctx_gcode_reader &context,
                                 GCodeStateTracker &state,
                                 uint32_t number) const;
    // Replace only the coordinates explicitly supplied by G92. E belongs to
    // the active tool while XYZ remain global.
    virtual void process_G92(const plugin_run_context &run_context,
                             const run_ctx_gcode_reader &context,
                             GCodeStateTracker &state,
                             const GCodeLineParser::GCodeLine &line) const;
    // Select absolute M82 or relative M83 coordinates for extrusion.
    virtual void process_M82_M83(const plugin_run_context &run_context,
                                 const run_ctx_gcode_reader &context,
                                 GCodeStateTracker &state,
                                 uint32_t number) const;
    // Publish selection of the non-negative tool parsed from a numeric T word.
    virtual void process_T(const plugin_run_context &run_context,
                           const run_ctx_gcode_reader &context,
                           GCodeStateTracker &state,
                           uint32_t tool) const;
    // Set a tool target with M104 or wait for it with M109. Missing S leaves
    // temperature state unchanged, matching the conservative parser contract.
    virtual void process_M104_M109(const plugin_run_context &run_context,
                                   const run_ctx_gcode_reader &context,
                                   GCodeStateTracker &state,
                                   const GCodeLineParser::GCodeLine &line,
                                   uint32_t number) const;
    // Set the bed target with M140 or wait for it with M190.
    virtual void process_M140_M190(const plugin_run_context &run_context,
                                   const run_ctx_gcode_reader &context,
                                   GCodeStateTracker &state,
                                   const GCodeLineParser::GCodeLine &line,
                                   uint32_t number) const;
    // Set the chamber target with M141 or wait for it with M191.
    virtual void process_M141_M191(const plugin_run_context &run_context,
                                   const run_ctx_gcode_reader &context,
                                   GCodeStateTracker &state,
                                   const GCodeLineParser::GCodeLine &line,
                                   uint32_t number) const;
    // Convert the conventional 0..255 M106 speed to percent, or stop the
    // selected fan for M107. Dialects with other units override this handler.
    virtual void process_M106_M107(const plugin_run_context &run_context,
                                   const run_ctx_gcode_reader &context,
                                   GCodeStateTracker &state,
                                   const GCodeLineParser::GCodeLine &line,
                                   uint32_t number) const;
    // Store the M220 speed multiplier expressed as a percentage.
    virtual void process_M220(const plugin_run_context &run_context,
                              const run_ctx_gcode_reader &context,
                              GCodeStateTracker &state,
                              const GCodeLineParser::GCodeLine &line) const;
    // Store the M221 extrusion multiplier expressed as a percentage.
    virtual void process_M221(const plugin_run_context &run_context,
                              const run_ctx_gcode_reader &context,
                              GCodeStateTracker &state,
                              const GCodeLineParser::GCodeLine &line) const;
    // Recognize M117 without changing modal state. Display text is not part of
    // the portable Reader model, and a valid display command is not a warning.
    virtual void process_M117(const plugin_run_context &run_context,
                              const run_ctx_gcode_reader &context,
                              GCodeStateTracker &state) const;
    // Convert Marlin/RepRap's zero-based M486 S value to the canonical Reader
    // identity. S-1 becomes UINT32_MAX and closes the current object.
    virtual void process_M486(const plugin_run_context &run_context,
                              const run_ctx_gcode_reader &context,
                              GCodeStateTracker &state,
                              const GCodeLineParser::GCodeLine &line) const;

    // Publish a caller-owned delta. The event and its temporary item storage
    // need only remain alive for this synchronous call.
    void publish_items(const plugin_run_context &run_context,
                       const run_ctx_gcode_reader &context,
                       GCodeStateTracker &state,
                       raw_gcode_reader_event_kind kind,
                       const std::vector<raw_gcode_reader_event_item> &items) const;
    // Report one stable non-fatal condition. The host maps this code to its
    // localized wording and severity; no plugin-owned text crosses the ABI.
    void publish_diagnostic(const plugin_run_context &run_context,
                            const run_ctx_gcode_reader &context,
                            GCodeStateTracker &state,
                            raw_gcode_reader_diagnostic_type type) const;
    // Report a malformed object marker and discard only the current object
    // identity. Positions and other machine state remain trustworthy because
    // the marker cannot affect them.
    void publish_invalid_object_marker(
        const plugin_run_context &run_context,
        const run_ctx_gcode_reader &context,
        GCodeStateTracker &state) const;
    // Publish a recognized command which changes no portable state.
    void publish_state_update(const plugin_run_context &run_context,
                              const run_ctx_gcode_reader &context,
                              GCodeStateTracker &state) const;
    // Report why the command cannot be represented, then discard every known
    // portable and provider-private value while preserving identity and line.
    void publish_unknown(const plugin_run_context &run_context,
                         const run_ctx_gcode_reader &context,
                         GCodeStateTracker &state,
                         raw_gcode_reader_diagnostic_type diagnostic =
                             RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS) const;

    // Shared parsing helpers are protected so a dialect can use exactly the
    // same strict command and parameter rules as the common parser.
    static uint32_t probe_marker(std::string_view prefix, std::string_view marker);
    static std::optional<std::string_view> named_parameter(std::string_view line,
                                                           std::string_view name);
    static uint32_t active_tool(const GCodeStateTracker &state);

    // Reassemble physical lines from arbitrary chunks and dispatch each
    // complete line in source order. It is protected so a dialect may bracket
    // the common parser with strictly run-local setup and cleanup. Overrides
    // must delegate exactly once and must not retain the borrowed arguments.
    void read_impl(const plugin_run_context &run_context,
                   const run_ctx_gcode_reader &context,
                   GCodeStateTracker &state) const override;

private:
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    // Parse recognized comment-only G-code and publish a state update, marker, or diagnostic.
    bool consume_common_comment(const plugin_run_context &run_context,
                                const run_ctx_gcode_reader &context,
                                GCodeStateTracker &state,
                                const GCodeLineParser::GCodeLine &line,
                                std::optional<raw_gcode_reader_marker_type> &pending_action) const;
    // Parse one physical line and publish events carrying the tracker's current
    // zero-based line number. The text excludes CR/LF terminators.
    void consume_line(const plugin_run_context &run_context,
                      const run_ctx_gcode_reader &context,
                      GCodeStateTracker &state,
                      std::string_view text,
                      std::optional<raw_gcode_reader_marker_type> &pending_action) const;
};

}}} // namespace slic3r_api::GCodeGeneration::Reader

#endif // slic3r_Api_plugin_cpp_gcode_BuiltinGCodeReaderBase_hpp_
