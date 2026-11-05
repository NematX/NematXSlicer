///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "GCodePreamble.hpp"
#include "DefaultGCodeFirmwareSession.hpp"
#include "GCodeThumbnailViews.hpp"
#include "GCodeLineParser.hpp"
#include "MachineEnvelope.hpp"
#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

/* Startup composition and shared script interpreter.
   All public geometry stays in the plan. This helper emits initialization and
   existing event trees, and updates the owning session after encoding commands.
   Automatic bed, chamber, and tool heating is supplied by the
   extrusion_edit.startup_temperature.default plugin after start_gcode; this
   helper keeps only startup mode, tool selection, and first-layer fan state.
   Thumbnail text may be retained for end_print, but no source view survives begin.
   A custom G-code E move changes the retraction state only when its own line
   declares it: a negative E move retracts with the comment "; retraction" and a
   positive E move restores with "; unretraction", ignoring case and surrounding
   spaces. Every other E move is raw. */
namespace slic3r_api { namespace GCodeGeneration {
namespace {
// Parse an exact command token, excluding comments and numeric suffixes.
bool command_code(std::string_view command, char prefix, int32_t *code_out)
{
    if (code_out == nullptr || command.size() < 2 || command.front() != prefix)
        return false;
    const char *begin = command.data() + 1;
    const char *end = command.data() + command.size();
    const std::from_chars_result parsed = std::from_chars(begin, end, *code_out);
    return parsed.ec == std::errc() && parsed.ptr == end;
}


// Count typed start scripts before writing any startup text.
size_t start_script_count(const ExtrusionEntity &root)
{
    const EPropertyCustomGcode *script = root.get(EPropertyCustomGcode::key);
    size_t count = script && script->kind == C_EXTRUSION_CUSTOM_GCODE_SCRIPT &&
        script->script_type == GCODE_SCRIPT_TYPE_START_GCODE ? 1 : 0;
    for (uint32_t idx = 0; idx < root.child_count(); ++idx)
        count += start_script_count(root.child(idx));
    return count;
}

// Classify the retraction declared by the comment of one external E move.
// Matching ignores case and surrounding spaces: "; retraction" declares a
// retraction and "; unretraction" declares its restore. Every other comment
// leaves the move raw, so it changes no retraction state.
ExternalMoveRetraction declared_retraction(const std::string_view comment)
{
    const auto matches = [comment](const std::string_view word) {
        size_t begin = 0;
        size_t end = comment.size();
        while (begin < end && std::isspace(static_cast<unsigned char>(comment[begin])))
            ++begin;
        while (end > begin && std::isspace(static_cast<unsigned char>(comment[end - 1])))
            --end;
        if (end - begin != word.size())
            return false;
        for (size_t idx = 0; idx < word.size(); ++idx)
            if (std::tolower(static_cast<unsigned char>(comment[begin + idx])) != word[idx])
                return false;
        return true;
    };
    if (matches("retraction"))
        return ExternalMoveRetraction::Retraction;
    if (matches("unretraction"))
        return ExternalMoveRetraction::Unretraction;
    return ExternalMoveRetraction::None;
}
}
class DefaultGCodeFirmwareSession::GCodeStateInterpreter
{
public:
    explicit GCodeStateInterpreter(DefaultGCodeFirmwareSession &session) : m_session(session) {}

    void apply(const std::string &gcode)
    {
        GCodeLineParser parser;
        parser.parse_buffer(gcode, [this](GCodeLineParser &, const GCodeLineParser::GCodeLine &line) {
            apply_line(line);
        });
    }

private:
    void apply_line(const GCodeLineParser::GCodeLine &line)
    {
        const std::string_view command = line.command();
        int32_t code = 0;
        if (command_code(command, 'T', &code)) {
            if (code >= 0 && code <= int32_t(std::numeric_limits<extruder_index_t>::max()))
                (void)m_session.select_extruder(static_cast<extruder_index_t>(code), false);
            return;
        }

        if (command_code(command, 'G', &code)) {
            if (code == 20) {
                m_session.m_units_in_mm = false;
                return;
            }
            if (code == 21) {
                m_session.m_units_in_mm = true;
                return;
            }
            if (code == 90) {
                m_session.m_xyz_relative_mode = false;
                return;
            }
            if (code == 91) {
                m_session.m_xyz_relative_mode = true;
                return;
            }
            if (code == 92) {
                apply_coordinate_reset(line);
                return;
            }
            if (code >= 0 && code <= 3)
                apply_move(line);
            return;
        }

        if (!command_code(command, 'M', &code))
            return;
        if (code == 82 || code == 83) {
            m_session.m_e_relative_mode = code == 83;
            for (DefaultExtruder &tool : m_session.m_extruders)
                tool.extrusion_axis().set_relative_mode(m_session.m_e_relative_mode);
            return;
        }
        if (code == 104 || code == 109) {
            apply_tool_temperature(line, code == 109);
            return;
        }
        if (code == 140 || code == 190) {
            apply_heater(line, m_session.m_printer.bed_heater(), code == 190);
            return;
        }
        if (code == 141 || code == 191) {
            apply_heater(line, m_session.m_printer.chamber_heater(), code == 191);
            return;
        }
        if (code == 106 || code == 107)
            apply_fan(line, code == 107);
    }

    double unit_scale() const { return m_session.m_units_in_mm ? 1.0 : 25.4; }

    // Resolve an explicit or current/script tool selector, returning no tool for missing, negative, or out-of-range IDs.
    std::optional<extruder_index_t> addressed_tool(const GCodeLineParser::GCodeLine &line,
                                                   char selector) const
    {
        int32_t tool_id = -1;
        if (line.parameter_value(selector, tool_id)) {
            if (tool_id == EXTRUDER_ID_UNKNOWN || tool_id < EXTRUDER_ID_UNKNOWN ||
                tool_id > (std::numeric_limits<extruder_index_t>::max)() ||
                tool_id >= int32_t(m_session.m_extruders.size()))
                return std::nullopt;
            return static_cast<extruder_index_t>(tool_id);
        }
        if (m_session.m_current_extruder_idx)
            return m_session.m_current_extruder_idx;
        return m_session.m_script_processing_tool;
    }

    void apply_coordinate_reset(const GCodeLineParser::GCodeLine &line)
    {
        const double scale = unit_scale();
        const std::optional<float> e = line.parameter_value('E');
        if (e) {
            const std::optional<extruder_index_t> tool_id = addressed_tool(line, 'T');
            if (tool_id)
                m_session.extruder(*tool_id).extrusion_axis().set_position(*e * scale);
        }

        const std::optional<float> x = line.parameter_value('X');
        const std::optional<float> y = line.parameter_value('Y');
        const std::optional<float> z = line.parameter_value('Z');
        const std::optional<c_vec3d> old_position = m_session.m_gantry.position();
        if (!old_position && !(x && y && z))
            return;
        c_vec3d position = old_position.value_or(c_vec3d{});
        if (x) position.x = *x * scale;
        if (y) position.y = *y * scale;
        if (z) position.z = *z * scale;
        m_session.m_gantry.set_position(position);
    }

    void apply_move(const GCodeLineParser::GCodeLine &line)
    {
        const double scale = unit_scale();
        const std::optional<float> feedrate = line.parameter_value('F');
        if (feedrate) {
            m_session.m_gantry.request_speed(*feedrate * scale / 60.0);
            m_session.m_gantry.mark_speed_encoded();
        }

        const std::optional<float> x = line.parameter_value('X');
        const std::optional<float> y = line.parameter_value('Y');
        const std::optional<float> z = line.parameter_value('Z');
        const std::optional<c_vec3d> old_position = m_session.m_gantry.position();
        if (old_position || (x && y && z)) {
            c_vec3d position = old_position.value_or(c_vec3d{});
            if (x)
                position.x = m_session.m_xyz_relative_mode ? position.x + *x * scale : *x * scale;
            if (y)
                position.y = m_session.m_xyz_relative_mode ? position.y + *y * scale : *y * scale;
            if (z)
                position.z = m_session.m_xyz_relative_mode ? position.z + *z * scale : *z * scale;
            m_session.m_gantry.set_position(position);
        }

        const std::optional<float> e = line.parameter_value('E');
        if (e) {
            const std::optional<extruder_index_t> tool_id = addressed_tool(line, 'T');
            if (tool_id)
                m_session.extruder(*tool_id).extrusion_axis().observe_external_move(
                    *e * scale, m_session.m_e_relative_mode,
                    declared_retraction(line.comment()));
        }
    }

    void apply_heater(const GCodeLineParser::GCodeLine &line, HeaterState &heater, bool waited)
    {
        std::optional<float> temperature = line.parameter_value('S');
        if (!temperature && waited)
            temperature = line.parameter_value('R');
        if (temperature && std::isfinite(*temperature) &&
            *temperature >= double(std::numeric_limits<int16_t>::min()) &&
            *temperature <= double(std::numeric_limits<int16_t>::max()))
            heater.synchronize_after_external_gcode(int16_t(std::lround(*temperature)), waited);
    }

    void apply_tool_temperature(const GCodeLineParser::GCodeLine &line, bool waited)
    {
        const std::optional<extruder_index_t> tool_id = addressed_tool(line, 'T');
        if (tool_id)
            apply_heater(line, m_session.extruder(*tool_id).heater(), waited);
    }

    void apply_fan(const GCodeLineParser::GCodeLine &line, bool stopped)
    {
        const std::optional<extruder_index_t> tool_id = addressed_tool(line, 'P');
        if (!tool_id)
            return;
        const float pwm = stopped ? 0.f : line.parameter_value('S').value_or(255.f);
        m_session.extruder(*tool_id).fan().synchronize_after_external_gcode(
            std::clamp(double(pwm) * 100.0 / 255.0, 0.0, 100.0));
    }

    DefaultGCodeFirmwareSession &m_session;
};


// Scan the first concrete plan visit, compose thumbnails and start events, and clear borrowed state on both success and failure; unresolved or out-of-range tools are rejected.
std::string GCodePreamble::begin(
    const Print &print,
    const ConstPrintingPlan &plan,
    const std::array<std::string, 3> &thumbnail_blocks)
{
    const Config config = print.config();
    m_config = &config;
    m_in_preamble = true;
    m_start_written = false;
    m_initial_tool = EXTRUDER_ID_UNKNOWN;
    m_trailing_thumbnails.clear();
    try {
        for (uint32_t gi = 0; gi < plan.group_count() && m_initial_tool == EXTRUDER_ID_UNKNOWN; ++gi) {
            const ConstPrintingGroup group = plan.group(gi);
            for (uint32_t li = 0;
                 li < group.layer_group_count() && m_initial_tool == EXTRUDER_ID_UNKNOWN;
                 ++li) {
                const ConstPrintingLayerGroup layer = group.layer_group(li);
                for (uint32_t ti = 0; ti < layer.tool_group_count(); ++ti) {
                    const extruder_index_t tool = layer.tool_group(ti).extruder_id();
                    if (tool <= EXTRUDER_ID_UNKNOWN || tool >= m_session.m_extruders.size())
                        throw std::invalid_argument("Preamble requires resolved, valid tool IDs.");
                    m_initial_tool = tool;
                    break;
                }
            }
        }
        const ConstPrintingScopeEvents events = plan.events();
        const size_t starts = events.has_before() ? start_script_count(events.before()) : 0;
        if (starts > 1) throw std::invalid_argument("The preamble contains multiple start scripts.");
        std::string output = thumbnail_blocks[RAW_GCODE_THUMBNAIL_FILE_PREFIX];
        output += m_session.write_pre_thumbnail(print, plan);
        output += thumbnail_blocks[RAW_GCODE_THUMBNAIL_BEFORE_START_GCODE];
        m_trailing_thumbnails = thumbnail_blocks[RAW_GCODE_THUMBNAIL_FILE_SUFFIX];
        output += m_session.write_post_thumbnail(print, plan);
        if (starts == 0) {
            output += m_session.write_pre_start({});
            output += m_session.write_post_start();
        }
        if (events.has_before()) output += m_session.write_event(events.before());
        m_config = nullptr; m_in_preamble = false;
        return output;
    } catch (...) { m_config = nullptr; m_in_preamble = false; throw; }
}

// Move the deferred text out so repeated cleanup cannot duplicate thumbnails.
std::string GCodePreamble::finish()
{
    std::string result;
    result.swap(m_trailing_thumbnails);
    return result;
}

// Analyze resolved commands, then emit only missing mode categories. The
// actual script is applied later, after these prefixes, by the same interpreter.
std::string GCodePreamble::pre_start(const std::string &resolved)
{
    if (!m_config || m_config->bool_or_default("start_gcode_manual", false)) return {};
    bool units = false, xyz = false, e = false;
    GCodeLineParser parser;
    parser.parse_buffer(resolved, [&](GCodeLineParser &, const GCodeLineParser::GCodeLine &line) {
        int32_t code = 0;
        if (command_code(line.command(), 'G', &code)) {
            units |= code == 20 || code == 21;
            xyz |= code == 90 || code == 91;
        }
        if (command_code(line.command(), 'M', &code)) e |= code == 82 || code == 83;
    });
    std::string output;
    if (!units) output += "G21\n";
    if (!xyz) output += "G90\n";
    if (!e) output += m_config->bool_or_default("use_relative_e_distances", false) ? "M83\n" : "M82\n";
    DefaultGCodeFirmwareSession::GCodeStateInterpreter(m_session).apply(output);
    return output;
}

// Restore startup modes, select the initial tool, and emit first-layer fan state; the no-argument result owns only these commands, while startup heater targets are appended by the temperature plugin after start_gcode and no input state is retained.
std::string GCodePreamble::post_start()
{
    if (!m_config || m_config->bool_or_default("start_gcode_manual", false)) return {};
    std::string output;
    const bool relative_e = m_config->bool_or_default("use_relative_e_distances", false);
    if (!m_session.m_units_in_mm) output += "G21\n";
    const bool reset_xyz = m_session.m_xyz_relative_mode;
    if (reset_xyz) output += "G90\n";
    if (reset_xyz || m_session.m_e_relative_mode != relative_e) output += relative_e ? "M83\n" : "M82\n";
    DefaultGCodeFirmwareSession::GCodeStateInterpreter(m_session).apply(output);
    const std::optional<MachineEnvelope> envelope = configured_machine_envelope(*m_config);
    if (envelope) output += m_session.encode_machine_envelope(*envelope);
    if (m_initial_tool == EXTRUDER_ID_UNKNOWN) return output;
    output += m_session.select_extruder(m_initial_tool);
    DefaultExtruder &tool = m_session.extruder(m_initial_tool);
    if (m_config->vector_int_or_default("disable_fan_first_layers", m_initial_tool, 0) > 0) {
        tool.fan().request_speed_percent(0.0);
        if (tool.fan().needs_encoding()) {
            output += m_session.encode_fan(m_initial_tool, tool.fan().effective_speed_percent());
            tool.fan().mark_encoded();
        }
    }
    return output;
}

// Prepare and execute one script with borrowed producer values, firmware execution state and writable machine outputs; the returned text is owned by the caller, while missing host options, parser errors and state failures propagate without retaining producer ownership.
std::string GCodePreamble::process_script(
    gcode_script_type script_type,
    const std::string &script,
    const Config *producer_config,
    extruder_index_t processing_extruder_id)
{
    if (!m_session.m_scripts.valid())
        throw std::invalid_argument(
            "Custom G-code scripts require a host script processor.");

    const std::string resolved_script = script.empty() ?
        m_session.resolve_empty_script(script_type, producer_config) : script;

    // The script producer has already frozen every structural placeholder in
    // its Config. The firmware adds only values that depend on the machine at
    // this exact execution point.
    GCodeScriptContext context = producer_config == nullptr ?
        m_session.m_scripts.prepare(script_type) : m_session.m_scripts.prepare(script_type, *producer_config);
    GCodeScriptConfig config = context.config();

    extruder_index_t processing_tool = m_session.m_current_extruder_idx.value_or(0);
    if (processing_extruder_id != EXTRUDER_ID_UNKNOWN)
        processing_tool = processing_extruder_id;
    if (processing_tool >= m_session.m_extruders.size())
        throw std::invalid_argument("A custom G-code script references an unknown processing extruder.");

    // Host-created execution options receive the state immediately before this
    // script. Per-tool lengths stay separated in SEMM, and producer-frozen layer
    // values take precedence over the firmware layer state.
    const bool frozen_layer_num = producer_config != nullptr && producer_config->has("layer_num");
    const bool frozen_layer_z = producer_config != nullptr && producer_config->has("layer_z");
    const bool frozen_layer_height = producer_config != nullptr &&
        producer_config->has("layer_current_height");
    if (!frozen_layer_num)
        config.set("layer_num", m_session.m_layer_num);
    if (!frozen_layer_z)
        config.set("layer_z", unscaled(m_session.m_layer_print_z));
    if (!frozen_layer_height)
        config.set("layer_current_height", unscaled(m_session.m_layer_height));

    double zhop = 0.0;
    const std::optional<c_vec3d> gantry_position = m_session.m_gantry.position();
    if (m_session.m_layer_num >= 0 && gantry_position) {
        const double tool_z_offset = m_session.m_current_extruder_idx ?
            m_session.extruder(*m_session.m_current_extruder_idx).z_offset() : 0.0;
        const double layer_machine_z = unscaled(m_session.m_layer_print_z) +
            m_session.m_gantry.z_offset() - tool_z_offset;
        const double lift = gantry_position->z - layer_machine_z;
        if (lift > EPSILON)
            zhop = lift;
    }
    config.set("zhop", zhop);

    const std::vector<double> used_filament = m_session.filament_used_mm();
    std::vector<double> volumes;
    std::vector<double> weights;
    volumes.reserve(m_session.m_extruders.size());
    weights.reserve(m_session.m_extruders.size());
    double total_volume = 0.0;
    double total_weight = 0.0;
    for (size_t tool_idx = 0; tool_idx < m_session.m_extruders.size(); ++tool_idx) {
        const double volume = used_filament[tool_idx] *
            m_session.m_extruders[tool_idx].extrusion_axis().filament_crossection();
        const double weight = volume * m_session.m_extruders[tool_idx].filament_density() * 0.001;
        volumes.push_back(volume);
        weights.push_back(weight);
        total_volume += volume;
        total_weight += weight;
    }
    config.set("current_object_idx", m_session.m_current_object_idx);
    config.set("extruded_volume", volumes);
    config.set("extruded_weight", weights);
    config.set("extruded_volume_total", total_volume);
    config.set("extruded_weight_total", total_weight);

    m_session.complete_script_context(script_type, processing_tool, config);

    std::vector<double> used_filament_snapshot;
    if (script_type == GCODE_SCRIPT_TYPE_BEFORE_LAYER_GCODE &&
        config.has("layer_used_filament")) {
        used_filament_snapshot.reserve(m_session.m_extruders.size());
        std::vector<double> layer_used_filament;
        layer_used_filament.reserve(m_session.m_extruders.size());
        for (size_t tool_idx = 0; tool_idx < m_session.m_extruders.size(); ++tool_idx) {
            const double used = used_filament[tool_idx];
            used_filament_snapshot.push_back(used);
            layer_used_filament.push_back(used - m_session.m_last_layer_used_filament[tool_idx]);
        }
        config.set("layer_used_filament", layer_used_filament);
    }
    if (config.has("gcode_bed_temperature") && m_session.m_printer.bed_heater().requested_temperature())
        config.set("gcode_bed_temperature", int32_t(m_session.m_printer.bed_heater().effective_temperature()));

    // Export the complete current machine snapshot before the parser runs.
    // An unknown position uses a stable placeholder but remains unknown unless
    // the script or the generated G-code actually changes that vector.
    const std::optional<c_vec3d> old_position = m_session.m_gantry.position();
    const std::vector<double> position = old_position ?
        std::vector<double>{old_position->x, old_position->y, old_position->z} :
        std::vector<double>{0.0, 0.0, 0.0};
    config.set("position", position);

    std::vector<double> old_e_positions;
    std::vector<double> old_retracted;
    std::vector<double> old_restart_extra;
    old_e_positions.reserve(m_session.m_extruders.size());
    old_retracted.reserve(m_session.m_extruders.size());
    old_restart_extra.reserve(m_session.m_extruders.size());
    for (const DefaultExtruder &tool : m_session.m_extruders) {
        old_e_positions.push_back(tool.extrusion_axis().position());
        old_retracted.push_back(tool.extrusion_axis().retracted());
        old_restart_extra.push_back(tool.extrusion_axis().restart_extra());
    }
    config.set("e_retracted", old_retracted);
    config.set("e_restart_extra", old_restart_extra);
    if (config.has("e_position"))
        config.set("e_position", old_e_positions);

    m_session.m_script_processing_tool = processing_tool;
    std::string prefix;
    std::string output;
    const bool startup = m_in_preamble && script_type == GCODE_SCRIPT_TYPE_START_GCODE;
    try {
        output = context.process(resolved_script, processing_tool);
        if (startup) {
            if (m_start_written) throw std::invalid_argument("The preamble contains multiple start scripts.");
            m_start_written = true;
            prefix = m_session.write_pre_start(output);
        }
        DefaultGCodeFirmwareSession::GCodeStateInterpreter(m_session).apply(output);
        m_session.m_script_processing_tool.reset();
    } catch (...) {
        m_session.m_script_processing_tool.reset();
        throw;
    }

    // Read and validate every output before mutating any session component.
    // This keeps script application atomic even when the last vector is bad.
    const std::vector<double> new_position = config.get_floats("position");
    const std::vector<double> new_retracted = config.get_floats("e_retracted");
    const std::vector<double> new_restart_extra = config.get_floats("e_restart_extra");
    const std::vector<double> new_e_positions = config.has("e_position") ?
        config.get_floats("e_position") : std::vector<double>();
    if (new_position.size() != 3 || new_retracted.size() != m_session.m_extruders.size() ||
        new_restart_extra.size() != m_session.m_extruders.size() ||
        (!new_e_positions.empty() && new_e_positions.size() != m_session.m_extruders.size()))
        throw std::invalid_argument("A G-code script returned an invalid machine-state vector size.");

    for (double value : new_position) {
        if (!std::isfinite(value))
            throw std::invalid_argument("A G-code script returned a non-finite position.");
    }
    for (size_t tool_idx = 0; tool_idx < m_session.m_extruders.size(); ++tool_idx) {
        if (!std::isfinite(new_retracted[tool_idx]) ||
            !std::isfinite(new_restart_extra[tool_idx]) ||
            new_retracted[tool_idx] < -EPSILON || new_restart_extra[tool_idx] < -EPSILON ||
            (!new_e_positions.empty() && !std::isfinite(new_e_positions[tool_idx])))
            throw std::invalid_argument("A G-code script returned an invalid extrusion state.");
    }

    // A script which leaves the placeholder position untouched must not turn
    // an unknown Gantry position into an invented origin.
    if (new_position != position)
        m_session.m_gantry.set_position(c_vec3d{new_position[0], new_position[1], new_position[2]});
    for (size_t tool_idx = 0; tool_idx < m_session.m_extruders.size(); ++tool_idx) {
        const bool e_position_changed = !new_e_positions.empty() &&
                                        new_e_positions[tool_idx] != old_e_positions[tool_idx];
        const bool retraction_changed = new_retracted[tool_idx] != old_retracted[tool_idx] ||
                                        new_restart_extra[tool_idx] != old_restart_extra[tool_idx];
        if (e_position_changed || retraction_changed) {
            const std::optional<double> e_position = e_position_changed ?
                std::optional<double>(new_e_positions[tool_idx]) : std::optional<double>();
            m_session.m_extruders[tool_idx].extrusion_axis().synchronize_after_external_gcode(
                e_position, new_retracted[tool_idx], new_restart_extra[tool_idx]);
        }
    }
    if (!used_filament_snapshot.empty())
        m_session.m_last_layer_used_filament = std::move(used_filament_snapshot);
    // Event chunks are concatenated directly by the plan writer. Collapse all
    // trailing line breaks to one so an empty line cannot accumulate at every
    // scope boundary and the following command can never join this script.
    while (!output.empty() && (output.back() == '\n' || output.back() == '\r'))
        output.pop_back();
    if (!output.empty())
        output += '\n';
    return prefix + output + (startup ? m_session.write_post_start() : std::string());
}




}}
