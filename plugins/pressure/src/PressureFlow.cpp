///|/ Released under the GNU Affero General Public License, version 3 or later.

#include "PressureFlow.hpp"
#include "PressureRewrite.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string_view>
#include <utility>

// Provides ordered pressure processing to a caller that already owns Reader
// results. Nominal and classification components prove each source motion; model
// controls are read even outside layers. Planner keeps all original movements,
// profiles and caps. Projection prepares emission groups only after planning.
// Cross-source material transfers change these groups, never planner inputs.
// Each group retains internal original-source phase parts, while public geometry
// indices remain local to the row that supplies its text (the anchor).
//
// Rows share immutable fragment storage until their last use. The last spatially
// representable source is retained, not just the last planner movement: a later
// collapsed suffix can still merge backward before any E state advances. Empty
// and effect-free comments may separate transfer partners; controls and Reader
// effects cannot. One persistent Extrusion per tool preserves relative rounding.
// Calls are sequential and not thread safe. Failure abandons mutated state, but
// the local output never escapes from the throwing call.
//
// Call tree:
//   consume: follow nominal, classification, controls and effective modes in order.
//     finish / publication: close or receive original planner results and project.
//       resolve: attach collapsed material to compatible, still-unemitted groups.
//         compatible: require settings and spatial/nominal continuity.
//       emit: advance per-tool E once per final group, rewrite anchor or residue.
//         append: separate real text across source records and returned fragments.
//     modes: schedule coordinate-frame rebasing only for effective mode changes.

namespace nematx_pressure {
namespace {
// Returns a view of one record in source.text without changing its bytes.
// Keep source.text alive and unchanged while using the returned view.
std::string_view record(const SourceFragment &source, std::size_t row);
// Combines group endpoints in emission order while retaining the anchor's indices.
GeometryGroup merged(const GeometryGroup &first, const GeometryGroup &last,
                     const GeometryGroup &anchor);
} // namespace

Flow::Flow(const FlowConfig &config) : m_config(config), m_controls{std::nullopt, config.initial_acceleration}
{
    if (config.tools.empty() || config.tools.size() != config.axis_letters.size() ||
        config.xyz_digits < 0 || config.xyz_digits > 9 || config.e_digits < 0 || config.e_digits > 9 ||
        !std::isfinite(config.initial_acceleration) || config.initial_acceleration <= 0)
        throw std::invalid_argument("Invalid resolved pressure Flow configuration.");
    for (const FlowTool &tool : config.tools) {
        if (!std::isfinite(tool.pressure_factor) || tool.pressure_factor < 0)
            throw std::invalid_argument("Invalid configured pressure factor.");
        m_tools.push_back({Extrusion(config.e_digits, 0.)});
    }
}

FlowOutput Flow::consume(SourceFragment source, const std::vector<MoveHint> &hints, FlowBoundary boundary)
{
    if (m_failed || m_ended)
        throw std::logic_error("Pressure Flow is no longer usable.");
    FlowOutput output;
    try {
        if (boundary.end_print) {
            finish(output);
            m_in_layer = false;
            m_classification.cycle.reset();
        } else if (boundary.begin_layer) {
            m_in_layer = true;
        }
        const NominalFragment nominal = read_nominal_fragment(source, m_nominal, m_config.axis_letters);
        ClassificationContext context{m_in_layer, m_config.volumetric, {}};
        for (const FlowTool &tool : m_config.tools)
            context.enabled_tools.push_back(tool.pressure_factor > 0 || tool.annotate);
        const ClassificationFragment classified = classify_fragment(source, nominal, hints, context, m_classification);
        const std::shared_ptr<const SourceFragment> owned = std::make_shared<const SourceFragment>(std::move(source));
        slic3r_api::GCodeStateTracker state = owned->reader.initial;
        for (std::size_t row = 0; row < owned->lines.size(); ++row) {
            const std::size_t ordinal = m_next_ordinal++;
            const LineDecision &decision = classified.lines[row];
            if (decision.close_before)
                finish(output);
            const ControlResult control = read_control_line(record(*owned, row), row, m_controls);
            m_controls = control.next;
            if (control.issue)
                output.issues.push_back({FlowIssueKind::InvalidAcceleration, ordinal, row, state.active_tool(), {}});
            const SourceLine &line = owned->lines[row];
            for (std::size_t index = line.first_event; index < line.first_event + line.event_count; ++index) {
                const ReaderEvent &event = owned->reader.events[index];
                if (event.kind == RAW_GCODE_READER_EVENT_INVALIDATE || event.kind == RAW_GCODE_READER_EVENT_DIAGNOSTIC)
                    output.issues.push_back({event.kind == RAW_GCODE_READER_EVENT_INVALIDATE ?
                        FlowIssueKind::ReaderInvalidation : FlowIssueKind::ReaderDiagnostic,
                        ordinal, row, state.active_tool(), event.items});
            }
            const std::optional<NominalMotion> motion = decision.motion_index ?
                std::optional<NominalMotion>(nominal.motions[*decision.motion_index]) : std::nullopt;
            std::optional<NominalMotion> mode_motion = motion;
            if (!mode_motion) {
                for (const NominalMotion &candidate : nominal.motions) {
                    if (candidate.event_index >= line.first_event && candidate.event_index < line.first_event + line.event_count &&
                        (candidate.start != candidate.end || (candidate.e_delta && *candidate.e_delta != 0))) {
                        mode_motion = candidate;
                        break;
                    }
                }
            }
            modes(*owned, row, mode_motion, state);
            // Side-effect-free comments and blank records alone permit material
            // transfer; equal-value controls still retain their ordered effect.
            const std::string_view text = record(*owned, row);
            const std::size_t first = text.find_first_not_of(" \t\r\n");
            const bool transfer_transparent = line.event_count == 0 &&
                text.substr(0, 18) != "; PRESSURE_FACTOR=" && text.substr(0, 7) != ";M204 S" &&
                (first == std::string_view::npos || (text[first] == ';' && text.substr(first, 6) != ";TYPE:"));
            Row pending{owned, row, ordinal, transfer_transparent, std::nullopt};
            if (motion) {
                const std::uint32_t tool = *motion->active_tool;
                ToolState &e = m_tools.at(tool);
                const bool absolute = *motion->e_absolute;
                const double start = absolute ? *motion->e_start : (e.reset_frame ? 0. : e.sum);
                const double end = absolute ? *motion->e_end : start + *motion->e_delta;
                const bool rebase = e.reset_frame || (absolute && (!e.nominal_end || *e.nominal_end != start));
                e.reset_frame = false;
                e.sum = end;
                e.nominal_end = end;
                const SourceGeometry geometry{{*motion->start[0], *motion->start[1], *motion->start[2]},
                    {*motion->end[0], *motion->end[1], *motion->end[2]}, start, end};
                const FlowTool &settings = m_config.tools[tool];
                const double k = m_controls.pressure_override.value_or(settings.pressure_factor);
                pending.print = Print{*motion, geometry, k, *motion->feedrate, settings.annotate, rebase};
                m_rows.push_back(std::move(pending));
                const Move move = normalize_move(geometry.end[0] - geometry.start[0],
                    geometry.end[1] - geometry.start[1], geometry.end[2] - geometry.start[2],
                    *motion->e_delta, *motion->feedrate / 60., m_controls.acceleration, m_config.limits, ordinal);
                publication(m_planner.push(move), false, output);
            } else {
                m_rows.push_back(std::move(pending));
                // Side effects do not stop the pressure planner, but they make a
                // preceding collapsed suffix choose its still-retained left anchor.
                resolve(false);
                emit(false, output);
            }
        }
        m_nominal = nominal.final;
        m_classification = classified.final;
        if (boundary.end_layer || boundary.end_print) {
            finish(output);
            m_in_layer = false;
            m_classification.cycle.reset();
        }
        m_ended = boundary.end_print;
        return output;
    } catch (...) {
        m_failed = true;
        throw;
    }
}

void Flow::publication(Publication published, bool final, FlowOutput &output)
{
    for (const PlannedMove &planned : published.moves) {
        const std::deque<Row>::iterator found = std::find_if(m_rows.begin(), m_rows.end(),
            [&planned](const Row &row) { return row.ordinal == planned.move.source_index; });
        if (found == m_rows.end() || !found->print)
            throw std::logic_error("Pressure planner source is not retained.");
        Print &print = *found->print;
        print.planned = planned;
        print.geometry = project_geometry(planned, print.source_geometry, m_config.xyz_digits);
        bool resolution_loss = print.geometry.status == GeometryStatus::EntireSourceCollapsed;
        for (std::size_t group = 0; group < print.geometry.count; ++group) {
            const GeometryGroup &geometry = print.geometry.groups[group];
            print.parts[group].push_back({found->ordinal, geometry.first_phase, geometry.phase_count});
            resolution_loss = resolution_loss || geometry.kind == SegmentKind::Merged;
        }
        if (resolution_loss)
            output.issues.push_back({FlowIssueKind::GeometryMerged, found->ordinal, found->local, print.motion.active_tool, {}});
    }
    if (published.horizon_insufficient && !m_rows.empty())
        output.issues.push_back({FlowIssueKind::HorizonInsufficient, m_rows.back().ordinal, m_rows.back().local,
            m_rows.back().print ? m_rows.back().print->motion.active_tool : std::nullopt, {}});
    resolve(final);
    emit(final, output);
}

void Flow::resolve(bool final)
{
    for (std::size_t index = 0; index < m_rows.size(); ++index) {
        Row &row = m_rows[index];
        if (!row.print || !row.print->planned || row.print->transferred ||
            row.print->geometry.status != GeometryStatus::EntireSourceCollapsed)
            continue;
        Print &collapsed = *row.print;
        std::size_t next = index + 1;
        while (next < m_rows.size() && !m_rows[next].print && m_rows[next].transfer_transparent)
            ++next;
        // Wait for a future profile only while a forward partner can still arrive.
        if (next == m_rows.size() && !final)
            continue;
        if (next < m_rows.size() && m_rows[next].print && compatible(collapsed, *m_rows[next].print)) {
            Print &receiver = *m_rows[next].print;
            if (!receiver.planned)
                continue;
            receiver.geometry.groups[0] = merged(collapsed.geometry.groups[0], receiver.geometry.groups[0], receiver.geometry.groups[0]);
            receiver.parts[0].insert(receiver.parts[0].begin(), collapsed.parts[0].begin(), collapsed.parts[0].end());
            // Rebase belongs before the first transferred material, not its residue.
            receiver.rebase = receiver.rebase || collapsed.rebase;
            collapsed.transferred = true;
            continue;
        }
        std::size_t previous = index;
        while (previous > 0) {
            --previous;
            if (m_rows[previous].print && m_rows[previous].print->transferred)
                continue;
            if (m_rows[previous].print || !m_rows[previous].transfer_transparent)
                break;
        }
        if (previous == index || !m_rows[previous].print || !m_rows[previous].print->planned ||
            m_rows[previous].print->geometry.status != GeometryStatus::Representable ||
            !compatible(*m_rows[previous].print, collapsed))
            throw std::invalid_argument("Collapsed pressure source at local row " + std::to_string(row.local) +
                ", tool " + std::to_string(*collapsed.motion.active_tool) +
                " has no compatible un-emitted spatial neighbour.");
        Print &anchor = *m_rows[previous].print;
        const std::size_t last = anchor.geometry.count - 1;
        anchor.geometry.groups[last] = merged(anchor.geometry.groups[last], collapsed.geometry.groups[0], anchor.geometry.groups[last]);
        anchor.parts[last].insert(anchor.parts[last].end(), collapsed.parts[0].begin(), collapsed.parts[0].end());
        collapsed.transferred = true;
    }
}

void Flow::emit(bool final, FlowOutput &output)
{
    std::optional<std::size_t> hold;
    if (!final) {
        for (std::size_t index = 0; index < m_rows.size(); ++index) {
            const std::optional<Print> &print = m_rows[index].print;
            if (print && print->planned && !print->transferred && print->geometry.status == GeometryStatus::Representable)
                hold = index;
        }
    }
    std::size_t count = 0;
    for (Row &row : m_rows) {
        if (hold && count == *hold)
            break;
        std::string text;
        if (!row.print) {
            text = std::string(record(*row.source, row.local));
        } else {
            Print &print = *row.print;
            if (!print.planned || (print.geometry.status == GeometryStatus::EntireSourceCollapsed && !print.transferred))
                break;
            if (print.transferred) {
                text = rewrite_residue(*row.source, row.local, print.motion, m_config.axis_letters);
            } else {
                Extrusion &e = m_tools[*print.motion.active_tool].extrusion;
                if (print.rebase)
                    e.rebase(print.geometry.groups[0].nominal_start);
                std::array<ExtrusionResult, 3> extrusion{};
                for (std::size_t group = 0; group < print.geometry.count; ++group)
                    extrusion[group] = e.advance(print.geometry.groups[group].nominal_end,
                        print.geometry.groups[group].speed_end, print.k);
                text = rewrite_print(*row.source, row.local, print.motion, print.geometry, extrusion,
                    print.k, m_config.axis_letters, {m_config.xyz_digits, m_config.e_digits, print.annotate});
            }
        }
        append(std::move(text), output);
        ++count;
    }
    m_rows.erase(m_rows.begin(), m_rows.begin() + count);
    if (final && !m_rows.empty())
        throw std::logic_error("Pressure closure left an unresolved source.");
}

void Flow::append(std::string text, FlowOutput &output)
{
    if (text.empty())
        return;
    if (m_unterminated)
        output.bytes += '\n';
    output.bytes += text;
    m_unterminated = text.back() != '\n';
}

bool Flow::compatible(const Print &first, const Print &last) const
{
    if (first.motion.active_tool != last.motion.active_tool || first.k != last.k || first.feed != last.feed ||
        first.motion.e_absolute != last.motion.e_absolute || last.rebase)
        return false;
    if (!first.planned || !last.planned)
        return true;
    const GeometryGroup &end = first.geometry.groups[first.geometry.count - 1];
    const GeometryGroup &start = last.geometry.groups[0];
    return end.end == start.start && end.nominal_end == start.nominal_start;
}

void Flow::finish(FlowOutput &output)
{
    publication(m_planner.finish(), true, output);
}

void Flow::modes(const SourceFragment &source, std::size_t row,
                 const std::optional<NominalMotion> &motion, slic3r_api::GCodeStateTracker &state)
{
    const SourceLine &line = source.lines[row];
    bool changed_e = false, changed_xyz = false, changed_units = false;
    for (std::size_t index = line.first_event; index < line.first_event + line.event_count; ++index) {
        const ReaderEvent &event = source.reader.events[index];
        for (const raw_gcode_reader_event_item &item : event.items) {
            changed_e = changed_e || item.type == RAW_GCODE_READER_ITEM_E_ABSOLUTE;
            changed_xyz = changed_xyz || item.type == RAW_GCODE_READER_ITEM_XYZ_ABSOLUTE;
            changed_units = changed_units || item.type == RAW_GCODE_READER_ITEM_UNITS_MM;
        }
        state.apply({event.kind, event.items.data(), std::uint32_t(event.items.size())});
    }
    const std::optional<bool> e = motion ? motion->e_absolute : state.e_is_absolute();
    const std::optional<bool> xyz = motion ? motion->xyz_absolute : state.xyz_is_absolute();
    const std::optional<bool> mm = motion ? motion->units_mm : state.units_are_millimetres();
    if (m_modes && ((e && (motion || changed_e) && *e != m_modes->e_absolute) ||
        (xyz && (motion || changed_xyz) && *xyz != m_modes->xyz_absolute) ||
        (mm && (motion || changed_units) && *mm != m_modes->units_mm))) {
        for (ToolState &tool : m_tools)
            tool.reset_frame = true;
    }
    if (!m_modes && e && xyz && mm)
        m_modes = CycleSnapshot{state.active_tool().value_or(0), *xyz, *e, *mm};
    else if (m_modes) {
        // A comment following an inline G91 must not replace the last effective
        // printing XYZ mode with that row's residual parser mode.
        if (e && (motion || changed_e)) m_modes->e_absolute = *e;
        if (xyz && (motion || changed_xyz)) m_modes->xyz_absolute = *xyz;
        if (mm && (motion || changed_units)) m_modes->units_mm = *mm;
    }
}

namespace {
std::string_view record(const SourceFragment &source, std::size_t row)
{
    const SourceLine &line = source.lines.at(row);
    return std::string_view(source.text).substr(line.offset, line.length);
}

GeometryGroup merged(const GeometryGroup &first, const GeometryGroup &last, const GeometryGroup &anchor)
{
    return {first.start, last.end, first.nominal_start, last.nominal_end,
        first.distance + last.distance, first.duration + last.duration,
        first.speed_start, last.speed_end, anchor.first_phase, anchor.phase_count, SegmentKind::Merged};
}
} // namespace
} // namespace nematx_pressure
