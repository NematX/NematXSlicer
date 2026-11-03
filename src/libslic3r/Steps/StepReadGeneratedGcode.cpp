///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "StepReadGeneratedGcode.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>

#include <boost/nowide/fstream.hpp>
#include <boost/log/trivial.hpp>

#include "libslic3r/Api/host/GCodeReaderService.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Utils.hpp"

/*
Generated G-code reading and statistics implementation
=======================================

Only a reusable fixed-size buffer is retained while parsing. Every returned
string_view remains valid until the Reader requests the next chunk, matching
the synchronous Reader callback lifetime and allowing multi-gigabyte files to be processed
without loading them completely into memory.
Print material totals convert consumed mm3 to filament mm, grams and cost using
each logical tool's configured diameter, density and price per kg. The tower
share uses the WipeTower role and does not require object labels. Source paths
and their firmware-selected reader IDs remain owned by Print after export.
*/

namespace Slic3r::Steps::StepReadGeneratedGcode {

// Install the G-code reader service selector preset/category declaration on the supplied host.
void register_selector_traits(Orchestrator &orchestrator);

// Copy output/printer-preset traits into the borrowed host, which owns the strings; no print/configuration data changes.
// Duplicate/conflicting installation asserts and allocation errors propagate without transferring provider ownership.
void register_selector_traits(Orchestrator &orchestrator)
{
    StepSelectorTraits traits;
    traits.category = RAW_OPTION_CATEGORY_OUTPUT;
    traits.option_preset_type = RAW_PRESET_TYPE_FFF_PRINTER;
    const int32_t registered = orchestrator.register_builtin_step_selector(GCODE_READER, std::move(traits));
    assert(registered == 1);
    (void)registered;
}

// Borrow the host/Print and reader ID, stream the Print-owned source into an
// owned result, then replace the caller's result only on success. The source
// is not modified; missing paths, I/O, reader failures and cancellation propagate.
void run(Orchestrator &orchestrator,
         Print &print,
         std::string_view reader_plugin_id,
         GCodeProcessorResult &result)
{
    const std::string source_path = print.gcode_reader_source_path();
    if (source_path.empty())
        throw std::runtime_error("The G-code writer did not publish a Reader source path.");

    boost::nowide::ifstream stream(source_path.c_str(), std::ios::in | std::ios::binary);
    if (!stream)
        throw std::runtime_error("Cannot open the G-code Reader source file '" + source_path + "'.");

    std::array<char, 64u * 1024u> buffer = {};
    bool finished = false;
    GCodeReaderChunkSource source = [&stream, &buffer, &finished]() -> std::string_view {
        if (finished)
            return {};

        stream.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        const std::streamsize count = stream.gcount();
        if (count > 0)
            return std::string_view(buffer.data(), static_cast<size_t>(count));
        if (stream.bad() || !stream.eof())
            throw std::runtime_error("Failed while reading the G-code Reader source file.");

        finished = true;
        return {};
    };

    GCodeReaderService reader(orchestrator, &print);
    GCodeProcessorResult built = reader.read_for_viewer(reader_plugin_id, std::move(source));
    built.filename = source_path;
    result = std::move(built);
}

// Borrow a generated reader result and its matching print config; convert each
// logical tool's volume with configured filament properties and set only time,
// filament totals and tower shares. Preserve source areas and export metadata;
// positive diameters are required and allocation/formatting failures propagate.
void fill_print_statistics(const GCodeProcessorResult &result, const PrintConfig &config,
                           PrintStatistics &stats)
{
    const uint8_t normal = static_cast<uint8_t>(PrintEstimatedStatistics::ETimeMode::Normal);
    const float time = result.print_statistics.modes[normal].time;
    stats.estimated_print_time.clear();
    stats.estimated_print_time_str.clear();
    stats.estimated_print_time[normal] = time;
    stats.estimated_print_time_str[normal] = get_time_dhms(time);
    stats.filament_stats = result.print_statistics.volumes_per_extruder;
    stats.total_extruded_volume = 0.0;
    stats.total_used_filament = 0.0;
    stats.total_weight = 0.0;
    stats.total_cost = config.time_cost.value * (time / 3600.f);
    stats.total_wipe_tower_filament = 0.0;
    stats.total_wipe_tower_filament_weight = 0.0;
    stats.total_wipe_tower_volume = 0.0;
    stats.total_wipe_tower_cost = 0.0;
    const std::map<GCodeExtrusionRole, std::map<extruder_index_t, double>>::const_iterator tower =
        result.print_statistics.volumes_per_role_per_extruder.find(GCodeExtrusionRole::WipeTower);
    for (const auto &[tool, volume] : result.print_statistics.volumes_per_extruder) {
        const double area = PI * std::pow(0.5 * config.filament_diameter.get_at(tool), 2.0);
        const double density = config.filament_density.get_at(tool);
        const double price = config.filament_cost.get_at(tool);
        const double mass = volume * density * 0.001;
        stats.total_extruded_volume += volume;
        stats.total_used_filament += volume / area;
        stats.total_weight += mass;
        stats.total_cost += mass * price * 0.001;
        if (tower != result.print_statistics.volumes_per_role_per_extruder.end()) {
            const std::map<extruder_index_t, double>::const_iterator found = tower->second.find(tool);
            if (found != tower->second.end()) {
                const double tower_mass = found->second * density * 0.001;
                stats.total_wipe_tower_filament += found->second / area;
                stats.total_wipe_tower_filament_weight += tower_mass;
                stats.total_wipe_tower_volume += found->second;
                stats.total_wipe_tower_cost += tower_mass * price * 0.001;
            }
        }
    }
}

// Borrow a completed Print whose source and selected reader are still published;
// read a local owned viewer result and fill its time/material totals. Keep the
// source file and other statistics intact. Cancellation propagates; other read
// failures produce a non-critical warning rather than failing the exported artifact.
void read_statistics(Orchestrator &orchestrator, Print &print)
{
    std::string warning;
    try {
        GCodeProcessorResult result;
        run(orchestrator, print, print.gcode_reader_plugin_id(), result);
        fill_print_statistics(result, print.config(), print.print_statistics());
        return;
    } catch (const CanceledException &) {
        throw;
    } catch (const std::exception &error) {
        warning = "G-code statistics are unavailable because the generated text could not be read: " +
                  std::string(error.what());
    } catch (...) {
        warning = "G-code statistics are unavailable because the generated text reader failed unexpectedly.";
    }
    BOOST_LOG_TRIVIAL(warning) << warning;
    Print::StatusMonitor status(print);
    status.active_step_add_warning(PrintStateBase::WarningLevel::NON_CRITICAL, warning);
}

} // namespace Slic3r::Steps::StepReadGeneratedGcode
