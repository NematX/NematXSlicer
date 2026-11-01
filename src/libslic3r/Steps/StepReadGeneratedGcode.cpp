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
#include <vector>

#include <boost/nowide/fstream.hpp>

#include "libslic3r/Api/host/GCodeReaderService.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Utils.hpp"

/*
After command generation, the host can request a Reader pass for a preview and
statistics. This module opens the writer's readable reference, streams chunks
through the selected Reader, and turns its events into the viewer result and
Print totals. It reads commands, not the sliced layers that produced them.

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
Changing the reader selector requires STEP_GCODE to run again and recreate the source file.

Representative direct calls (branches are not a timeline):

    run()
        |-- print.gcode_reader_source_path()
        |-- stream.read()
        `-- reader.read_for_viewer()
*/

namespace Slic3r::Steps::StepReadGeneratedGcode {

// Register reader selector metadata with the plugin manager; changing the selector requires G-code generation to run again.
void register_selector_traits(Orchestrator &orchestrator);

void register_selector_traits(Orchestrator &orchestrator)
{
    StepSelectorTraits traits;
    traits.category = RAW_OPTION_CATEGORY_OUTPUT;
    traits.option_preset_type = RAW_PRESET_TYPE_FFF_PRINTER;
    traits.invalidates_step = STEP_GCODE;
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
    // Lower the flag before anything can fail: a reread that does not complete leaves the statistics unavailable.
    print.print_statistics().statistics_available = false;
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
// the time of layer 0, filament totals, tower shares and the support flag. Preserve source areas and the
// plan facts; positive diameters are required and allocation/formatting failures propagate.
// The availability flag is lowered first and raised only after every value is set,
// so a failure leaves the statistics marked unavailable.
void fill_print_statistics(const GCodeProcessorResult &result, const PrintConfig &config,
                           PrintStatistics &stats)
{
    stats.statistics_available = false;
    const uint8_t normal = static_cast<uint8_t>(PrintEstimatedStatistics::ETimeMode::Normal);
    const float time = result.print_statistics.modes[normal].time;
    stats.estimated_print_time.clear();
    stats.estimated_print_time_str.clear();
    stats.estimated_print_time[normal] = time;
    stats.estimated_print_time_str[normal] = get_time_dhms(time);
    // Layer 0 of the Reader includes the moves before the first layer; a result without layer times gives 0, never the
    // value of a previous reading.
    const std::vector<float> &layer_times = result.print_statistics.modes[normal].layers_times;
    stats.first_layer_time = layer_times.empty() ? 0. : double(layer_times.front());
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
    // A role is listed in used_filaments_per_role only when the Reader saw material extruded with it.
    stats.has_support =
        result.print_statistics.used_filaments_per_role.count(GCodeExtrusionRole::SupportMaterial) != 0 ||
        result.print_statistics.used_filaments_per_role.count(GCodeExtrusionRole::SupportMaterialInterface) != 0;
    stats.statistics_available = true;
}

} // namespace Slic3r::Steps::StepReadGeneratedGcode
