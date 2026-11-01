///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_host_GCodeExportMetadata_hpp_
#define slic3r_Api_host_GCodeExportMetadata_hpp_

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

/*
G-code export metadata
======================

This host-side module serializes export configuration entries and calculates
extrusion widths from the first Print object's active region settings, and
computes the print statistics of an export once: as numbers (seconds, millimeters,
cubic millimeters, grams, unrounded) and as the formatted text lines that are
formed from those same numbers. The configuration keys written are those whose
definition nature allows the G-code destination. The configuration filter is also used by the
G-code generator's saved settings.

The encode_gcode_export_* functions prepare configuration, width and statistics
values for the host's export payload. Keep their returned values alive while
building and lending the C arrays and string pointers: those pointers do not
take ownership. Exporters decide how to arrange these values in a file.
*/

namespace Slic3r {

class DynamicPrintConfig;
class Print;

// Measured material of one tool, from the Reader volume and the filament diameter and density of the export configuration.
struct GCodeExportFilamentUsage
{
    size_t tool = 0;
    double length_mm = 0.;
    double volume_mm3 = 0.;
    double weight_g = 0.;
};

// The statistics of one export. The plan facts (layer_count, max_print_z) are those that STEP_GCODE published into the Print statistics, zero until then
// (also for a direct export of a reference that STEP_GCODE did not produce, even when the Print has a plan); the
// measured values (times, filaments, totals, text entries) only when the Print has available statistics, else they stay
// empty or zero. Owned by value: it holds no pointer into the Print.
struct GCodeExportStatistics
{
    // Label/value pairs in file order, exactly as the text file lists them.
    std::vector<std::pair<std::string, std::string>> entries;
    // Nominal path time of the Reader in seconds (no acceleration), and the seconds attributed to its layer 0.
    double estimated_time_seconds = 0.;
    double first_layer_time_seconds = 0.;
    // Layers of the plan, auxiliary layers included, and the highest layer Z of the plan in millimeters (no Z offset).
    int32_t layer_count = 0;
    double max_print_z = 0.;
    // One element per tool of the export configuration, in tool order.
    std::vector<GCodeExportFilamentUsage> filaments;
    // Sums of the elements of filaments.
    double total_length_mm = 0.;
    double total_volume_mm3 = 0.;
    double total_weight_g = 0.;
};

// Append all keys that the G-code destination allows, except disabled optional settings, in key order; existing output entries are retained.
void encode_gcode_export_configuration(const DynamicPrintConfig &config,
                                       std::vector<std::pair<std::string, std::string>> &entries);

// Calculate widths from the first Print object's region settings and return owned G-code role labels/values with an empty pair after each region.
std::vector<std::pair<std::string, std::string>> encode_gcode_export_extrusion_widths(const Print &print);

// Compute the statistics of the Print once, using the configuration copy for the tool count (its nozzle diameters; measured tools beyond it are ignored) and filament properties; the measured part stays empty or zero when the Print has no available statistics.
GCodeExportStatistics encode_gcode_export_statistics(const Print &print, const DynamicPrintConfig &config);

} // namespace Slic3r

#endif // slic3r_Api_host_GCodeExportMetadata_hpp_
